import torch
import torch.nn as nn
import torch.nn.functional as F
# 从torch 引入 resnet18 并加载预训练参数
from torchvision.models import resnet18
from torchvision.models import ResNet18_Weights


# 图像嵌入层，使用简单 CNN
class ImageEmbedding(nn.Module):
    def __init__(self, embed_dim, num_layers=3, dropout_rate=0.5, is_resnet=False):
        super(ImageEmbedding, self).__init__()
        self.cnn_layers = nn.ModuleList()
        self.resnet = resnet18(weights=ResNet18_Weights.IMAGENET1K_V1)
        self.resnet.fc = nn.Linear(self.resnet.fc.in_features, embed_dim)
        self.is_resnet = is_resnet

        in_channels = 3
        out_channels = 8

        # 使用 num_layers 层的 CNN, no residuals
        for _ in range(num_layers):
            self.cnn_layers.append(nn.Conv2d(in_channels, out_channels, kernel_size=3, padding=1))
            self.cnn_layers.append(nn.BatchNorm2d(out_channels))
            self.cnn_layers.append(nn.ReLU())
            self.cnn_layers.append(nn.MaxPool2d(kernel_size=2, stride=2))
            self.cnn_layers.append(nn.Dropout2d(dropout_rate))
            in_channels = out_channels
            out_channels *= 2
        # 计算经过 CNN 后图像的尺寸
        h, w = 256 // (2 ** num_layers), 256 // (2 ** num_layers)
        self.fc = nn.Linear(in_channels * h * w, embed_dim)

    def forward(self, images):
        # 输入 images 形状为 (batch_size, seq_length, 2, 3, 256, 256)
        batch_size, seq_length, num_cameras, _, _, _ = images.shape
        # 合并 batch、seq 和 camera 维度
        images = images.view(-1, 3, 256, 256)
        out = images
        if not self.is_resnet:
            for layer in self.cnn_layers:
                out = layer(out)
            out = out.view(batch_size * seq_length * num_cameras, -1)
            embedded = self.fc(out)
        else:
            embedded = self.resnet(out)
        # 恢复 batch 和 seq 维度
        embedded = embedded.view(batch_size, seq_length, num_cameras, -1)
        # 对两个相机的特征进行拼接
        embedded = embedded.view(batch_size, seq_length, -1)
        return embedded


# 电机数据嵌入层，使用几层全连接
class MotorEmbedding(nn.Module):
    def __init__(self, motor_dim=12, embed_dim=128, num_fc_layers=3, dropout_rate=0.2):
        super(MotorEmbedding, self).__init__()
        self.fc_layers = nn.ModuleList()
        in_dim = motor_dim
        hidden_dim = 64

        for _ in range(num_fc_layers - 1):
            self.fc_layers.append(nn.Linear(in_dim, hidden_dim))
            self.fc_layers.append(nn.ReLU())
            self.fc_layers.append(nn.Dropout(dropout_rate))
            in_dim = hidden_dim

        self.fc_layers.append(nn.Linear(in_dim, embed_dim))

    def forward(self, motor_data):
        for layer in self.fc_layers:
            motor_data = layer(motor_data)
        return motor_data


# 位置编码
class PositionalEncoding(nn.Module):
    def __init__(self, d_model, max_len=5000):
        super(PositionalEncoding, self).__init__()
        pe = torch.zeros(max_len, d_model)
        position = torch.arange(0, max_len, dtype=torch.float).unsqueeze(1)
        div_term = torch.exp(torch.arange(0, d_model, 2).float() * (-torch.log(torch.tensor(10000.0)) / d_model))
        pe[:, 0::2] = torch.sin(position * div_term)
        pe[:, 1::2] = torch.cos(position * div_term)
        self.register_buffer('pe', pe.unsqueeze(0))

    def forward(self, x):
        # 只考虑序列的时间步，忽略相机数量维度
        seq_length = x.size(1)
        # 为每个时间步复制相同的位置编码
        pe = self.pe[:, :seq_length, :].repeat(x.size(0), 1, 1)
        x = x + pe
        return x


# Transformer 编码器
class TransformerEncoderModel(nn.Module):
    def __init__(self, embed_dim, nhead, num_layers):
        super(TransformerEncoderModel, self).__init__()
        encoder_layer = nn.TransformerEncoderLayer(d_model=embed_dim * 3, nhead=nhead, batch_first=True)
        self.transformer_encoder = nn.TransformerEncoder(encoder_layer, num_layers=num_layers)

    def forward(self, src):
        return self.transformer_encoder(src)


# Transformer 解码器
class TransformerDecoderModel(nn.Module):
    def __init__(self, embed_dim, nhead, num_layers, motor_dim):
        super(TransformerDecoderModel, self).__init__()
        decoder_layer = nn.TransformerDecoderLayer(d_model=embed_dim * 3, nhead=nhead, batch_first=True)
        self.transformer_decoder = nn.TransformerDecoder(decoder_layer, num_layers=num_layers)
        self.fc = nn.Linear(embed_dim * 3, motor_dim)

    def forward(self, tgt, memory):
        tgt_mask = torch.nn.Transformer.generate_square_subsequent_mask(tgt.size(1)).to(tgt.device)
        output = self.transformer_decoder(tgt, memory, tgt_mask=tgt_mask)
        return self.fc(output)


# 整体模型
class MultimodalTransformer(nn.Module):
    def __init__(self, embed_dim, nhead, num_layers, output_seq_length, max_seq_length):
        super(MultimodalTransformer, self).__init__()
        self.positional_encoding = PositionalEncoding(embed_dim * 3, max_seq_length)
        self.transformer_encoder = TransformerEncoderModel(embed_dim, nhead, num_layers)
        self.transformer_decoder = TransformerDecoderModel(embed_dim, nhead, num_layers, 12)
        self.output_seq_length = output_seq_length

    def forward(self, image_embedded, motor_embedded, tgt_embed, num_candidates=5, temperature=0.8):
        # 拼接图像和电机信号的嵌入
        combined_embedded = torch.cat([motor_embedded, image_embedded], dim=-1)

        # 应用位置编码
        combined_embedded = self.positional_encoding(combined_embedded)

        # 编码器计算
        memory = self.transformer_encoder(combined_embedded)
        tgt = self.positional_encoding(tgt_embed).to(memory.device)

        candidates = []
        for _ in range(num_candidates):
            # 初始化解码器输入
            # tgt = torch.zeros((motor_embedded.size(0), self.output_seq_length, motor_embedded.size(-1) * 3)).to(
                # motor_embedded.device)

            for t in range(self.output_seq_length):
                if t > 0:
                    tgt_input = tgt[:, :t, :]
                else:
                    tgt_input = tgt[:, :1, :]
                output = self.transformer_decoder(tgt_input, memory)
                last_output = output[:, -1, :]
                probs = F.softmax(last_output / temperature, dim=-1)
                next_token = torch.multinomial(probs, num_samples=1)
                tgt[:, t, :] = next_token.squeeze(1)
            candidates.append(tgt)
        return candidates


# 示例使用
if __name__ == "__main__":
    import time
    embed_dim = 32
    nhead = 4
    num_layers = 10
    max_seq_length = 10
    output_seq_length = 5

    image_embedding = ImageEmbedding(embed_dim, is_resnet=False)
    motor_embedding = MotorEmbedding(embed_dim=embed_dim)
    multimodal_transformer = MultimodalTransformer(embed_dim, nhead, num_layers, output_seq_length, max_seq_length)

    # 输入图像形状为 (batch_size, seq_length, 2, 3, 256, 256)
    images = torch.randn(1, max_seq_length, 2, 3, 256, 256)
    # 输入电机数据形状为 (batch_size, seq_length, 12)
    motor_data = torch.randn(1, max_seq_length, 12)

    start_time = time.time()
    for i in range(10):
        image_embedded = image_embedding(images)
        motor_embedded = motor_embedding(motor_data)

        tgt_embed = torch.zeros((1, output_seq_length, embed_dim * 3)).to(motor_data.device)
        candidates = multimodal_transformer(image_embedded, motor_embedded, tgt_embed, num_candidates=1, temperature=0.8)
        # print(f"Generated {len(candidates)} candidates.")
    print(f"Time taken for 10 iterations: {time.time() - start_time:.2f} seconds")

    # 计算模型大小的函数
    def get_model_size(model):
        param_size = 0
        for param in model.parameters():
            param_size += param.nelement() * param.element_size()
        buffer_size = 0
        for buffer in model.buffers():
            buffer_size += buffer.nelement() * buffer.element_size()
        size_all_mb = (param_size + buffer_size) / 1024 ** 2
        return size_all_mb


    # 计算各模型的大小
    image_embedding_size = get_model_size(image_embedding)
    motor_embedding_size = get_model_size(motor_embedding)
    transformer_encoder_size = get_model_size(multimodal_transformer.transformer_encoder)
    transformer_decoder_size = get_model_size(multimodal_transformer.transformer_decoder)
    total_model_size = (
            image_embedding_size + motor_embedding_size + transformer_encoder_size + transformer_decoder_size
    )

    print(f"Image Embedding Size: {image_embedding_size:.2f} MB")
    print(f"Motor Embedding Size: {motor_embedding_size:.2f} MB")
    print(f"Transformer Encoder Size: {transformer_encoder_size:.2f} MB")
    print(f"Transformer Decoder Size: {transformer_decoder_size:.2f} MB")
    print(f"Total Model Size: {total_model_size:.2f} MB")

    # 训练一个 epoch 的代码
    criterion = nn.MSELoss()
    optimizer = torch.optim.Adam(params=[{'params': multimodal_transformer.parameters()},
                                         {'params': image_embedding.parameters()},
                                         {'params': motor_embedding.parameters()}], lr=0.001)
    # optimizer = torch.optim.Adam(multimodal_transformer.parameters(), lr=0.001)

    num_epochs = 10
    time_start = time.time()
    for epoch in range(num_epochs):
        optimizer.zero_grad()

        image_embedded = image_embedding(images)
        motor_embedded = motor_embedding(motor_data)
        tgt_embed = torch.zeros((1, output_seq_length, embed_dim * 3), requires_grad=True).to(motor_data.device)

        # 设置为训练模式
        multimodal_transformer.train()

        candidates = multimodal_transformer(image_embedded, motor_embedded, tgt_embed, num_candidates=1,
                                            temperature=0.8)

        # 创建一个需要梯度的目标张量
        target = torch.randn_like(candidates[0], requires_grad=False)
        loss = criterion(candidates[0], target)

        # 检查损失是否可以计算梯度
        if loss.requires_grad:
            time_loss = time.time()
            loss.backward()
            optimizer.step()
            print(f'Epoch [{epoch + 1}/{num_epochs}], Loss: {loss.item():.4f}')
            print(f'Backward pass took {time.time() - time_loss:.4f} seconds')
        else:
            print("Error: Loss does not require grad. Check model parameters and inputs.")
    print(f"Training completed in {time.time() - time_start:.2f} seconds.")


    def train_model(image_embedding, motor_embedding, multimodal_transformer, num_epochs=5):
        criterion = nn.MSELoss()
        optimizer = torch.optim.Adam(multimodal_transformer.parameters(), lr=0.001)

        # 设备设置
        device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        image_embedding.to(device)
        motor_embedding.to(device)
        multimodal_transformer.to(device)

        # 生成一批训练数据
        batch_size = 16
        train_images = torch.randn(batch_size, max_seq_length, 2, 3, 256, 256).to(device)
        train_motor = torch.randn(batch_size, max_seq_length, 12).to(device)

        # 生成目标数据 - 调整维度为 [batch_size, output_seq_length, 12]
        train_targets = torch.randn(batch_size, output_seq_length, 12).to(device)

        # 记录训练前的模型输出作为基准
        multimodal_transformer.eval()
        with torch.no_grad():
            image_embed = image_embedding(train_images[:1])
            motor_embed = motor_embedding(train_motor[:1])
            tgt_embed = torch.zeros((1, output_seq_length, embed_dim * 3), requires_grad=False).to(device)
            before_output = multimodal_transformer(image_embed, motor_embed, tgt_embed)[0]

        # 开始训练
        print("开始训练...")
        for epoch in range(num_epochs):
            multimodal_transformer.train()
            total_loss = 0

            # 模拟多批次训练
            for i in range(0, batch_size, 4):  # 批次大小为4
                batch_images = train_images[i:i + 4]
                batch_motor = train_motor[i:i + 4]
                batch_targets = train_targets[i:i + 4]

                optimizer.zero_grad()

                # 前向传播
                image_embed = image_embedding(batch_images)
                motor_embed = motor_embedding(batch_motor)

                # 初始化解码器输入
                tgt_embed = torch.zeros((len(batch_images), output_seq_length, embed_dim * 3),
                                        requires_grad=True).to(device)

                # 获取模型输出
                candidates = multimodal_transformer(image_embed, motor_embed, tgt_embed)

                # 确保输出维度匹配 [batch_size, output_seq_length, 12]
                # 这里假设 candidates[0] 是 [batch_size, output_seq_length, embed_dim*3]
                # 需要通过一个线性层映射到12维
                output_projection = nn.Linear(embed_dim * 3, 12).to(device)
                predicted = output_projection(candidates[0])

                # 计算损失
                loss = criterion(predicted, batch_targets)

                # 反向传播和优化
                loss.backward()
                optimizer.step()

                total_loss += loss.item()

            # 打印每轮的平均损失
            avg_loss = total_loss / (batch_size // 4)
            print(f'Epoch {epoch + 1}/{num_epochs}, Loss: {avg_loss:.6f}')

        # 记录训练后的模型输出
        multimodal_transformer.eval()
        with torch.no_grad():
            image_embed = image_embedding(train_images[:1])
            motor_embed = motor_embedding(train_motor[:1])
            tgt_embed = torch.zeros((1, output_seq_length, embed_dim * 3), requires_grad=False).to(device)
            after_output = multimodal_transformer(image_embed, motor_embed, tgt_embed)[0]

        # 计算训练前后输出的差异
        output_diff = torch.norm(before_output - after_output).item()
        print(f"训练前后输出差异: {output_diff:.6f}")

        return avg_loss, output_diff

    embed_dim = 32
    nhead = 4
    num_layers = 1
    max_seq_length = 10
    output_seq_length = 5

    image_embedding = ImageEmbedding(embed_dim, is_resnet=False)
    motor_embedding = MotorEmbedding(embed_dim=embed_dim)
    multimodal_transformer = MultimodalTransformer(embed_dim, nhead, num_layers, output_seq_length, max_seq_length)

    # 训练模型
    final_loss, output_diff = train_model(image_embedding, motor_embedding, multimodal_transformer, num_epochs=5)

    # 判断模型是否有学习
    if output_diff > 1e-3 and final_loss < 1.0:
        print("模型似乎在学习！")
    else:
        print("模型可能没有有效学习，请检查训练设置。")
