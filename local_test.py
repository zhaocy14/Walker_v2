import torch
from torchvision.models import resnet18
from torch import nn
# from rknn.api import RKNN


# ---------------------- Transformer 模型定义与转换 ---------------------- #
def convert_transformer_to_rknn():
    # 定义简单的Transformer模型（Encoder+Decoder）
    class TransformerModel(nn.Module):
        def __init__(self, d_model=512, nhead=8, num_layers=2):
            super().__init__()
            self.encoder_layer = nn.TransformerEncoderLayer(d_model=d_model, nhead=nhead)
            self.transformer_encoder = nn.TransformerEncoder(self.encoder_layer, num_layers=num_layers)
            self.decoder_layer = nn.TransformerDecoderLayer(d_model=d_model, nhead=nhead)
            self.transformer_decoder = nn.TransformerDecoder(self.decoder_layer, num_layers=num_layers)
            self.proj = nn.Linear(d_model, d_model)  # 简单投影层

        def forward(self, src, tgt):
            memory = self.transformer_encoder(src)
            output = self.transformer_decoder(tgt, memory)
            return self.proj(output)

    model = TransformerModel().eval()

    # 生成测试输入
    src = torch.randn(10, 1, 512)  # 序列长度=10, batch=1, 特征维度=512
    tgt = torch.randn(5, 1, 512)  # 目标序列长度=5

    # 导出ONNX
    torch.onnx.export(
        model,
        (src, tgt),
        "transformer.onnx",
        opset_version=13,
        input_names=["src", "tgt"],
        output_names=["output"]
    )

    # 转换为RKNN
    rknn = RKNN()
    rknn.load_onnx(model='transformer.onnx')
    rknn.config(mean_values=[[0]], std_values=[[1]], quantized_dtype='asymmetric_quantized-u8')
    rknn.build(do_quantization=True, dataset="transformer_dataset.txt")
    rknn.export_rknn("transformer.rknn")
    rknn.release()


# ---------------------- ResNet-18 转换 ---------------------- #
def convert_resnet_to_rknn():
    # 加载PyTorch模型
    model = resnet18(pretrained=True).eval()
    dummy_input = torch.randn(1, 3, 224, 224)

    # 导出ONNX
    torch.onnx.export(
        model,
        dummy_input,
        "resnet18.onnx",
        opset_version=11,
        input_names=["input"],
        output_names=["output"]
    )

    # 转换为RKNN
    rknn = RKNN()
    rknn.load_onnx(model='resnet18.onnx')
    rknn.config(mean_values=[[0, 0, 0]], std_values=[[255, 255, 255]], quantized_input_type='float32')
    rknn.build(do_quantization=True, dataset="resnet_dataset.txt")
    rknn.export_rknn("resnet18.rknn")
    rknn.release()


if __name__ == "__main__":
    convert_transformer_to_rknn()
    convert_resnet_to_rknn()
