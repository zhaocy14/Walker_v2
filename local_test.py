import os
import time
import numpy as np
import torch
import torchvision.models as models
from rknn.api import RKNN

# 设置测试参数
WARMUP_ROUNDS = 10  # 预热轮数
TEST_ROUNDS = 100  # 测试轮数
IMAGE_SIZE = (224, 224)  # 图像尺寸


def generate_random_image():
    """生成随机图像数据作为输入"""
    # 生成范围在[0, 255]的随机像素值
    img = np.random.randint(0, 256, size=(1, *IMAGE_SIZE, 3), dtype=np.uint8)
    return img


def export_pytorch_model():
    """导出PyTorch模型为TorchScript格式"""
    if not os.path.exists('./resnet18.pt'):
        net = models.resnet18(pretrained=True)
        net.eval()
        trace_model = torch.jit.trace(net, torch.rand(1, 3, 224, 224))
        trace_model.save('./resnet18.pt')
        print("PyTorch模型导出成功")


def get_rknn_model():
    """获取并初始化RKNN模型"""
    if not os.path.exists('./resnet_18.rknn'):
        print("RKNN模型不存在，请先转换模型")
        return None

    rknn = RKNN(verbose=False)

    # 加载RKNN模型
    ret = rknn.load_rknn('./resnet_18.rknn')
    if ret != 0:
        print("加载RKNN模型失败")
        return None

    # 初始化运行环境
    ret = rknn.init_runtime()
    if ret != 0:
        print("初始化RKNN运行环境失败")
        return None

    return rknn


def get_torch_model():
    """获取并初始化PyTorch模型"""
    if not os.path.exists('./resnet18.pt'):
        export_pytorch_model()

    # 加载TorchScript模型
    model = torch.jit.load('./resnet18.pt')
    model.eval()
    return model


def test_rknn_inference_speed(rknn_model, image):
    """测试RKNN模型的推理速度"""
    print("开始RKNN模型预热...")
    for _ in range(WARMUP_ROUNDS):
        rknn_model.inference(inputs=[image], data_format=['nhwc'])

    print("开始RKNN模型性能测试...")
    start_time = time.time()
    for _ in range(TEST_ROUNDS):
        rknn_model.inference(inputs=[image], data_format=['nhwc'])
    end_time = time.time()

    total_time = end_time - start_time
    avg_time = total_time / TEST_ROUNDS

    print(f"RKNN模型测试完成:")
    print(f"- 总耗时: {total_time:.4f}秒")
    print(f"- 平均单次推理耗时: {avg_time:.4f}秒 ({1 / avg_time:.2f} FPS)")

    return avg_time


def test_torch_inference_speed(torch_model, image):
    """测试PyTorch模型在CPU上的推理速度"""
    # 转换图像格式以适应PyTorch [N,H,W,C] -> [N,C,H,W]
    image_tensor = torch.from_numpy(image).permute(0, 3, 1, 2).float()

    print("开始PyTorch模型预热...")
    with torch.no_grad():
        for _ in range(WARMUP_ROUNDS):
            torch_model(image_tensor)

    print("开始PyTorch模型性能测试...")
    start_time = time.time()
    with torch.no_grad():
        for _ in range(TEST_ROUNDS):
            torch_model(image_tensor)
    end_time = time.time()

    total_time = end_time - start_time
    avg_time = total_time / TEST_ROUNDS

    print(f"PyTorch模型测试完成:")
    print(f"- 总耗时: {total_time:.4f}秒")
    print(f"- 平均单次推理耗时: {avg_time:.4f}秒 ({1 / avg_time:.2f} FPS)")

    return avg_time


def main():
    """主函数：执行两种模型的推理速度测试并比较结果"""
    print(f"测试配置: 预热 {WARMUP_ROUNDS} 轮，测试 {TEST_ROUNDS} 轮")
    print(f"使用随机生成的 {IMAGE_SIZE[0]}x{IMAGE_SIZE[1]} 图像作为输入")

    # 生成随机图像数据
    image = generate_random_image()

    # 获取并测试RKNN模型
    rknn_model = get_rknn_model()
    if rknn_model is None:
        return

    rknn_time = test_rknn_inference_speed(rknn_model, image)

    # 获取并测试PyTorch模型
    torch_model = get_torch_model()
    torch_time = test_torch_inference_speed(torch_model, image)

    # 释放资源
    rknn_model.release()

    # 比较结果
    speedup = torch_time / rknn_time
    print("\n===== 推理速度对比结果 =====")
    print(f"RKNN模型 vs PyTorch CPU模型")
    print(f"加速比: {speedup:.2f}x")

    # 生成简单的性能对比图表
    print("\n===== 性能对比图表 =====")
    bar_width = 40
    rknn_bars = int(bar_width * speedup / (speedup + 1))
    torch_bars = bar_width - rknn_bars
    print(f"RKNN模型 ({speedup:.2f}x): " + "█" * rknn_bars)
    print(f"PyTorch模型 (1.00x): " + "█" * torch_bars)


if __name__ == "__main__":
    main()