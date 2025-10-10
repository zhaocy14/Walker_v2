import numpy as np
import cv2
from rknn.api import RKNN
import os
import torch
import torchvision.models as models
import time
from concurrent.futures import ThreadPoolExecutor

# 测试配置
TEST_ROUNDS = 10
TARGET_PLATFORM = 'rk3588'
MODEL_NAME = 'resnet50'
WORKERS = 4  # 线程池大小
USE_SIMULATOR = False  # 设置为False以在实际设备上运行


def export_pytorch_model():
    net = getattr(models, MODEL_NAME)(weights=getattr(models, f'ResNet50_Weights').DEFAULT)
    net.eval()
    trace_model = torch.jit.trace(net, torch.Tensor(1, 3, 224, 224))  # 改为单样本
    trace_model.save(f'./{MODEL_NAME}.pt')


def load_image():
    img = cv2.imread('./space_shuttle_224.jpg')
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    return np.expand_dims(img, 0)  # 形状: [1, 224, 224, 3]


if __name__ == '__main__':
    model = f'./{MODEL_NAME}.pt'
    if not os.path.exists(model):
        export_pytorch_model()

    # 创建RKNN对象
    rknn = RKNN(verbose=False)

    # 配置模型（移除batch_size）
    print(f'--> Config model ({MODEL_NAME}) for {TARGET_PLATFORM}')
    rknn.config(
        mean_values=[123.675, 116.28, 103.53],
        std_values=[58.395, 58.395, 58.395],
        target_platform=TARGET_PLATFORM
    )
    print('done')
    # ret = rknn.load_rknn(f'./{MODEL_NAME}_{TARGET_PLATFORM}.rknn')
    # 加载模型（使用单样本输入）
    print('--> Loading model')
    ret = rknn.load_pytorch(model=model, input_size_list=[[1, 3, 224, 224]])
    if ret != 0:
        print('Load model failed!')
        exit(ret)
    print('done')

    # 构建模型
    print('--> Building model')
    ret = rknn.build(do_quantization=True, dataset='./dataset.txt')
    if ret != 0:
        print('Build model failed!')
        exit(ret)
    print('done')

    # 导出rknn模型
    print('--> Export rknn model')
    ret = rknn.export_rknn(f'./{MODEL_NAME}_{TARGET_PLATFORM}.rknn')
    if ret != 0:
        print('Export rknn model failed!')
        exit(ret)
    print('done')

    # 根据环境选择初始化方式
    print('--> Init runtime environment')
    if USE_SIMULATOR:
        print("Running in simulator mode (single-core)")
        ret = rknn.init_runtime()  # 模拟器模式
    else:
        print("Running on physical device (multi-core)")
        # 设置环境变量配置NPU核心
        os.environ['RKNN_NPU_CORE_MASK'] = '0xF'  # 启用所有4个NPU核心
        ret = rknn.init_runtime()  # 不直接传递core_mask
    if ret != 0:
        print('Init runtime environment failed!')
        exit(ret)
    print('done')

    # 加载单张图像
    image = load_image()

    # 创建线程池进行异步推理
    executor = ThreadPoolExecutor(max_workers=WORKERS)

    # 预热运行
    print('--> Warmup NPU model')
    rknn.inference(inputs=[image], data_format=['nhwc'])
    print('done')

    # NPU多次推理测试（单样本）
    print(f'--> Running NPU model ({MODEL_NAME}) for {TEST_ROUNDS} rounds')
    npu_times = []

    for i in range(TEST_ROUNDS):
        start_time = time.time()
        outputs = rknn.inference(inputs=[image], data_format=['nhwc'])
        end_time = time.time()
        npu_times.append(end_time - start_time)
        print(f'NPU round {i + 1}/{TEST_ROUNDS}: {npu_times[-1]:.6f} seconds')

    npu_avg_time = sum(npu_times) / TEST_ROUNDS
    print(f'NPU average inference time: {npu_avg_time:.6f} seconds')

    rknn.release()

    # 准备PyTorch模型
    print('--> Loading PyTorch model')
    net = getattr(models, MODEL_NAME)(weights=getattr(models, f'ResNet50_Weights').DEFAULT)
    net.eval()

    # 预处理图像
    img_tensor = torch.Tensor(image).permute(0, 3, 1, 2)  # 转换为NCHW格式
    mean = torch.Tensor([123.675, 116.28, 103.53]).view(1, 3, 1, 1)
    std = torch.Tensor([58.395, 58.395, 58.395]).view(1, 3, 1, 1)
    img_tensor = (img_tensor - mean) / std

    # 预热运行
    print('--> Warmup PyTorch model')
    with torch.no_grad():
        net(img_tensor)
    print('done')

    # PyTorch多次推理测试
    print(f'--> Running PyTorch model ({MODEL_NAME}) for {TEST_ROUNDS} rounds')
    cpu_times = []

    for i in range(TEST_ROUNDS):
        start_time = time.time()
        with torch.no_grad():
            outputs_cpu = net(img_tensor)
        end_time = time.time()
        cpu_times.append(end_time - start_time)
        print(f'CPU round {i + 1}/{TEST_ROUNDS}: {cpu_times[-1]:.6f} seconds')

    cpu_avg_time = sum(cpu_times) / TEST_ROUNDS
    print(f'CPU average inference time: {cpu_avg_time:.6f} seconds')

    # 性能对比
    speedup = cpu_avg_time / npu_avg_time
    print("\n===== Performance Comparison =====")
    print(f"{MODEL_NAME} on NPU ({TARGET_PLATFORM}) vs CPU")
    print(f"Speedup: {speedup:.2f}x")
    print(f"NPU Throughput: {1 / npu_avg_time:.2f} FPS")
    print(f"CPU Throughput: {1 / cpu_avg_time:.2f} FPS")
