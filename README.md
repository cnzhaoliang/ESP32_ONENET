# ESP32_ONENET
项目使用Vscode+PlatformIO环境编写
ESP32(运行Arduino框架、FreeRTOS）用于驱动MAX30102、SGP30、DHT11，并将处理后的数据上报至oneNET云平台
驱动MAX30102传感器以(sampleRate / sampleAverage = 25)Hz的采样率采集(samples = 128)个红外光强数据作为一个采样周期，使用arduinoFFT库实现的FFT算法进行频谱分析，提取采样周期信号的主频作为心跳测量值
