# ESP32_ONENET
# 开发环境
  Vscode+PlatformIO
# 功能简介
  ESP32(运行Arduino框架、FreeRTOS)用于驱动MAX30102、SGP30、DHT11，在OLED上显示MAX30102采样波形，将处理后的数据上报至oneNET云平台
# 心率算法
  驱动MAX30102传感器以(sampleRate / sampleAverage = 25)Hz的采样率采集(samples = 128)个IR数据作为一个采样周期，使用arduinoFFT库实现的FFT算法进行频谱分析，提取采样周期信号的主频作为心跳测量值
