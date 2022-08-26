//  多线程基于FreeRTOS，可以多个任务并行处理；
//  ESP32具有两个32位Tensilica Xtensa LX6微处理器；
//  Arduino默认使用core1，core0并没有使用
//  多线程可以指定核运行；
#include <Arduino.h>
#include <Wire.h>           // 硬件IIC驱动库
#include <WiFi.h>           // Wifi支持库
#include "MAX30105.h"       // MAX30102支持库
#include "arduinoFFT.h"     // FFT支持库
#include "Adafruit_SGP30.h" // SGP30支持库
#include "SimpleDHT.h"      // DHT11支持库
#include "PubSubClient.h"   // MQTT支持库
#include "ArduinoJson.h"    // JSON操作库

/*********************************网络配置*********************************/
const char *ssid = "HONOR";                     // wifi名
const char *password = "12345687";              // wifi密码
const char *mqtt_server = "mqtts.heclouds.com"; // onenet 的 IP地址 mqtts.heclouds.com 183.230.40.96
const int port = 1883;                          // 端口号
// 产品ID
#define mqtt_pubid "539400"
// 设备名称
#define mqtt_devid "ESP32"
// 鉴权信息
#define mqtt_password "version=2018-10-31&res=products%2F539400%2Fdevices%2FESP32&et=4817148113&method=md5&sign=F4hWzomPX%2FJYsl3t1B32EA%3D%3D"
WiFiClient espClient;           // 创建一个WIFI连接客户端
PubSubClient client(espClient); // 创建一个PubSub客户端, 传入创建的WIFI客户端
// 设备上传数据的post主题
#define ONENET_TOPIC_PROP_POST "$sys/" mqtt_pubid "/" mqtt_devid "/dp/post/json" //"$sys/" mqtt_pubid "/" mqtt_devid "/thing/property/post"
// post上传数据使用的模板
#define ONENET_POST_BODY_FORMAT "{\"id\":%d,\"dp\":%s}"
// data填充模板
#define DATA_FORMAT "{\"heartrate\":[{\"v\":%d}],\"tvoc\":[{\"v\":%d}],\"eco2\":[{\"v\":%d}],\"h2\":[{\"v\":%d}],\"ethanol\":[{\"v\":%d}],\"temperature\":[{\"v\":%.2f}],\"humidity\":[{\"v\":%.2f}]}"
// 记录已经post了多少条
int postMsgId = 0;
/*********************************网络配置*********************************/

/*********************************FreeRTOS配置*********************************/
#define USE_MULTCORE     // 使用多核模式
xSemaphoreHandle xMutex; // Wire()对象不是线程安全的，创建二值信号量用于协调MAX30102和SGP30
/*********************************FreeRTOS配置*********************************/

/*********************************FFT配置*********************************/
arduinoFFT FFT = arduinoFFT();
const uint16_t samples = 128; // This value MUST ALWAYS be a power of 2
const double samplingFrequency = 25;
double vReal[samples];
double vImag[samples];
/*********************************FFT配置*********************************/

/*********************************MAX30102配置*********************************/
MAX30105 particleSensor;
const byte RATE_SIZE = 4; // Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE];    // Array of heart rates
byte rateSpot;
long lastBeat;        // Time at which the last beat occurred
float beatsPerMinute; // raw heart rate value
int heartrate;

byte ledBrightness = 16; // Options: 0=Off to 255=50mA
byte sampleAverage = 2;  // Options: 1, 2, 4, 8, 16, 32
byte ledMode = 2;        // Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR
int sampleRate = 50;     // Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
int pulseWidth = 411;    // Options: 69, 118, 215, 411
int adcRange = 4096;     // Options: 2048, 4096, 8192, 16384;
/*********************************MAX30102配置*********************************/

/*********************************SGP30配置*********************************/
Adafruit_SGP30 sgp;
int tvoc;
int eco2;
int h2;
int ethanol;
/*********************************SGP30配置*********************************/

/*********************************DHT11配置*********************************/
#define pinDHT11 17
SimpleDHT11 dht11(pinDHT11);
float temp;
float humi;
/*********************************DHT11配置*********************************/

void max30102_setup(void);
void max30102_progress(void);
void sgp30_setup(void);
void sgp30_progress(void);
void dht11_progress(void);
void sensors_update(void);

//连接WIFI相关函数
void setupWifi()
{
  Serial.print("connecting WIFI");
  WiFi.begin(ssid, password);
  while (!WiFi.isConnected())
  {
    Serial.print(".");
    delay(500);
  }
  Serial.println("\nWifi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

//重连函数, 如果客户端断线,可以通过此函数重连
void clientReconnect()
{
  while (!client.connected()) //再重连客户端
  {
    Serial.println("reconnect MQTT...");
    if (client.connect(mqtt_devid, mqtt_pubid, mqtt_password))
    {
      Serial.println("connected");
    }
    else
    {
      Serial.print("failed: ");
      Serial.println(client.state());
      Serial.println("try again in 1 sec");
      vTaskDelay(1000);
    }
  }
}

void xTaskOne(void *xTask1)
{
  while (1)
  {
    sensors_update();
    vTaskDelay(1000);
  }
}

void xTaskTwo(void *xTask2)
{
  while (1)
  {
    max30102_progress();
    vTaskDelay(10);
  }
}

void xTaskThree(void *xTask3)
{
  while (1)
  {
    sgp30_progress();
    vTaskDelay(500);
  }
}

void xTaskFour(void *xTask4)
{
  while (1)
  {
    dht11_progress();
    vTaskDelay(500);
  }
}

void xTaskFive(void *xTask4)
{
  while (1)
  {
    if (!WiFi.isConnected()) //先看WIFI是否还在连接
    {
      setupWifi();
    }
    if (!client.connected()) //如果客户端没连接ONENET, 重新连接
    {
      clientReconnect();
    }
    client.loop(); //客户端循环检测
    vTaskDelay(1000);
  }
}

void setup()
{
  // 初始化串口
  Serial.begin(115200);
  delay(1000);
  // Wire()对象不是线程安全的，创建二值信号量用于协调MAX30102和SGP30
  xMutex = xSemaphoreCreateMutex();
  max30102_setup();
  sgp30_setup();
  delay(1000);
  setupWifi();                         //调用函数连接WIFI
  client.setServer(mqtt_server, port); //设置客户端连接的服务器,连接Onenet服务器, 使用1883端口
  delay(1000);
  Serial.println("setServer Init!");
  client.connect(mqtt_devid, mqtt_pubid, mqtt_password); //客户端连接到指定的产品的指定设备.同时输入鉴权信息
  delay(1000);
  Serial.println("connect Init!");
  if (client.connected())
  {
    Serial.println("OneNet is connected!"); //判断是不是连好了.
  }
  delay(1000);
#ifndef USE_MULTCORE
  xTaskCreate(
      xTaskOne,    /* Task function. */
      "mqtt_post", /* String with name of task. */
      4096,        /* Stack size in bytes. */
      NULL,        /* Parameter passed as input of the task */
      4,           /* Priority of the task.(configMAX_PRIORITIES - 1 being the highest, and 0 being the lowest.) */
      NULL);       /* Task handle. */

  xTaskCreate(
      xTaskTwo,   /* Task function. */
      "max30102", /* String with name of task. */
      4096,       /* Stack size in bytes. */
      NULL,       /* Parameter passed as input of the task */
      2,          /* Priority of the task.(configMAX_PRIORITIES - 1 being the highest, and 0 being the lowest.) */
      NULL);      /* Task handle. */

  xTaskCreate(
      xTaskThree, /* Task function. */
      "sgp30",    /* String with name of task. */
      4096,       /* Stack size in bytes. */
      NULL,       /* Parameter passed as input of the task */
      2,          /* Priority of the task.(configMAX_PRIORITIES - 1 being the highest, and 0 being the lowest.) */
      NULL);      /* Task handle. */

  xTaskCreate(
      xTaskFour, /* Task function. */
      "dht11",   /* String with name of task. */
      4096,      /* Stack size in bytes. */
      NULL,      /* Parameter passed as input of the task */
      2,         /* Priority of the task.(configMAX_PRIORITIES - 1 being the highest, and 0 being the lowest.) */
      NULL);     /* Task handle. */
  xTaskCreate(
      xTaskFive, /* Task function. */
      "check",   /* String with name of task. */
      4096,      /* Stack size in bytes. */
      NULL,      /* Parameter passed as input of the task */
      1,         /* Priority of the task.(configMAX_PRIORITIES - 1 being the highest, and 0 being the lowest.) */
      NULL);     /* Task handle. */
#else
  // 最后一个参数至关重要，决定这个任务创建在哪个核上.PRO_CPU 为 0, APP_CPU 为 1,或者 tskNO_AFFINITY 允许任务在两者上运行.
  xTaskCreatePinnedToCore(xTaskOne, "mqtt_post", 4096, NULL, 4, NULL, 0);
  xTaskCreatePinnedToCore(xTaskTwo, "max30102", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(xTaskThree, "sgp30", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(xTaskFour, "dht11", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(xTaskFive, "check", 4096, NULL, 1, NULL, 0);
#endif
}

void loop()
{
}

// Initialize MAX30102
void max30102_setup(void)
{
  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) // Use default I2C port, 100kHz speed
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
  delay(1000);
  for (int i = 1; (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)); i++)
  {
    Serial.println(String("Retrying: the ") + String(i) + String(" times for ") + String("MAX30102"));
    delay(1000);
  }
  Serial.println("MAX30105 is activating!");
  // MAX30102初始化配置
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); // Configure sensor with these settings
}

// MAX30102工作循环
void max30102_progress(void)
{
  //心率数据产生
  xSemaphoreTake(xMutex, portMAX_DELAY);
  long irValue = particleSensor.getIR();
  xSemaphoreGive(xMutex);
  if (irValue >= 1990 * ledBrightness)
  {
    Serial.println("Wait for the data to stabilize...");
    vTaskDelay(2000);
    Serial.println("Sampling...");
    xSemaphoreTake(xMutex, portMAX_DELAY);
    for (uint16_t i = 0; i < samples; i++)
    {
      vReal[i] = particleSensor.getIR();
      vImag[i] = 0;
    }
    xSemaphoreGive(xMutex);
    double min_value = vReal[0];
    double max_value = 1.0;
    for (uint16_t i = 0; i < samples; i++)
    {
      if (vReal[i] <= min_value)
      {
        min_value = vReal[i];
      }
    }
    for (uint16_t i = 0; i < samples; i++)
    {
      vReal[i] -= min_value;
      if (vReal[i] >= max_value)
      {
        max_value = vReal[i];
      }
    }
    for (uint16_t i = 0; i < samples; i++)
    {
      vReal[i] = vReal[i] * (double(65535) / max_value);
    }
    vTaskDelay(1);
    Serial.println("Sampling ended, calculating...");
    FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    vTaskDelay(1);
    FFT.Compute(vReal, vImag, samples, FFT_FORWARD);
    vTaskDelay(1);
    FFT.ComplexToMagnitude(vReal, vImag, samples);
    vTaskDelay(1);
    Serial.println("calculation successed.");
    double main_frequency = FFT.MajorPeak(vReal, samples, samplingFrequency);
    heartrate = int(60 * main_frequency);
  }
}

// Initialize SGP30
void sgp30_setup(void)
{
  if (!sgp.begin()) // Use default I2C port, 100kHz speed
    Serial.println("SGP30 was not found. Please check wiring/power. ");
  delay(1000);
  for (int i = 1; (!sgp.begin()); i++)
  {
    Serial.println(String("Retrying: the ") + String(i) + String(" times for ") + String("SGP30"));
    delay(1000);
  }
  Serial.println("SGP30 is activating!");
}

// SGP30工作循环
void sgp30_progress(void)
{
  xSemaphoreTake(xMutex, portMAX_DELAY);
  if (sgp.IAQmeasure())
  {
    tvoc = sgp.TVOC;
    eco2 = sgp.eCO2;
  }
  if (sgp.IAQmeasureRaw())
  {
    h2 = sgp.rawH2;
    ethanol = sgp.rawEthanol;
  }
  xSemaphoreGive(xMutex);
}

// DHT11工作循环
void dht11_progress(void)
{
  dht11.read2(&temp, &humi, NULL);
}

// 数据上报
void sensors_update(void)
{
  if (client.connected())
  {
    postMsgId++;
    //先拼接出json字符串
    char param[256];
    char jsonBuf[256];
    sprintf(param, DATA_FORMAT, heartrate, tvoc, eco2, h2, ethanol, temp, humi); //我们把要上传的数据写在param里
    sprintf(jsonBuf, ONENET_POST_BODY_FORMAT, postMsgId, param);
    //再从mqtt客户端中发布post消息
    if (client.publish(ONENET_TOPIC_PROP_POST, jsonBuf))
    {
      Serial.print("Post message to cloud: ");
      Serial.println(jsonBuf);
    }
    else
    {
      Serial.println("Publish message to cloud failed!");
    }
  }
}
