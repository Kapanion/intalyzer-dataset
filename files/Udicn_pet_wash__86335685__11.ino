
/*
可以使用温度传感器检测宠物洗澡机里面的温度进行智能控温
后面改成按一下按钮执行
*/
#include <U8g2lib.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>
#include <ESP32Servo.h>
#include <Ticker.h>
#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
const char* ssid = "REDMI Turbo 4";
const char* password = "@Wcq200528";

IPAddress staticIP(192,168,210,87);   // 静态IP地址
IPAddress gateway(192,168,65,62);      // 网关地址
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(192,168,65,62);
//const char* ssid = "CMCC-6GNW";
//const char* password = "MvAXMYdq";
void handle_interrupt();

bool condition=false ;
/*
wifi
*/
WiFiServer server(80);


#define LED_PIN_shadu 5

/*
超声波
*/
float temp_distance;
#define trigPin  2  // 触发信号引脚（建议使用GPIO2）
#define echoPin  5  // 回声信号引脚（建议使用GPIO5）
/*
OLED
*/
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_ADDRESS 0x3C
/*
温湿度
*/
#define DHTPIN 4
#define DHTTYPE DHT11
/*
空心杯
*/
#define FREQ 2000     // 频率
#define CHANNEL 0     // 通道
#define RESOLUTION 8  // 分辨率
#define LED 12        // LED 引脚
/*
继电器
*/
#define LED_PIN_feng 14
#define LED_Pin_water 15
#define LED_Pin_bubble 16
#define xxx 17
#define LED_PIN_shuazi 27

//舵机
#define SERVO_PIN 13
#define MAX_WIDTH 2500
#define MIN_WIDTH 500


HTTPClient http;
/*
按钮
*/
bool status = false;
#define button_pin_start 25
#define button_pin_stop 26
/*
超声波
*/
float get_distance()
{
  long duration;
  float distance;
  // 发送触发信号（必须至少10微秒高电平）
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // 等待回声信号（最大等待时间设为400cm对应的时间）
  duration = pulseIn(echoPin, HIGH, 29370); // 400cm对应约29370微秒

  if (duration == 0) {
    Serial.println("Out of range");
  } else {
    // 计算距离（声速343m/s ≈ 0.0343cm/μs）
    distance = (duration * 0.0343) / 2;
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
  }

  // 确保至少60ms间隔（HC-SR04要求）
  delay(100);
  return distance;
}
/*
舵机
*/
Servo my_servo;
char duoji = 1;
int angel;
/*
  定时器
*/

bool flag = false;
/*
温湿度
*/
char flag_temperature;
DHT dht(DHTPIN, DHTTYPE);
int a;
 float h;
 float t;
/*
定时器
*/
hw_timer_t *timer = NULL;
hw_timer_t *timer_once=NULL;
/*OLED显示屏*/
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
void timer_interrupt(){
  u8g2.clearBuffer();
  a=1;
}
/*
流程控制
*/
unsigned long previousMillis = 0;  // 用于存储上一次操作的时间戳
unsigned longcurrentMillis ;
int currentStep = 0;               // 当前步骤
bool isRunning = false;            // 是否正在运行流程
bool step_flag=true;
/*
中断函数
*/
int xixi=0;
void toggle();  //定时器中断函数
void handle_interrupt()
{
  if (digitalRead(button_pin_stop) == HIGH) {
    if(flag==1)
    {
        if (!isRunning) {
          isRunning = true;  // 停止计时
        } else {
          isRunning = false;  // 开始计时
          step_flag=true;

          //resetAllDevices();  // 重置所有设备
         }

    }
  }
  if (digitalRead(button_pin_start) == HIGH) {
    flag=1;
    currentStep=1;
    previousMillis = millis();  // 记录当前时间
    isRunning = false;  // 开始计时
}
}
/*
wifi
*/
int time_wifi = 0;
/*
重置所有设备
*/
void resetAllDevices() {
  digitalWrite(LED_Pin_water, LOW);
  digitalWrite(LED_Pin_bubble, LOW);
  digitalWrite(LED_PIN_shuazi, LOW);
  digitalWrite(xxx, LOW);
  digitalWrite(LED_PIN_feng, LOW);
  my_servo.write(0);
}
void setup() 
{ 
  u8g2.setFont(u8g2_font_wqy12_t_gb2312);
  u8g2.enableUTF8Print();
  //u8g2.setFont(u8g2_font_t0_13_tf );  // 选择一个中等大小的字体
  u8g2.begin();

  timer = timerBegin(0,80,true);

  // 配置定时器
  timerAttachInterrupt(timer,timer_interrupt,true);

  // 定时模式，单位us，只触发一次
  timerAlarmWrite(timer,1000000,true); 

  // 启动定时器
  timerAlarmEnable(timer); 
  //串口
  Serial.begin(115200);
  //超声波
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  //wifi
  WiFi.begin(ssid, password);
  // WiFi.config(staticIP, gateway, subnet, primaryDNS);
   Serial.println("掩码地址");
   WiFi.subnetMask();
   Serial.println("默认网关IP地址");
   WiFi.gatewayIP();
   Serial.println("DNS服务器地址");
   WiFi.dnsIP();
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    time_wifi++;
    if (time_wifi >= 5)
    {
      time_wifi = 0;
      Serial.println("time_out");
      break;
    }
    Serial.println("Connecting to WiFi..");
  }  //等待wifi连接->手机热点
  Serial.println(WiFi.localIP());
  Serial.println("Connected to the WiFi network");


  // String url = "https://www.baidu.com/";
  // http.begin(url);

  // int httpCode = http.GET();
  // Serial.println(httpCode);
  server.begin();

  Serial.println("HTTP服务器已启动");
  ////水泵暂时无法使用，用D2指示灯代替
  pinMode(button_pin_start, INPUT_PULLDOWN);
  pinMode(button_pin_stop, INPUT_PULLDOWN);
  pinMode(LED_PIN_shadu,OUTPUT);
  dht.begin();  //开启温湿度检测

  pinMode(LED_Pin_water, OUTPUT);   //用第15IO口控制一个水泵的继电器
  pinMode(LED_Pin_bubble, OUTPUT);  //用第16IO口控制一个泡沫泵的继电器
  pinMode(xxx, OUTPUT);  //用第16IO口控制一个泡沫泵的继电器
  // 配置外部中断引脚
  pinMode(LED_PIN_feng, OUTPUT);
  pinMode(LED_PIN_shuazi, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(button_pin_start), handle_interrupt, RISING);  // 改为RISING触发
  attachInterrupt(digitalPinToInterrupt(button_pin_stop), handle_interrupt, RISING);  // 改为RISING触发
  // 初始化串行通信，用于调试




  // 设置频率
  my_servo.setPeriodHertz(50);
  // 关联 servo 对象与 GPIO 引脚，设置脉宽范围
  my_servo.attach(SERVO_PIN, MIN_WIDTH, MAX_WIDTH);
  //这里的舵机配置和空心杯的PWM的配置必须要在启动时重新配置一遍
  ledcSetup(CHANNEL, FREQ, RESOLUTION);  // 设置通道
  ledcAttachPin(LED, CHANNEL);           // 将通道与对应的引脚连接
  //定时器
  // ESP32PWM::allocateTimer(0);

}

void loop() {

  // 设置并显示小字体
  u8g2.setFont(u8g2_font_wqy12_t_gb2312);
  u8g2.setCursor(0, 20);             // 设置光标位置（第一行）
  u8g2.print("温度: ");
  u8g2.print(t);
  u8g2.print(" °C");
  u8g2.setCursor(0, 50);             // 设置光标位置（第二行）
  u8g2.print("湿度:");             // 打印第二行文本
  u8g2.print(h);

  // 发送缓冲区内容到显示器
  u8g2.sendBuffer();
  if(a==1)
  {
    a=0;
    Serial.println("进入读温度");
    h = dht.readHumidity();
    t = dht.readTemperature();
    temp_distance=get_distance();
  }
  if(temp_distance>=60)
  {
    u8g2.print("  ");
    u8g2.print("空");             // 打印第二行文本
  }
  else
  {
    u8g2.print("  ");
    u8g2.print("非空空");             // 打印第二行文本
  }

  WiFiClient client = server.available();
  if (client)
  {
    String request = client.readStringUntil('\r');
    
    // 添加调试信息，打印收到的所有请求
    Serial.println("收到新的请求：");
    Serial.println(request);

    if (request.indexOf("0x01") >= 0)
    {
      // 处理接收到的0x01指令
      currentStep=1;
      Serial.println("收到的数据是：");
      Serial.println(request);
    }
    if (request.indexOf("GET /environment") != -1) {
      Serial.println("收到环境数据请求");
      // 读取传感器数据
      char json[64];
      if (isnan(h) || isnan(t)) {
         snprintf(json, sizeof(json), 
          "{\"temperature\":%.1f,\"humidity\":%.1f}", t, h);
      } else {
        snprintf(json, sizeof(json), 
          "{\"temperature\":%.1f,\"humidity\":%.1f}", t, h);
      }
      client.println("HTTP/1.1 200 OK");
      client.println("Content-Type: application/json");
      client.println("Connection: close");
      client.println("Access-Control-Allow-Origin: *");  // 解决跨域
      client.println("Server: ESP-HTTP-Server");
      client.println();  // 空行分隔头与内容
      client.println(json);
      
      Serial.print("已发送响应: ");
      Serial.println(json);
    }


    if (request.indexOf("GET /isend") != -1) {
      Serial.println("收到isend状态请求");
      // 构建JSON响应
      char json[64];
      snprintf(json, sizeof(json), "{\"isend\":%s}", condition ? "true" : "false");

      // 发送HTTP响应头
      client.println("HTTP/1.1 200 OK");
      client.println("Content-Type: application/json");  // 改为application/json
      client.println("Connection: close");
      client.println("Access-Control-Allow-Origin: *");
      client.println("Server: ESP-HTTP-Server");        // 添加服务器标识
      client.println();  // 空行分隔头与内容
      
      // 发送响应内容
      client.println(json);
      
      Serial.print("已发送isend响应: ");
      Serial.println(json);
      Serial.print("当前condition值: ");
      Serial.println(condition);
    }
    client.flush();
    client.stop();
  }
  unsigned long currentMillis = millis();
  switch (currentStep) {
    case 1:
      //OK
      {
        if(step_flag)
        {
          Serial.println("舵机");
          my_servo.setPeriodHertz(50);
          my_servo.attach(SERVO_PIN, MIN_WIDTH, MAX_WIDTH);
          my_servo.write(90);
        }
        if(!isRunning)
        {
          //Serial.println(isRunning);
          step_flag=false;
          if (currentMillis - previousMillis >= 2000) {  // 等待2秒
            currentStep = 2;
            step_flag=true;
            my_servo.detach();
            previousMillis = currentMillis;
          }
        }
        else{
          previousMillis = currentMillis;
        }
        break;
      }
    case 2:
      {
        //
        //OK
        if(step_flag)
        {
          Serial.println("开始水泵");
          //digitalWrite(xxx, HIGH);
          digitalWrite(LED_Pin_water, HIGH);//IO口给上升信号水泵开始喷水
        }
       // digitalWrite(LED_BLUE, HIGH);  //暂时用D2指示灯代替
        if(!isRunning)
        {
          step_flag=false;
          if (currentMillis - previousMillis >= 4000) {  // 等待2秒
            currentStep = 3;
            step_flag=true;
            previousMillis = currentMillis;
          }
        }
        else{
          previousMillis = currentMillis;
        Serial.println("暂停水泵");

          digitalWrite(LED_Pin_water, LOW);//IO口给上升信号水泵开始喷水
        }         //等2秒钟停止喷水
        break;
      }
    case 3:
      {
        Serial.println("停止水泵");
        currentStep = 4;
        digitalWrite(LED_Pin_water, LOW);  //停止水泵
        break;
      }
    case 4:
      {
        //OK
        if(step_flag)
        {
          Serial.println("开始泡沫水泵");
          digitalWrite(LED_Pin_bubble, HIGH);  //IO口给上升信号泡沫水泵开始喷水
        }

        if(!isRunning)
        {
          
          step_flag=false;
          if (currentMillis - previousMillis >= 4000) {  // 等待2秒
            currentStep = 5;
            step_flag=true;
            previousMillis = currentMillis;
          }
        }
        else{
          previousMillis = currentMillis;
          Serial.println("停止泡沫水泵");
          digitalWrite(LED_Pin_bubble, LOW);  //停止
        }                       //等两秒停止
        break;
      }
    case 5:
      {
        //OK
        Serial.println("停止泡沫水泵");
        currentStep = 6;
        digitalWrite(LED_Pin_bubble, LOW);  //停止
        break;
      }
    case 6:
      {
        if(step_flag)
        {
          Serial.println("洗澡");
          digitalWrite(LED_PIN_shuazi, HIGH);  //IO口给上升信号泡沫水泵开始喷水
        }
        // ledcSetup(CHANNEL, FREQ, RESOLUTION);  // 设置通道
        // ledcAttachPin(LED, CHANNEL);           // 将通道与对应的引脚连接
        // ledcAttachPin(LED, CHANNEL);
        // ledcWrite(CHANNEL, 255);  // 输出PWM
        // //空心杯开始旋转->给宠物洗澡，空心杯使用PWM控制
        //延时
        //空心杯停止
        //可以加一个判断语句终止洗澡
        //这里可以使用定时器去计时
        if(!isRunning)
        {
          step_flag=false;
          if (currentMillis - previousMillis >= 4000) {  // 等待2秒
            currentStep = 7;
            step_flag=true;
            previousMillis = currentMillis;
          }
        }
        else{
          previousMillis = currentMillis;
          Serial.println("停止洗澡");
        digitalWrite(LED_PIN_shuazi, LOW);  //IO口给上升信号泡沫水泵开始喷水
        }
        break;
      }
      case 7:
      {
        currentStep = 8;
        Serial.println("停止洗澡");
        digitalWrite(LED_PIN_shuazi, LOW);  //IO口给上升信号泡沫水泵开始喷水
      }
      case 8:
      {
        if(step_flag)
        {
          Serial.println("冲水");
          digitalWrite(LED_Pin_water, HIGH);  //开始水泵
        }
        if(!isRunning)
        {
          step_flag=false;
          if (currentMillis - previousMillis >= 4000) {  // 等待2秒
            currentStep = 9;
            step_flag=true;
            previousMillis = currentMillis;
          }
        }
        else{
          Serial.println("停止冲水");
          digitalWrite(LED_Pin_water, LOW);  //停止水泵
          previousMillis = currentMillis;
        }
        break;
      }
      case 9:
      {
        currentStep = 10;
        Serial.println("停止冲水");
        digitalWrite(LED_Pin_water, LOW);  //停止水泵
        break;
      }
    case 10:
      {
        if(step_flag)
        {
        Serial.println("吹风");
        digitalWrite(xxx, HIGH);
        digitalWrite(LED_PIN_feng, HIGH);
        }

        if(!isRunning)
        {
          step_flag=false;
          if (currentMillis - previousMillis >= 4000) {  // 等待2秒
            currentStep = 11;
            step_flag=true;
            digitalWrite(xxx, LOW);
            previousMillis = currentMillis;
          }
        }
        else{
          previousMillis = currentMillis;
          Serial.println("停止吹风");
          digitalWrite(LED_PIN_feng, LOW);  //IO口给上升信号泡沫水泵开始喷水                  //等两秒停止
        }                     //等两秒停止
        break;
        //使用带有风扇的电机吹风
      }
      case 11:
      {
        currentStep = 12;
        Serial.println("停止吹风");
        digitalWrite(xxx, LOW);
        digitalWrite(LED_PIN_feng, LOW);  //IO口给上升信号泡沫水泵开始喷水                  //等两秒停止
        previousMillis = currentMillis;
        break;       
      }

    case 12:
      {
        previousMillis = currentMillis;
        Serial.println("舵机回去");
        currentStep = 13;
        my_servo.setPeriodHertz(50);
        // 关联 servo 对象与 GPIO 引脚，设置脉宽范围
        my_servo.attach(SERVO_PIN, MIN_WIDTH, MAX_WIDTH);
        my_servo.write(0);  //开机舵机转至90度
        condition =true;
        delay(3000);
        break;
      }



      //消毒
      case 13:
      {

        if(step_flag&&temp_distance>=60)
        {
        Serial.println("开始渍水");
        digitalWrite(LED_Pin_water, HIGH);  //开始水泵
        }

        if(!isRunning&&temp_distance>=60)
        {
          step_flag=false;
          if (currentMillis - previousMillis >= 5000) {  // 等待2秒
            currentStep = 14;
            step_flag=true;
            previousMillis = currentMillis;
          }
        }
        else{
          Serial.println("停止渍水");
          digitalWrite(LED_Pin_water, LOW);  //停止水泵
          previousMillis = currentMillis;
        }                     //等两秒停止
        break;
      }
      case 14:
      {
        currentStep = 15;
        Serial.println("停止冲水");
        digitalWrite(LED_Pin_water, LOW);  //停止水泵
        break;
      }
      case 15:
      {
      if(step_flag&&temp_distance>=60)
              {
              Serial.println("开始消毒");

              //digitalWrite(xxx, HIGH);
              digitalWrite(LED_PIN_shadu, LOW);
              // 打开紫色LED灯
              }

              if(!isRunning&&temp_distance>=60)
              {
                step_flag=false;
                if (currentMillis - previousMillis >= 5000) {  // 等待2秒
                  currentStep = 16;
                  step_flag=true;
                  digitalWrite(LED_PIN_shadu, HIGH);
                  previousMillis = currentMillis;
                }
              }
              else{
                previousMillis = currentMillis;
                Serial.println("关闭紫色LED灯");
              // digitalWrite(LED_PIN_feng, LOW);  //IO口给上升信号泡沫水泵开始喷水                  //等两秒停止
              }                     //等两秒停止
              break;

      }
      
      
    default:
      {
        //flag=1;
        //resetAllDevices();
        condition=false;
        break;
      }
  }
}

void toggle()  //定时器中断
{
  flag_temperature = 1;
}

