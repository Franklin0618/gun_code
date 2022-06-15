/////////////////////////////////////////////////////头文件
#include<WiFi.h>
#include<Adafruit_GFX.h>
#include<Adafruit_ST7789.h>
#include<SPI.h>
#include<esp_task_wdt.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <soc/soc.h> 
#include <soc/rtc_cntl_reg.h>
/////////////////////////////////////////////////////宏定义
#define COMPANY
//#define PLAY_GROUND

#define FIRST_GUN
//#define SECOND_GUN

//引脚宏
#define NEAR_SWITCH         25  //接近开关
#define READER1             7  //读卡器
#define PUSH                33  //按压开关控制
#define FOG_CTRL            12 //雾化器控制
#define SHAKE_CTRL          33         //震动控制 
#define LED                 2           //板载灯
#define LED_FIRE            27          //枪火灯
//屏幕宏
#define TFT_MOSI 23
#define TFT_SCLK 18
#define TFT_CS    5 // Not connected

#define TFT_DC    4
#define TFT_RST   26  // Connect reset to ensure display initialises
//#define TFT_MISO 19

//dac采集引脚
#define BATTERY 20
#define BULLET  32
#define MAXBULLET 50
//////////////////////////////////////////////////////全局变量
WiFiUDP Udp_send;                           //创建UDP发送对象
WiFiUDP Udp_recv;                           //创建UDP接收对象

Adafruit_ST7789 tft=Adafruit_ST7789(TFT_CS,TFT_DC,TFT_MOSI,TFT_SCLK,TFT_RST);

int battery=100;
int bullet=0;
char battery_ch[4];
char bullet_ch[4];

byte buf_send[1];                              //发送区
byte buf_recv[1];                               //接收区
uint8_t serial_send[8]={0x01,0x08,0xA3,0x20,0x01,0x01,0x00,0x75};   //nfc读卡器扇区读取命令
byte readbuff[22];                              //串口读取缓冲区
char macStr[44];                                //将串口读取到的数据转化位char型存放的数组
char old_macStr[44];                            //状态对比
int old_sta;                                    //按压开关状态对比变量
int send_num;
//网络涉及变量
#ifdef COMPANY
const char *ssid = "wifi名字长才能穿透墙";              //需要连接的wifi的名字
const char *password = "1q2w3e4r";                      //需要连接的wifi的密码
#endif

#ifdef PLAY_GROUND
const char *ssid = "angel";            //需要连接的wifi的名字
const char *password = "1qaz2wsx3edc";          //需要连接的wifi的密码
#endif

IPAddress gateway(192, 168, 31, 1);                     
IPAddress subnet(255,255, 0, 0);
unsigned int localUdpPort = 50002;                      //本地端口号（接收）
uint16_t port = 50003;                                  //目标端口号

//枪的ip设置
#ifdef FIRST_GUN
//第一把枪
char* ip = "192.168.31.200";                                            //目标的ip地址
IPAddress staticIP(192, 168, 31, 151);                                  //设置本机静态ip地址
#endif

#ifdef SECOND_GUN
//第二把枪
char* ip = "192.168.31.201";                                            //目标的ip地址
IPAddress staticIP(192, 168, 31, 152);                                  //设置本机静态ip地址
#endif

void snprintf_buf(){
    snprintf(macStr, sizeof(macStr), "%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X",
           readbuff[0], readbuff[1], readbuff[2], readbuff[3], readbuff[4], readbuff[5],
           readbuff[6], readbuff[7], readbuff[8], readbuff[9], readbuff[10], readbuff[11],
           readbuff[12], readbuff[13], readbuff[14], readbuff[15], readbuff[16], readbuff[17],
           readbuff[18], readbuff[19], readbuff[20], readbuff[21]);
}

//nfc检测
//读取读卡器扇区中的数据，如果是1则是左手，如果是0则是右手
void nfc_detect(void * parameter){
    while (1){
        push_detect();
        Serial.write(serial_send,8);
        while(Serial.available()>0){                
            Serial.read(readbuff,22);
        }
        snprintf_buf();
        if(macStr[41]=='1'&&old_macStr[41]!='1'){        //左手握
            bitWrite(buf_send[0], 1, 1);
            udp_send();
            memcpy(old_macStr,macStr,44);
            memset(buf_send,0,1);
        }
        else if(macStr[41]=='0'&&old_macStr[41]!='0'){  //右手握
            bitWrite(buf_send[0],3,1);
            udp_send();
            memcpy(old_macStr,macStr,44);
            memset(buf_send,0,1);
        }
        if (macStr=="0"){
            if(old_macStr[41]=='1'){        //左手松
                bitWrite(buf_send[0],2,1);
                udp_send();
                memcpy(old_macStr,macStr,44);
                memset(buf_send,0,1);
            }
            else if(old_macStr[41]=='0'){      //右手松
                bitWrite(buf_send[0],4,1);
                udp_send();
                memcpy(old_macStr,macStr,44);
                memset(buf_send,0,1);
            }
        }
    }   
}

//抛弹检测
void throw_bullet_detect(){
    if(digitalRead(NEAR_SWITCH)==LOW){
        bitWrite(buf_send[0], 0, 1);
        udp_send();     //优先进行网络通信
        send_num=1;
//        shake_signal(); //振动信号
//        gun_fire_signal();  //喷雾信号
        //Serial.println("trigger");
        bitWrite(buf_send[0], 0, 0);
    }
}

//前按压开关检测
void push_detect(){
        if(old_sta!=digitalRead(PUSH)){
            if(digitalRead(PUSH)==1){
                bitWrite(buf_send[0], 6, 1);
                udp_send();
                memset(buf_send,0,1);
            }else if(digitalRead(PUSH)==0){
                bitWrite(buf_send[0], 5, 1);
                udp_send();
                memset(buf_send,0,1);
            }
            old_sta=digitalRead(PUSH);
        }
    
}

//震动通知和喷雾一起
void fog_signal(void*para){
      while(1){
        if(send_num==1){
          send_num=0;
          digitalWrite(LED_FIRE,HIGH);
          digitalWrite(FOG_CTRL,HIGH);
          vTaskDelay(500); 
        }else{
          digitalWrite(FOG_CTRL,LOW);
          digitalWrite(LED_FIRE,LOW);
        }
        //Serial.println("fog");
      }
}


//显示屏
void display(){
    tft.fillScreen(ST77XX_BLUE);
    tft.drawLine(0, 160, 172, 160, ST77XX_RED);
    
    tft.setCursor(5,80);
    tft.setTextSize(9);
    if(battery>20){
        tft.setTextColor(ST77XX_GREEN);
    }
    else{
        tft.setTextColor(ST77XX_RED);
    }
    itoa(battery,battery_ch,10);
    tft.printf(battery_ch);

    tft.setCursor(5,240);
    tft.setTextSize(9);
    if(bullet>=5){
        tft.setTextColor(ST77XX_WHITE);
    }
    else{
        tft.setTextColor(ST77XX_RED);
    }
    itoa(bullet,bullet_ch,10);
    tft.printf(bullet_ch);
}

//dac采集
float analogread_float_return(int pin){
    float num=(float)analogRead(pin);
    return num/4095;
}

//显示屏初始化
void display_init(){
    tft.init(172,320);
    tft.setRotation(0);
    tft.fillScreen(ST77XX_WHITE);
    tft.fillCircle(86,160,86,ST77XX_RED);
    tft.setCursor(64,120);
    tft.setTextColor(ST77XX_BLUE);
    tft.setTextSize(10);
    tft.printf("9");
    delay(2000);
    tft.fillScreen(ST77XX_BLUE);
}

//弹夹计数及电量检测
void ammo_and_battery_detect(void*para){
    while(1){
        tft.drawLine(0, 160, 172, 160, ST77XX_RED);
        // if(int(analogread_float_return(BATTERY))!=battery){
        //     battery=(int)analogread_float_return(BATTERY);
        //     display();
        // }
        if(int(analogread_float_return(BULLET))!=bullet){
            bullet=(int)analogread_float_return(BULLET)*MAXBULLET;
            display();
        }
    }
}



//udp通信
void udp_send(){
    Udp_send.beginPacket(ip,port);                                          //准备发送数据
    //Serial.print("send to");Serial.print(ip);Serial.print(":");Serial.println(port); //串口打印发送目标
    Udp_send.write((const uint8_t*)buf_send,(sizeof(buf_send)) );  //将数据写入缓冲区   
    Udp_send.endPacket();                                            //发送数据
}

//网络初始化
void net_init(){
    if (WiFi.config(staticIP, gateway, subnet) == false) {  //传入静态ip地址
        Serial.println("Configuration failed.");
    }
    WiFi.mode(WIFI_STA);                      //设置芯片的wifi模式为站点模式
    WiFi.begin(ssid, password);               //启动wifi模块，并连接网络
    while (!WiFi.isConnected())               //如果未连接wifi将循环打印.
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println("Connected");
    Serial.print("IP Address:");
    Serial.println(WiFi.localIP());           //如果已连接则打印机器本身的ip地址
    Udp_recv.begin(localUdpPort);                  //启用UDP监听以接收数据
    Serial.println("net init success!");
}

//io初始化
void io_init(){
    pinMode(NEAR_SWITCH,INPUT);
    pinMode(PUSH,INPUT);
    pinMode(FOG_CTRL,OUTPUT);
    pinMode(SHAKE_CTRL,OUTPUT);
    pinMode(LED,OUTPUT);
    pinMode(LED_FIRE,OUTPUT);
    
    digitalWrite(LED,LOW);
    digitalWrite(FOG_CTRL,LOW);
    digitalWrite(SHAKE_CTRL,LOW);
    digitalWrite(NEAR_SWITCH,HIGH);
    digitalWrite(LED_FIRE,LOW);
    Serial.println("io init success!");
}

void setup(){
    Serial.begin(9600);
    net_init();
    io_init();
    display_init();
    esp_task_wdt_init(60,0);
    esp_task_wdt_add(NULL);
//    xTaskCreate(
//              nfc_detect,          /*任务函数*/
//              "nfc_detect",        /*带任务名称的字符串*/
//              10000,            /*堆栈大小，单位为字节*/
//              NULL,             /*作为任务输入传递的参数*/
//              1,                /*任务的优先级*/
//              tskNO_AFFINITY);            /*任务句柄*/
        xTaskCreatePinnedToCore(
              ammo_and_battery_detect,          /*任务函数*/
              "ammo_and_battery_detect",        /*带任务名称的字符串*/
              10000,            /*堆栈大小，单位为字节*/
              NULL,             /*作为任务输入传递的参数*/
              1,                /*任务的优先级*/
              NULL,
              1);            /*任务句柄*/
        xTaskCreatePinnedToCore(
        fog_signal,          /*任务函数*/
        "fog_signal",        /*带任务名称的字符串*/
        10000,            /*堆栈大小，单位为字节*/
        NULL,             /*作为任务输入传递的参数*/
        1,                /*任务的优先级*/
        NULL,
        0);            /*任务句柄*/
        digitalWrite(LED,HIGH);
}


void loop(){
    //Serial.println("loop");
    esp_task_wdt_reset(); 
    throw_bullet_detect();
    
}
