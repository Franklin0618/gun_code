/////////////////////////////////////////////////////头文件
#include<WiFi.h>
#include<Adafruit_GFX.h>
#include<Adafruit_ST7789.h>
#include<SPI.h>
/////////////////////////////////////////////////////宏定义
#define COMPANY
//#define PLAY_GROUND

#define FIRST_GUN
//#define SECOND_GUN

//引脚宏
#define NEAR_SWITCH         25  //接近开关
#define READER1             7  //读卡器
#define PUSH                27  //按压开关控制
#define FOG_CTRL            34 //雾化器控制
#define SHAKE_CTRL          33         //震动控制 
#define LED                 2           //板载灯
//屏幕宏
#define TFT_MOSI 23
#define TFT_SCLK 18
#define TFT_CS    5 // Not connected

#define TFT_DC    4
#define TFT_RST   2  // Connect reset to ensure display initialises
//#define TFT_MISO 19

//dac采集引脚
#define BATTERY 20
#define BULLET  15

//////////////////////////////////////////////////////全局变量
WiFiUDP Udp_send;                           //创建UDP发送对象
WiFiUDP Udp_recv;                           //创建UDP接收对象

Adafruit_ST7789 tft=Adafruit_ST7789(TFT_CS,TFT_DC,TFT_MOSI,TFT_SCLK,TFT_RST);

int battery=0;
int bullet=0;
char battery_ch[4];
char bullet_ch[4];

byte buf_send[1];                              //发送区
byte buf_recv[1];                               //接收区
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


//nfc左右手检测
void nfc_detect(){

}

//抛弹检测
void throw_bullet_detect(){
    if(digitalRead(NEAR_SWITCH)==LOW){
        bitWrite(buf_send[0], 0, 1);
        udp_send();     //优先进行网络通信
        shake_signal(); //振动信号
        gun_fire_signal();  //喷雾信号
        bitWrite(buf_send[0], 0, 0);
    }
}

//震动通知
void shake_signal(){
    digitalWrite(SHAKE_CTRL,HIGH);
    delay(5);
    digitalWrite(SHAKE_CTRL,LOW);
}

//喷雾和枪口火焰通知
void gun_fire_signal(){
    digitalWrite(FOG_CTRL,HIGH);
    delay(5);
    digitalWrite(FOG_CTRL,LOW);
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
    return num/4095*100;
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
void ammo_and_battery_detect(){
    while(1){
        tft.drawLine(0, 160, 172, 160, ST77XX_RED);
        if(int(analogread_float_return(BATTERY))!=battery){
            battery=(int)analogread_float_return(BATTERY);
            display();
        }
        if(int(analogread_float_return(BULLET))!=bullet){
            bullet=(int)analogread_float_return(BULLET);
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

    digitalWrite(LED,HIGH);
    digitalWrite(FOG_CTRL,LOW);
    digitalWrite(SHAKE_CTRL,LOW);

    Serial.println("io init success!");
}

void setup(){
    Serial.begin(9600);
    net_init();
    io_init();
    display_init();
}

void loop(){
    throw_bullet_detect();

}
