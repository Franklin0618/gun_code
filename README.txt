枪的芯片是由两部分构成：

主核：esp32
辅核：esp32c3

主核：主要负责nfc读取，屏幕显示，开火检测，震动，喷雾等效果
辅核：主要负责电机的控制

主核用到的技术：串口，spi，freertos
辅核：开关量控制

主核用到的库：Adafruit_GFX，Adafruit_ST7789
