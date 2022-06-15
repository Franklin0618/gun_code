#define TRIGGER         3
#define MECHINE_POS     0
#define MECHINE_NEG     1
#define THROW           18
#define SPRING_PIPE     7
#define LED             2

int cmd=0;

void io_init(){
    pinMode(TRIGGER,INPUT);
    pinMode(SPRING_PIPE,INPUT);
    pinMode(MECHINE_POS,OUTPUT);
    pinMode(MECHINE_NEG,OUTPUT);
    pinMode(THROW,OUTPUT);
    pinMode(LED,OUTPUT);
    digitalWrite(MECHINE_POS,HIGH);
    digitalWrite(MECHINE_NEG,HIGH);
    digitalWrite(THROW,HIGH);
    digitalWrite(LED,HIGH);
}

void mechine_pos(){
  
    digitalWrite(MECHINE_POS,HIGH);
    digitalWrite(MECHINE_NEG,LOW);
    Serial.println("pos");
}

void mechine_neg(){
    digitalWrite(MECHINE_POS,HIGH);
    digitalWrite(MECHINE_NEG,LOW);
    Serial.println("neg");
}

void mechine_stop(){
    digitalWrite(MECHINE_POS,LOW);
    digitalWrite(MECHINE_NEG,LOW);
    Serial.println("stop");
}

void setup(){
    Serial.begin(9600);
    io_init();
}

void loop(){
    if(digitalRead(TRIGGER)==LOW){     //判断点击触发
        delay(10);                              //延迟10毫秒，因为可能会出现其他信号干扰扳机，这三行是防抖操作
        if(digitalRead(TRIGGER)==LOW){
            Serial.println("trigger"); 
            mechine_pos();
            cmd=1;
        }
    }
    else{                                       
        if(cmd==0){     //已进行复位                     
            mechine_stop();
        }
        else{       //电机复位
            while(1){
              if(digitalRead(SPRING_PIPE)==LOW){
                  mechine_neg();
                  delay(35);
                  mechine_stop();
                  cmd=0;
                  break;
              }
            }
        }
            
    }  
}
