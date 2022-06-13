#define TRIGGER         1
#define MECHINE_POS     6
#define MECHINE_NEG     3
#define THROW           4
#define SPRING_PIPE     5
#define LED             2

int cmd=0;

void io_init(){
    pinMode(TRIGGER,INPUT);
    pinMode(SPRING_PIPE,INPUT);
    pinMode(MECHINE_POS,OUTPUT);
    pinMode(MECHINE_NEG,OUTPUT);
    pinMode(THROW,OUTPUT);
    pinMode(LED,OUTPUT);
    digitalWrite(MECHINE_POS,LOW);
    digitalWrite(MECHINE_NEG,LOW);
    digitalWrite(THROW,HIGH);
    digitalWrite(LED,HIGH);
}

void mechine_pos(){
    digitalWrite(MECHINE_POS,HIGH);
    digitalWrite(MECHINE_NEG,LOW);
}

void mechine_neg(){
    digitalWrite(MECHINE_POS,HIGH);
    digitalWrite(MECHINE_NEG,LOW);
}

void mechine_stop(){
    digitalWrite(MECHINE_POS,LOW);
    digitalWrite(MECHINE_NEG,LOW);
}

void setup(){
    io_init();
}

void loop(){
    if(digitalRead(TRIGGER)==LOW){     //判断点击触发
        delay(10);                              //延迟10毫秒，因为可能会出现其他信号干扰扳机，这三行是防抖操作
        if(digitalRead(TRIGGER)==LOW){ 
            mechine_pos();
            cmd=1;
        }
    }
    else{                                       
        if(cmd==0){     //已进行复位                     
            mechine_stop();
        }
        else{       //电机复位
            if(digitalRead(SPRING_PIPE)==LOW){
                mechine_neg();
                delay(20);
                mechine_stop();
                cmd=0;
            }
        }
            
    }  
}
