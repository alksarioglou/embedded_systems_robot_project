#include "mbed.h"
#include "QEI.h"
#include "C12832.h"

#define wheelradius 0.04 // in m
#define PI 3.14159265358979323846

C12832 lcd(D11, D13, D12, D7, D10); 
QEI wheel1(PB_13,PB_14,NC,256);
QEI wheel2(PC_9,PB_7,NC,256);
Serial pc(USBTX,USBRX);

class Bipolar {
private:
    DigitalOut Bi;
public:
    Bipolar(PinName pin): Bi(pin) {}
    void tru(void) {Bi = 1 ;}
};

class encoder1 {
private:
    float w1,current1,newangle1,oldangle1,dth1,s1,w3,v3;
    Ticker tttq;
    int sample1;
public:
    encoder1 () {tttq.attach(callback(this,&encoder1::getnumber),0.1),s1=0,w1=0,w3=0,v3=0;}
    
    
    void getnumber (void) {
        sample1=wheel1.getPulses();
    }
    
    int returnsample1 (void) {
        return sample1;
    }
    
    void showspeed1 (void) {
        current1 = (float) sample1;
        newangle1=((current1/(2*256))*360)*((2*PI)/360);
        oldangle1 = 0;
        dth1=newangle1-oldangle1; //angle in rad
        s1+=dth1*wheelradius;
        w1=dth1/0.1; //angular speed in motor shaft in rad/s
        w3=w1*0.053333; //angular speed in wheel shaft in rad/s
        v3=w3*wheelradius; //linear speed of the wheel
        wheel1.reset();
        /*lcd.locate (0,3);
        lcd.printf("Left wheel:");
        lcd.locate (0,20);
        lcd.printf("%.3f",s1); */
    }
    
    float distancel (void) {   
        return s1;
    }
    
    float speed1 (void) {
        return v3;   
    }
    
};

class encoder2 {
private:
    float w2,current2,newangle2,oldangle2,dth2,s2,w4,v4,*stwo;
    Ticker ttt;
    int sample2;
public:
    encoder2 () {ttt.attach(callback(this,&encoder2::getnumber2),0.1),s2=0,w2=0,w4=0,v4=0,stwo = &s2;}
    
    void getnumber2 (void) {
        sample2=-wheel2.getPulses();
    }
    
    int returnsample2 (void) {
        return sample2;
    }
    
    void showspeed2 (void) {
        current2 = (float) sample2;
        newangle2=((current2/(2*256))*360)*((2*PI)/360);
        oldangle2 = 0;
        dth2=newangle2-oldangle2; //angle in rad
        s2+=dth2*wheelradius;
        w2=dth2/0.1; //angular speed in motor shaft in rad/s
        w4=w2*0.053333; //angular speed in wheel shaft in rad/s
        v4=w4*wheelradius; //linear speed of the wheel
        wheel2.reset();
        /*lcd.locate (70,3);
        lcd.printf("Right wheel:");
        lcd.locate (70,20);
        lcd.printf("%.3f",s2); */
    }
    
    float distancer (void) {
        return s2;
    }
    
    float speed2 (void) {
        return v4;
    }
};
    
class speedcontrol : public encoder1,encoder2 {
private:
    float kp2,ki2,kd2,error2,integral2,derivative2,controlaction2,lasterror2;
    float difference,ref;
    PwmOut MotorOne,MotorTwo;
public:
    speedcontrol(PinName l, PinName r) : MotorOne(l),MotorTwo(r) {lasterror2=0,ref=0,integral2=0,derivative2=0,controlaction2=0,kp2=1,ki2=0,kd2=0,difference=0;}   
    
    void constantspeed () {
        difference = speed1() - speed2();
        if (difference!=0) {
            error2 = ref - difference;
            integral2 = integral2 + error2;
            derivative2 = lasterror2 - error2;
            controlaction2 = (kp2*error2)+(ki2*integral2)+(kd2*derivative2);
            lasterror2 = error2;
            MotorOne.write(0.75f - controlaction2);
            MotorTwo.write(0.75f + controlaction2);
        }
        else {
            MotorOne.write(0.75f);
            MotorTwo.write(0.75f);     
        }
        wait(0.0001);
    }
        
    
};


int main () {
    encoder1 leftwheel;
    encoder2 rightwheel;
    speedcontrol s(PB_10,PC_8);
    lcd.cls();
    float diff,averagespeed,distance;
    DigitalOut Enable(PC_7);
    Enable = 1;
    Bipolar B1(PB_8);
    Bipolar B2(PB_9);
    PwmOut Motor1(PB_10);
    PwmOut Motor2(PC_8);
    B1.tru();
    B2.tru();
    Motor1.period(0.0002f); //0.4second period
    Motor2.period(0.0002f);
    Motor1.write(0.5f);
    Motor2.write(0.5f);
    while (1) {
        leftwheel.showspeed1();
        rightwheel.showspeed2();
        s.constantspeed();
        diff = leftwheel.speed1() - rightwheel.speed2();
        averagespeed = (leftwheel.speed1() + rightwheel.speed2())/2;
        pc.printf("Difference: ");
        pc.printf("%.5f\n",diff);
        pc.printf("\n");
        wait(1);
        
    }
}
   
    
    
