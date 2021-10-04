#include "mbed.h"
#include "QEI.h"
#include "C12832.h"

#define sensorKP    0.08
#define sensorKD    0.15
#define sensorKI    0

#define speedKP     0.02
#define speedKD     0
#define speedKI     0.001


#define RATE 0.1
#define wheelradius 0.04 // in m
#define PI 3.14159265358979323846

DigitalOut Enable(PB_8);
C12832 lcd(D11, D13, D12, D7, D10);
QEI wheel1(PB_13,PB_14,NC,256);
QEI wheel2(PC_9,PB_7,NC,256);
DigitalOut sensor1(PB_4);
DigitalOut sensor2(PB_5);
DigitalOut sensor3(PB_3);
DigitalOut sensor4(PA_10);
DigitalOut sensor5(PA_9);
DigitalOut sensor6(PC_7);

AnalogIn out1(PC_0);
AnalogIn out2(PC_1);
AnalogIn out3(PC_2);
AnalogIn out4(PC_3);
AnalogIn out5(PA_4);
AnalogIn out6(PB_0);


class Bipolar {
private:
    DigitalOut Bi;
public:
    Bipolar(PinName pin): Bi(pin) {}
    void tru(void) {Bi = 1 ;}
};

class On {
private:
    DigitalOut pins;
public:
    On(PinName pin): pins(pin) {}
    void tru(void) {pins = 1 ;}
};

class motors {
private:
    PwmOut MotorOne;
    PwmOut MotorTwo;
public:
    motors(PinName l, PinName r) : MotorOne(l), MotorTwo(r) {}
    void setperiod(){
        MotorOne.period(0.0002f);
        MotorTwo.period(0.0002f);
        }
    void constant (){
        MotorOne.write(0.95f);
        MotorTwo.write(0.95f);
        }
    void left () {
        MotorTwo.write(0.8f);
        MotorOne.write(0.6f);
        }
    void right () {
        MotorTwo.write(0.6f);
        MotorOne.write(0.8f);
        }
    void stop () {
        MotorOne.write(0.5f);
        MotorTwo.write(0.5f);
        }  
};

class encoder1 {
private:
    float w1,current1,newangle1,oldangle1,dth1,s1,w3,v3;
    Ticker tttq;
    int sample1;
public:
    encoder1 () {tttq.attach(callback(this,&encoder1::getnumber),0.001),s1=0,w1=0,w3=0,v3=0;}
    
    
    void getnumber (void) {
        sample1=wheel1.getPulses();
    }
    
    int returnsample1 (void) {
        return sample1;
    }
    
    void showspeed1 (void) {
        current1 = sample1;
        newangle1=((current1/(2*256))*360)*((2*PI)/360);
        oldangle1 = 0;
        dth1=newangle1-oldangle1; //angle in rad
        s1+=dth1*wheelradius;
        w1=dth1/0.1; //angular speed in motor shaft in rad/s
        w3=w1*0.053333; //angular speed in wheel shaft in rad/s
        v3=w3*wheelradius; //linear speed of the wheel
        wheel1.reset();
    }
    
    float distancel (void) {   
        return s1;
    }
    
    float speed1 (void) {
        return w3;   
    }
    
    float angle1 (void) {
        return dth1;   
    }
};

class encoder2 {
private:
    float w2,current2,newangle2,oldangle2,dth2,s2,w4,v4;
    Ticker ttt;
    int sample2;
public:
    encoder2 () {ttt.attach(callback(this,&encoder2::getnumber2),0.001),s2=0,w2=0,w4=0,v4=0;}
    
    void getnumber2 (void) {
        sample2 = -wheel2.getPulses();
    }
    
    int returnsample2 (void) {
        return sample2;
    }
    
    void showspeed2 (void) {
        current2 = sample2 ;
        newangle2=((current2/(2*256))*360)*((2*PI)/360);
        oldangle2 = 0;
        dth2=newangle2-oldangle2; //angle in rad
        s2+=dth2*wheelradius;
        w2=dth2/0.1; //angular speed in motor shaft in rad/s
        w4=w2*0.053333; //angular speed in wheel shaft in rad/s
        v4=w4*wheelradius; //linear speed of the wheel
        wheel2.reset();
    }
    
    float distancer (void) {
           return s2;
    }
    
    float speed2 (void) {
           return w4;
    }
    
    float angle2 (void) {
        return  dth2;
    }
    
};


class Bluetooth {
private:
    Ticker ok;
    char c;
    Serial hm10;
    PwmOut MotorOne,MotorTwo;
public:
    Bluetooth(PinName l, PinName r, PinName T, PinName R) : MotorOne(l), MotorTwo(r), hm10(T,R) {ok.attach(callback(this, &Bluetooth::signal), 0.001),hm10.baud(9600);}
    
    void signal (void) {
        if(hm10.readable()){
            c = hm10.getc();
                if(c == 'A'){
                    turnaround();
                }
        }
    }
    void turnaround (void) {
        
        //led=1;
        MotorOne.write(0.5f);
        MotorTwo.write(0.5f);
        wait(1);
        MotorOne.write(0.8f);
        MotorTwo.write(0.4f);
    }
};

class control : public encoder1,encoder2 {
private:
    float values, error, controlaction,difference;
    int ref;
    encoder1 leftwheel;
    encoder2 rightwheel;
    Serial hm10;
    float integral;
    float derivative;
    float kd;
    float ki;
    float kp;
    Serial pc;
    Ticker ttt,dist;
    enum state {normal,bluetooth};
    state st;
    int sample1,sample2;
    float dth1,dth2;
    float angle,angleone,angletwo,lasterror,angulardiff;
    float error2,controlaction2,kp2,kd2,ki2,lasterror2,integral2,derivative2;
    char d;
    PwmOut MotorOne,MotorTwo;
public:
    control (PinName T, PinName R, PinName l, PinName r, PinName a, PinName b) : hm10(T,R), MotorOne(l), MotorTwo(r),pc(a,b) {ttt.attach(callback(this,&control::bluetoothsignal),0.0001),ref=0, integral=0, derivative=0,sample1=0,sample2=0,dth1=0,dth2=0,kd=sensorKD,ki=sensorKI,kp=sensorKP,kp2=speedKP,kd2=speedKD,ki2=speedKI,lasterror=0,angulardiff=0,integral2=0,derivative2=0,lasterror2=0,st=normal,angle=0,angleone=0,angletwo=0,hm10.baud(9600);}
    
    void bluetoothsignal() {
        if (hm10.readable()) {
                d = hm10.getc();
                if(d == 'A'){
                    turnaround();
                }
                if (d=='B'){
                    stop();
                    }
            
        }
        
    }
    
    
    void setspeed(){
        if ((out1.read()<0.36) && (out2.read()<0.36) && (out3.read()<0.36) && (out4.read()<0.36) && (out5.read()<0.36) && (out6.read()<0.36)){ //values need to be tested
            MotorOne.write(0.5f);
            MotorTwo.write(0.5f);
            MotorOne.write(0.5f);
            MotorTwo.write(0.5f);
            MotorOne.write(0.5f);
            MotorTwo.write(0.5f); 
        } //CONTROLSTOP
        
        
        else {
            values = (3)*out1.read()+(2)*out2.read()+1*out3.read()+(-1)*out4.read()+(-2)*out5.read()+(-3)*out6.read();
            error=ref-values;
            integral = integral + error;
            derivative = error - lasterror;
            lasterror=error;
            if ((error>0.75)|| (error<-0.75)) { //initially 0.75
                controlaction = (kp*error)+(ki*integral)+(kd*derivative);
                MotorOne.write(0.78f - controlaction); //init 0.75
                MotorTwo.write(0.78f + controlaction); //init 0.75
            }
            else if (error>2.7 || error<-2.7) {
                controlaction = (kp*error)+(ki*integral)+(kd*derivative);
                while ((out3.read()<0.78)) {
                    MotorOne.write(0.82f - controlaction);
                    MotorTwo.write(0.82f + controlaction);
                }
            }
            else {
                constantspeed();   
            }
        }
        rightwheel.showspeed2();
        leftwheel.showspeed1();
        lcd.printf("%.3f\n\n", (leftwheel.distancel()+rightwheel.distancer())/2); 
        wait(0.0001);
    }
    
    void setperiod(){
        MotorOne.period(0.0002f);
        MotorTwo.period(0.0002f);
    }
    
    void constantspeed () {
        difference = - leftwheel.speed1() + rightwheel.speed2();
        error2 = - difference;
        integral2 = integral2 + error2;
        derivative2 = error2 - lasterror2;
        lasterror2=error2;
        controlaction2 = (kp2*error2)+(ki2*integral2)+(kd2*derivative2);
        MotorOne.write(0.85f - controlaction2); //init 0.75
        MotorTwo.write(0.85f + controlaction2); //init 0.75
        wait(0.001);
    }
    
    void turnaround () {
        MotorOne.write(0.5f);
        MotorTwo.write(0.5f);
        wait(1);
        MotorOne.write(0.7f);
        MotorTwo.write(0.2f);
        wait(0.5);
        while ((out3.read()<0.45)) {
            MotorOne.write(0.7f);
            MotorTwo.write(0.15f);
        }
            MotorOne.write(0.5f);
            MotorTwo.write(0.5f);
            wait(1);
        while (!((out1.read()<0.36) && (out2.read()<0.36) && (out3.read()<0.36) && (out4.read()<0.36) && (out5.read()<0.36) && (out6.read()<0.36))) {
            values = (3)*out1.read()+(2)*out2.read()+1*out3.read()+(-1)*out4.read()+(-2)*out5.read()+(-3)*out6.read();
            error=ref-values;
            integral = integral + error;
            derivative = error - lasterror;
            lasterror=error;
            if ((error>0.75)|| (error<-0.75)) {
                controlaction = (kp*error)+(ki*integral)+(kd*derivative);
                MotorOne.write(0.75f - controlaction);
                MotorTwo.write(0.75f + controlaction);
            }
            else if (error>2.7 || error<-2.7) {
                controlaction = (kp*error)+(ki*integral)+(kd*derivative);
                while ((out3.read()<0.78)) {
                    MotorOne.write(0.65f - controlaction);
                    MotorTwo.write(0.65f + controlaction);
                }
            }
            else {
                constantspeed();   
            }
        }
        MotorOne.write(0.5f);
        MotorTwo.write(0.5f);
        wait(5);
    }
    
    void stop() {
     MotorOne.write(0.5f);
     MotorTwo.write(0.5f);
     wait(5); 
    }
     
};  


Bipolar B1(PB_10);
Bipolar B2(PB_9);
motors m(PC_6,PC_8);
encoder1 leftwheel;
encoder2 rightwheel;
control pidsensors(PA_11, PA_12, PC_6, PC_8,USBTX, USBRX);
Bluetooth ble(PC_6, PC_8, PA_11, PA_12);
Serial pc(USBTX,USBRX);


int main(void) {
    lcd.cls();
    Enable = 0;
    lcd.locate(20,20);
    B1.tru();
    B2.tru();
    sensor1=1;
    sensor2=1;
    sensor3=1;
    sensor4=1;
    sensor5=1;
    sensor6=1;
    pidsensors.setperiod();
    wait(0.5);
    Enable=1;
    while(1){
        pidsensors.setspeed();     
    }
    
}
