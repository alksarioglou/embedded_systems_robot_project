class control {
private:
    float values, error, controlaction;
    int ref;
    float integral;
    float derivative;
    float kd;
    float ki;
    float kp;
    float lasterror;
    PwmOut MotorOne,MotorTwo;
public:
    control (PinName l, PinName r) : MotorOne(l), MotorTwo(r) {ref=0, integral=0, derivative=0, kd=0, ki=0,kp=0.15, lasterror=0;}
    void setspeed(){
        values = (3.2)*out1.read()+(2.1)*out2.read()+1*out3.read()+(-1)*out4.read()+(-2)*out5.read()+(-3)*out6.read();
        error=ref-values;
        integral = integral + error;
        derivative = lasterror - error; //derivative=(lasterror - error)/dt ADD dt
        controlaction = (kp*error)+(ki*integral)+(kd*derivative);
        lasterror=error;
        MotorOne.write(0.6f - controlaction);
        MotorTwo.write(0.6f + controlaction);
        wait(0.001);
    }
    
    void setperiod(){
        MotorOne.period(0.0002f);
        MotorTwo.period(0.0002f);
    }
    
    void controlstop(){
        if ((out1.read()<0.05) && (out2.read()<0.05) && (out3.read()<0.05) && (out4.read()<0.05) && (out5.read()<0.05) && (out6.read()<0.05)){ //values need to be tested
            MotorOne.write(0.5f);
            MotorTwo.write(0.5f);
            wait;
        }
    }
     
};  
