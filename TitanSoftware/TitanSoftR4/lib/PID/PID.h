#ifndef PID_H
#define PID_H
class PID
{
public:
    double Kp = 0, Ki = 0, Kd = 0;
    double setpoint = 0;
    double input;

    PID() {  }; // Default initializer
    PID(double p, double i, double d) { Kp = p, Ki = i; Kd = d; }; // Gains initializer
    PID(double p, double i, double d, double s) { Kp = p, Ki = i; Kd = d; setpoint = s; }; // Full initializer

    float getKp() { return Kp; }
    float getKi() { return Ki; }
    float getKd() { return Kd; }
    void setKp(float newKp) { Kp = newKp; }
    void setKi(float newKi) { Ki = newKi; }
    void setKd(float newKd) { Kd = newKd; }

    double update(double input, double dt); // Updates PID maths and returns new output
    
    float getSetpoint() { return setpoint; }
    void setSetpoint(float newSetpoint) { setpoint = newSetpoint; } 
private:
    float integral = 0;
    double prevError = 0;
}; 

#endif