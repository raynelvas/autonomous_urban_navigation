#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include "Arduino.h"

class motorControl{
    public:
        motorControl(double, double, double, unsigned long);
    motorControl(unsigned long);

    double scaleCv(double);
    double scalePv(double);
    void lambdaTunning(double,double,double);
        double compute(double, double);

        bool reset();

        void setGains(double, double, double);

        void setSampleTime(unsigned long);

        void setCvLimits(double, double);
    
    void setPvLimits(double, double);
    
    double getK();              
      double getTi();           
      double getTd();       
    
  private:
        double _Kp;            // Proportional gain
        double _Ti;            // Integral time gain
        double _Td;            // Derivative gain
        double _kp;             
      double _ti;             
      double _td;     
        double lambda;        

      double state[3];       // State array of length 4 to store e[k-1], e[k-2], u[k-1] and u[k-2]
      double Ts;             // Sample time
        double _cvMax;     // Maximum output value of the controller
        double _cvMin;     // Minimum output value of the controller
    double _pvMax;     // Maximum input value of the controller
        double _pvMin;     // Minimum input value of the controller
}; 

#endif
