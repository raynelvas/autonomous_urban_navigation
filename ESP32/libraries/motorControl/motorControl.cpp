#include "motorControl.h"

motorControl::motorControl(double Kp, double Ti, double Td, unsigned long sampleTime){
    setSampleTime(sampleTime);
    setGains(Kp, Ti, Td);
    setCvLimits(255,0);   // Default limits to generate a 8 bit resolution PWM control signal
  
    memset(state, 0, 3u * sizeof(double));
}

motorControl::motorControl(unsigned long sampleTime) :motorControl(0.0,0.0,0.0,sampleTime)
{
}

double motorControl::compute(double setpoint,double input) {
  
      setpoint = scalePv(setpoint);
    input = scalePv(input);

        double error = setpoint - input;
    
    state[2] += Ts*error; // Termino integral
    state[1] = (error-state[0])/Ts; // Termino derivativo
    state[0] = error; // error[k-1]
    
    double output = _kp*(error+(1/_ti)*state[2]+_td*state[1]);
   

        /* Anti-windup */
        if(output > 100) {
            output = 100;
        }
        else if(output < -100) {
            output = -100;
        }
    
        output = scaleCv(output);
        return output;

}
void motorControl::lambdaTunning(double K,double tau,double delay)
{   
     lambda = 3*tau;
     _Kp = tau/(K*(0.5*delay+lambda));
     _Ti = tau;
     _Td = 0.5*delay;
     
     _kp = _Kp*((_Ti+_Td)/_Ti);
     _ti = _Ti + _Td;
     _td = _Ti*_Td/(_Ti+_Td);

}

double motorControl::scaleCv(double cv) 
{
  if (cv >= 0) return (cv) * (_cvMax - _cvMin) / (100.0) + _cvMin; else return (cv) * (-_cvMax + _cvMin) / (-100.0) - _cvMin;
}

double motorControl::scalePv(double pv) 
{
  return pv*(100.0/_pvMax);
}

bool motorControl::reset() {
    /* Clear the state buffer. The size will be always 3 samples */
    memset(state, 0, 3u*sizeof(double));
    return true;
}

void motorControl::setGains(double Kp, double Ti, double Td) {
  
  _kp = Kp;
  _ti = Ti;
  _td = Td;
}

double motorControl::getK(){ return  _kp; }
double motorControl::getTi(){ return  _ti;}
double motorControl::getTd(){ return  _td;}

void motorControl::setSampleTime(unsigned long sampleTime) {
    /* Sample time in ms to seconds */
    Ts = (double)sampleTime/1000.0;
}

void motorControl::setCvLimits(double cvMax, double cvMin) {
  _cvMax = cvMax;
    _cvMin = cvMin;
}

void motorControl::setPvLimits(double pvMax, double pvMin) {
    _pvMax = pvMax;
    _pvMin = pvMin;
}
