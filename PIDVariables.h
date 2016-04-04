#ifndef PIDVariables_h
#define PIDVariables_h
//PID Control
double PIDRgt, PIDRgtPwr, PIDLft;//monitored value, controlled value, setpoint
double Kp = 11.9, Ki = 100, Kd = 0.00001; //PID parameters
unsigned accSpd = 0;//used for acceleration of the robot to allow PID to operate properly
PID mtrPID(&PIDRgt, &PIDRgtPwr, &PIDLft, Kp, Ki, Kd, DIRECT);//PID control to allow robot to drive straight
#endif
