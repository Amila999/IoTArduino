/*
	PIDandLowPass.cpp - Library for PID controller and LowPass Filter
	Created by Amila Ruwan Guruge 2023
*/

#include "Arduino.h"
#include "PIDandLowPass.h"

PIDandLowPass::PIDandLowPass(float y_init)
{
    Tout = y_init;
}

//LowPass Filter
float PIDandLowPass::LowPassFilter(float y){
    float Tf = 5*Ts;
    float a = Ts/(Tf + Ts);
    float yf;
    static float yf_prev = Tout;

    yf = (1-a)*yf_prev + a*y;
    yf_prev = yf;
    return yf;
}

 //Pi controller
float PIDandLowPass::PiController(float y){
    float u_prev = u;
    float e_prev = e;

    e = r - y;
    u = u_prev + Kp*(e - e_prev) + (Kp/Ti)*Ts*e;
    if (u < 0)
      u = 0;
    if (u > 5)
      u = 5;
    return u;
}