/*
	PIDandLowPass.h - Library for PID controller and LowPass Filter
	Created by Amila Ruwan Guruge 2022
*/

#ifndef PIDandLowPass_h
#define PIDandLowPass_h 

#include "Arduino.h"

class PIDandLowPass{
	public:
		//Simulation
		float Ts;

		//Controller
		float r;
		float Kp;
		float Ti;

		//Constructor
		PIDandLowPass(float y_init);

		//Functions
		float PiController(float y);
		float LowPassFilter(float y);

	private:
		float u;
		float e;
		float Tout;
};

#endif