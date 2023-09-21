#include <PIDandLowPass.h>

//Simulation
float Ts = 0.1;

//Controller
float u = 0;

float y_init = 21.5;
float y = y_init;

//Simulator
//Air Heater Model
float Kh = 3.5;
float theta_t = 22;
float theta_d = 2;
float Tenv = 21.5;
float Tout = Tenv;
float Tout_prev = Tenv;

//Constructor
PIDandLowPass pidAndLowPass(y_init);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  pidAndLowPass.r = 30; //Setpoint
  pidAndLowPass.Kp = 0.8;
  pidAndLowPass.Ti = 21.5;
  pidAndLowPass.Ts = Ts;
}

void loop() {
  // put your main code here, to run repeatedly:
  u = pidAndLowPass.PiController(y);
  y = AirHeaterModel(u);
  u = pidAndLowPass.LowPassFilter(u);
  Serial.println(y);
  delay(100*Ts);
}

float AirHeaterModel(float u){
  Tout_prev = Tout;
  Tout = Tout_prev + (Ts/theta_t) * (-Tout_prev + Kh*u + Tenv);
  return Tout;
}