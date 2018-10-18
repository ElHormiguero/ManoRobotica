/*
  Brazo robótico con tensores movidos por servomotores según la flexión de los sensores y control de la orientación por un acelerómetro
  Autor: Javier Vargas. El Hormiguero.
  Diseño 3D de grossrc bajo licencia Creative Commons - Attribution - Non-Commercial (https://www.thingiverse.com/thing:2269115)
  https://creativecommons.org/licenses/by-nc/3.0/
*/

/*Conexion de servos
  0 - Articulacion dedo gordo PCA9685 --> 0 (EMAX ES08A) --> V: Step Down D24V50F5
  1 - Dedo gordo PCA9685              --> 1 (MG996R) --> V: Step Down D24V50F5
  2 - Dedo índice PCA9685             --> 2 (MG996R) --> V: Step Down D24V50F5
  3 - Dedo medio PCA9685              --> 3 (MG996R) --> V: Step Down D24V50F5
  4 - Dedo anular + meñique PCA9685   --> 4 (MG996R) --> V: Step Down D24V50F5
  5 - Angulo roll PCA9685             --> 5 (MG996R) --> V: Step Down D24V50F5
  6 - Angulo pitch PWM Arduino        --> 5 (POWER HD 1235MG) --> V: Vbat 7.4V - 2 diodos
  7 - Angulo yaw PWM Arduino          --> 7 (POWER HD 1235MG) --> V: Vbat 7.4V - 2 diodos
*/

//PINES
#define CE_PIN 3
#define CSN_PIN 10
#define PinServoPWM_6 5 //Servo pwm Pitch
#define PinServoPWM_7 7 //Servo pwm Yaw
//Servos conectados de la salida 0 a la 8 del driver
//CONFIGURACION
#define Nservos 9
#define Ndatos 8
#define TimeCero 1000 //(ms) Tiempo sin recibir datos para volver a la posicion inicial

//Servo MG996R (Driver)
#define SERVOMIN  125 // Anchura minima de señal PWM 
#define SERVOMAX  580 // Anchura maxima de señal PWM 
#define ANGULOMAX 220 //Angulo máximo 

//Servo EMAX ES08A (Driver)
#define mSERVOMIN  220 // Anchura minima de señal PWM 
#define mSERVOMAX  580 // Anchura maxima de señal PWM 
#define mANGULOMAX 165 //Angulo máximo

//Servo POWER HD 1235MG (PWM)
#define gSERVOMIN  544 // Anchura minima de señal PWM 
#define gSERVOMAX  2390 // Anchura maxima de señal PWM   
//Angulo de 180º

//Sensor de los dedos (0-gordo, 1-índice, 3-medio, 4-anular, 5-meñique)
const int SensFlex[] = {610, 560, 540, 550, 570};
const int SensNoFlex[] = {780, 780, 780, 745, 785};
//Angulo de los dedos (0-art.gordo, 1-gordo, 2-índice, 3-medio, 4-anular+meñique)
const int AngFlex[] = {50, 170, 115, 60, 50};
const int AngNoFlex[] = {110, 60, 220, 180, 170};
//Angulo de los ejes (0-yaw, 1-pitch, 2-roll)
const int AngMax[] = {180, 120, 180};
const int AngMin[] = {0, 70, 0};
const int AngCero[] = {75, 90, 90};
//Offset
const int Offset[] = {0, 0, 0};

//DRIVER SERVOS PCA
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); //Direccion 0x40
//SERVO POR PWM (ALIMENTADO POR BATERIA)
#include <Servo.h>
Servo servo6, servo7;

//NRF24L01
byte direccion[5] = {'h', 'o', 'r', 'm', 'i'}; //Variable con la dirección del canal por donde se va a transmitir
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(CE_PIN, CSN_PIN);
float Datos[Ndatos] = {0, 0, 0, 0, 0, 0, 0, 0};

unsigned long m = 0;
boolean e = 0;

void setup() {
  Serial.begin(115200);

  //Inicio driver servos
  pwm.begin();
  pwm.setPWMFreq(60);  //Frecuencia de 60Hz en servos analógicos

  //Inicio del NRF24L01
  radio.begin();
  radio.openReadingPipe(1, direccion);
  radio.startListening();   //empezamos a escuchar por el canal
  delay(500);


  //Inicio servos pwm
  servo6.attach(PinServoPWM_6, gSERVOMIN, gSERVOMAX);
  servo7.attach(PinServoPWM_7, gSERVOMIN, gSERVOMAX);

}

//////////////////////////////////
///////////////LOOP///////////////
//////////////////////////////////

void loop() {

  //DATOS RECIBIDOS POR RF
  if (radio.available()) {
    m = millis();
    //Datos recibidos por RF
    radio.read(Datos, sizeof(Datos));
    //Rotamos el brazo segun Yaw, Pitch y Roll
    RotarBrazo(Datos[0], Datos[1], Datos[2]);
    //Movemos los dedos sengun los sensores de flexion
    MoverDedos(Datos[3], Datos[4], Datos[5], Datos[6], Datos[7]);
  }

  //Vuelta a la posicion inicial si no recibe datos en TimeCero ms
  if (millis() > m + TimeCero) ServoCero();


}

//////////////////////////////////
//////////////////////////////////
//////////////////////////////////

void RotarBrazo(float yaw, float pitch, float roll) {
  //Yaw
  int AngYaw = constrain(AngCero[0] - yaw - Offset[0], AngMin[0], AngMax[0]);
  MoverServo(7, AngYaw);
  //Pitch
  int AngPitch = constrain(AngCero[1] - pitch - Offset[1], AngMin[1], AngMax[1]);
  MoverServo(6, AngPitch);
  //Roll
  int AngRoll = constrain(AngCero[2] + roll + Offset[2], AngMin[2], AngMax[2]);
  MoverServo(5, AngRoll);
}

void MoverDedos(int d1, int d2, int d3, int d4, int d5) {
  //Articulacion dedo gordo + dedo
  d1 = constrain(d1, SensFlex[0], SensNoFlex[0]);
  MoverServo(0, map(d1, SensFlex[0], SensNoFlex[0], AngFlex[0], AngNoFlex[0]));
  MoverServo(1, map(d1, SensFlex[0], SensNoFlex[0], AngFlex[1], AngNoFlex[1]));
  //Dedo índice
  d2 = constrain(d2, SensFlex[1], SensNoFlex[1]);
  MoverServo(2, map(d2, SensFlex[1], SensNoFlex[1], AngFlex[2], AngNoFlex[2]));
  //Dedo medio
  d3 = constrain(d3, SensFlex[2], SensNoFlex[2]);
  MoverServo(3, map(d3, SensFlex[2], SensNoFlex[2], AngFlex[3], AngNoFlex[3]));
  //Dedo anular + meñique
  d4 = constrain(d4, SensFlex[3], SensNoFlex[3]);
  d5 = constrain(d5, SensFlex[4], SensNoFlex[4]);
  int dMed = (d4 + d5) / 2;
  int SensFlexMed = (SensFlex[3] + SensFlex[4]) / 2;
  int SensNoFlexMed = (SensNoFlex[3] + SensNoFlex[4]) / 2;
  MoverServo(4, map(dMed, SensFlexMed, SensNoFlexMed, AngFlex[4], AngNoFlex[4]));
}

void MoverServo(int S, int A) {
  //EMAX ES08A
  if (S == 0) {
    pwm.setPWM(S, 0, map(A, 0, mANGULOMAX, mSERVOMIN, mSERVOMAX));
  }
  //MG996R
  else if (S <= 5) {
    pwm.setPWM(S, 0, map(A, 0, ANGULOMAX, SERVOMIN, SERVOMAX));
  }
  //POWER HD 1235MG
  else if (S <= 7) {
    if (S == 6) servo6.write(A);
    if (S == 7) servo7.write(A);
  }
}

void ServoCero() {
  MoverServo(0, AngNoFlex[0]);
  MoverServo(1, AngNoFlex[1]);
  MoverServo(2, AngNoFlex[2]);
  MoverServo(3, AngNoFlex[3]);
  MoverServo(4, AngNoFlex[4]);
  MoverServo(5, AngCero[2]);
  MoverServo(6, AngCero[1]);
  MoverServo(7, AngCero[0]);
}


