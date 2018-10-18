/*
  Control de un acelerómeto por DMP y sensado de flexión con envío por radiofrecuencia
  Autor: Javier Vargas. El Hormiguero.
  Diseño 3D de grossrc bajo licencia Creative Commons - Attribution - Non-Commercial (https://www.thingiverse.com/thing:2269115)
  https://creativecommons.org/licenses/by-nc/3.0/
*/

//CONFIGURACION
#define Muestreo 50 //Frecuencia de muestreo de lectura y envio de datos
#define Nsensores 5 //Sensores de flexion
#define Kf 0.6f //Filtro de lectura de los sensores de flexion
#define Ka 0.8f  //Filtro de lectura del acelerometro
#define AngMax 75 //Angulo de desviación máximo respecto al 0 en cualquiera de los ejes

//PINES
#define PinCE 27
#define PinCSN 28
#define PinInt 2
#define PinR 29
#define PinG 31
#define PinB 32
const int PinFlex[] = {17, 20, 21, 22, 23}; //Sensores SPRK-SEN-10264
//MOSI 11, MISO 12, SCK 13, SDA 18, SCL 19

//Libreria Parpadeo led
#include "BlinkLed.h"
BlinkLed LedR(PinR, 250);
BlinkLed LedG(PinG, 100, 900);
BlinkLed LedB(PinB, 500);

//MPU 6050
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
MPU6050 mpu(0x68); //Direccion del dispositivo 0x68
Quaternion q; // [w, x, y, z] quaternion container
VectorFloat gravity; // [x, y, z] gravity vector
boolean DataReady = 0; //Datos de del DMP listos, aviso por interrupcion)
uint16_t packetSize = 42; //Tamaño del paquete del DMP (por defecto 42 bytes)
float Yoffset = 0, Poffset = 0, Roffset = 0; //Offset de las medidas

//NRF24L01
byte direccion[5] = {'h', 'o', 'r', 'm', 'i'}; //Variable con la dirección del canal por donde se va a transmitir

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(PinCE, PinCSN);
//float Datos[Nsensores + 3];

float Datos[] = {0, 0, 0, 0, 0, 0, 0, 0};

unsigned long m = 0;
float Flexion[] = {0, 0, 0, 0, 0}; //Nivel de flexión de los sensores de los dedos
float ypr[3] = {0, 0, 0}; //Variables Yaw, Pitch y Roll
boolean AnguloOK = 0;

void setup() {
  Serial.begin(115200);

  //Inicio Led
  LedR.begin();
  LedG.begin();
  LedB.begin();
  LedB.NoBlink();
  LedB.On();

  delay(1000);

  //PinMode
  for (int i = 0; i < Nsensores; i++) {
    pinMode(PinFlex[i], INPUT);
  }

  //inicializamos el NRF24L01
  radio.begin();
  radio.openWritingPipe(direccion);

  //Inicio del MPU DMP (Digital Motion Processor)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  mpu.dmpInitialize();
  mpu.setXAccelOffset(-595);
  mpu.setYAccelOffset(-2773);
  mpu.setZAccelOffset(-77);
  mpu.setXGyroOffset(-29);
  mpu.setYGyroOffset(-4);
  mpu.setZGyroOffset(-3);
  mpu.setDMPEnabled(true);
  packetSize = mpu.dmpGetFIFOPacketSize(); //Tamaño del paquete de datos almacenado en la memoria FIFO
  attachInterrupt(digitalPinToInterrupt(PinInt), Interrupcion, RISING);


  LedB.Off();
}

//////////////////////////////////
///////////////LOOP///////////////
//////////////////////////////////

void loop() {


  //Loop principal
  if (millis() / Muestreo != m) {
    m = millis() / Muestreo;
    //Lectura de los sensores de flexion
    LecturaDedos();
    //Envio RF
    EnvioRF();
    //Lectura del acelerometro correcta
    if (AnguloOK) {
      LedR.Off(), LedG.On();
    }
    //Lectura incorrecta, no enviamos datos
    else {
      LedR.On(), LedG.Off();
    }
  }

  //Lectura de IMU por interrupcion del DMP
  LecturaIMU();

  //Estado del led
  LedR.Update();
  LedG.Update();
  LedB.Update();

}

//////////////////////////////////
//////////////////////////////////
//////////////////////////////////

void EnvioRF() {

  //Datos a enviar
  Datos[0] = ypr[0];
  Datos[1] = ypr[2]; //Pitch y roll cambiados**
  Datos[2] = ypr[1];
  Datos[3] = Flexion[0];
  Datos[4] = Flexion[1];
  Datos[5] = Flexion[2];
  Datos[6] = Flexion[3];
  Datos[7] = Flexion[4];

  //Envio por RF
  radio.write(Datos, sizeof(Datos));
}

void LecturaDedos() {
  for (int i = 0; i < Nsensores; i++) {
    //Lectura analogica y filtrado de datos
    Flexion[i] = Kf * analogRead(PinFlex[i]) + (1 - Kf) * Flexion[i];
//    Serial.print(Flexion[i]);
//    Serial.print(" / ");
  }
//  Serial.println();
}

void LecturaIMU() {
  static uint16_t fifoCount = 0; //Bytes en memoria FIFO
  static uint8_t mpuIntStatus; //Estado de la interrupcion
  static uint8_t fifoBuffer[64]; //Memoria FIFO
  static float yprAnt[3]; //Lecturas anteriores

  //DATOS PREPARADOS
  if (DataReady) {
    DataReady = 0;

    mpuIntStatus = mpu.getIntStatus(); //Estado de la interrupcion
    fifoCount = mpu.getFIFOCount(); //Estado de la memoria FIFO

    //ERROR DE DATOS
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) mpu.resetFIFO(); //Overflow de datos, no realizamos la lectura

    //DATOS CORRECTOS
    else if (mpuIntStatus & 0x02) { //Datos correctos

      //Lectura
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      fifoCount -= packetSize;
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      //Offset
      ypr[0] = ypr[0] * 180 / M_PI + Yoffset;
      ypr[1] = ypr[1] * 180 / M_PI + Poffset;
      ypr[2] = ypr[2] * 180 / M_PI + Roffset;

      //Filtrado
      for (int i = 0; i < 3; i++) ypr[i] = Ka * ypr[i] + (1 - Ka) * yprAnt[i];

      //Evitar gimbal lock
      if (abs(ypr[1]) > AngMax || abs(ypr[2]) > AngMax) {
        AnguloOK = 0;
        //Mantenemos lecturas anteriores
        for (int i = 0; i < 3; i++) ypr[i] = yprAnt[i];
      }
      else AnguloOK = 1;

      //Actualizacion de lectura anterior
      for (int i = 0; i < 3; i++) yprAnt[i] = ypr[i];

      //      Serial.print("ypr\t");
      //      Serial.print(ypr[0]);
      //      Serial.print("\t");
      //      Serial.print(ypr[1]);
      //      Serial.print("\t");
      //      Serial.println(ypr[2]);
    }
  }
}

void Interrupcion() {
  DataReady = 1;
}

