#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include <Servo.h>
#include <ros.h>
#include <sensor_msgs/Joy.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/UInt16.h>

// Pines del Arduino UNO con PWM: 11, 10, 9, 6, 5, 3
// Hay suficientes para: las dos ruedas, y 4 partes del cuerpo
// Por lo que hay que dejar sin pin PWM a 2 partes del cuerpo (Boca y otra)

// Orden de asignación de las consignas con los motores en ROS:   ceja_izq, ceja_der, cresta, cuello, cuerpo, boca

#define PIN_CEJA_IZQ  2   // Sin PWM
#define PIN_CEJA_DER  5
#define PIN_CRESTA    3
#define PIN_CUELLO    9
#define PIN_CUERPO    6
#define PIN_BOCA      8   // Sin PWM

#define MIN_CEJA_IZQ  544
#define MIN_CEJA_DER  544
#define MIN_CRESTA    544
#define MIN_CUELLO    544
#define MIN_CUERPO    544
#define MIN_BOCA      544

#define MAX_CEJA_IZQ  2400
#define MAX_CEJA_DER  2400
#define MAX_CRESTA    2400
#define MAX_CUELLO    2400
#define MAX_CUERPO    2400
#define MAX_BOCA      2400


// Ruedas: obligatorio que los pines de pulsos tengan PWM para analog_write (los de Dirección no)
#define PIN_RUEDA_IZQ 10 // Con PWM
#define PIN_RUEDA_DER 11 // Con PWM
#define DIR_RUEDA_IZQ 12 // NO SE PUEDE USAR EL 4, es el ZUMBADOR
#define DIR_RUEDA_DER 13

ros::NodeHandle nh;

class Body
{
public:
  /*Funciones*/
  Body()
      : subscriber_("/joy2", &Body::set_period_callback, this)
  {
  }

  /*Consignas*/
  void init(ros::NodeHandle &nh)
  {
    /*Attach*/
    ceja_izq_.attach( PIN_CEJA_IZQ,   MIN_CEJA_IZQ,   MAX_CEJA_IZQ);
    ceja_der_.attach( PIN_CEJA_DER,   MIN_CEJA_DER,   MAX_CEJA_DER);
    cresta_.attach(   PIN_CRESTA,     MIN_CRESTA,     MAX_CRESTA);
    cuello_.attach(   PIN_CUELLO,     MIN_CUELLO,     MAX_CUELLO);
    cuerpo_.attach(   PIN_CUERPO,     MIN_CUERPO,     MAX_CUERPO);
    boca_.attach(     PIN_BOCA,       MIN_BOCA,       MAX_BOCA);
    ceja_izq_consigna_ = 90;
    ceja_der_consigna_ = 90;
    cresta_consigna_ = 90;
    cuello_consigna_ = 90;
    cuerpo_consigna_ = 90;
    boca_consigna_ = 90;

    /* Ruedas */
    rueda_izq_consigna_ = 0;
    rueda_der_consigna_ = 0;

    nh.subscribe(subscriber_);
  }

  
  void detach(Servo motor)
  {
    motor.detach();
  };

  

  void attach(Servo motor, int pin, int minval, int maxval)
  {
    motor.attach(pin, minval, maxval);
  };

  
  void syncro()
  {
    /*Sincronizar consignas con valores*/
    ceja_izq_.write(  ceja_izq_consigna_);
    ceja_der_.write(  ceja_der_consigna_);
    cresta_.write(    cresta_consigna_);
    cuello_.write(    cuello_consigna_);
    cuerpo_.write(    cuerpo_consigna_);
    boca_.write(      boca_consigna_);

    /* NO FUNCIONA EN ARDUINO UNO*/
    /* Aplicación de las consignas de las ruedas*/
    /* Rueda izquierda*/
//    if(rueda_izq_consigna_ >= 0)
//    {
//      digitalWrite(DIR_RUEDA_IZQ, HIGH);
//      analogWrite(PIN_RUEDA_IZQ, rueda_izq_consigna_);
//    }
//    else
//    {
//      rueda_izq_consigna_ = -rueda_izq_consigna_;
//      digitalWrite(DIR_RUEDA_IZQ, LOW);
//      analogWrite(PIN_RUEDA_IZQ, rueda_izq_consigna_);
//    }
//    
//    /* Rueda derecha*/
//    if(rueda_der_consigna_ >= 0)
//    {
//      digitalWrite(DIR_RUEDA_DER, HIGH);
//      analogWrite(PIN_RUEDA_DER, rueda_der_consigna_);
//      
//    }
//    else
//    {
//      rueda_der_consigna_ = -rueda_der_consigna_;
//      digitalWrite(DIR_RUEDA_DER, LOW);
//      analogWrite(PIN_RUEDA_DER, rueda_der_consigna_);
//    }    
  }

  void set_period_callback(const sensor_msgs::Joy &msg)
  {
    ceja_izq_consigna_  = msg.axes[0];
    ceja_der_consigna_  = msg.axes[1];
    cuello_consigna_    = msg.axes[2];
    cresta_consigna_    = msg.axes[3];
    cuerpo_consigna_    = msg.axes[4];
    boca_consigna_      = msg.axes[5];
    
    rueda_izq_consigna_ = msg.axes[6];
    rueda_der_consigna_ = msg.axes[7];
  }

private:
  Servo ceja_izq_;
  Servo ceja_der_;
  Servo cresta_;
  Servo cuello_;
  Servo cuerpo_;
  Servo boca_;

  uint16_t ceja_izq_consigna_;
  uint16_t ceja_der_consigna_;
  uint16_t cresta_consigna_;
  uint16_t cuello_consigna_;
  uint16_t cuerpo_consigna_;
  uint16_t boca_consigna_;
  
  int16_t rueda_izq_consigna_;
  int16_t rueda_der_consigna_;

  ros::Subscriber<sensor_msgs::Joy, Body> subscriber_;
};



Body body;

void setup()
{
  // Pines de control de las ruedas como salidas
  pinMode(PIN_RUEDA_IZQ, OUTPUT);
  pinMode(PIN_RUEDA_DER, OUTPUT);
  pinMode(DIR_RUEDA_IZQ, OUTPUT);
  pinMode(DIR_RUEDA_DER, OUTPUT);
  
  nh.initNode();
  body.init(nh);
}

void loop()
{
  body.syncro();
  nh.spinOnce();
  delay(1);
}
