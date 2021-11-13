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

#define PIN_CEJA_IZQ 11
#define PIN_CEJA_DER 9
#define PIN_CRESTA 3
#define PIN_CUELLO 3
#define PIN_CUERPO 2
#define PIN_RUEDA_IZQ 8
#define PIN_RUEDA_DER 7

#define MIN_CEJA_IZQ 1000
#define MIN_CEJA_DER 1000
#define MIN_CRESTA 1000
#define MIN_CUELLO 1000
#define MIN_CUERPO 1000
#define MIN_RUEDA_IZQ 1000
#define MIN_RUEDA_DER 1000

#define MAX_CEJA_IZQ 2000
#define MAX_CEJA_DER 2000
#define MAX_CRESTA 2000
#define MAX_CUELLO 2000
#define MAX_CUERPO 2000
#define MAX_RUEDA_IZQ 2000
#define MAX_RUEDA_DER 2000

ros::NodeHandle nh;

class Body
{
public:
  /*Funciones*/
  Body()
      : subscriber_("/joy", &Body::set_period_callback, this)
  {
  }
  //    void detach(Servo);
  //    void attach(Servo,int,int,int);

  /*Consignas*/

  void init(ros::NodeHandle &nh)
  {
    /*Attach*/
    //ceja_izq_.attach(PIN_CEJA_IZQ,MIN_CEJA_IZQ,MAX_CEJA_IZQ);
    ceja_der_.attach(PIN_CEJA_DER, MIN_CEJA_DER, MAX_CEJA_DER);
    cresta_.attach(PIN_CRESTA, MIN_CRESTA, MAX_CRESTA);
    cuello_.attach(PIN_CUELLO, MIN_CUELLO, MAX_CUELLO);
    cuerpo_.attach(PIN_CUERPO, MIN_CUERPO, MAX_CUERPO);
    rueda_izq_.attach(PIN_RUEDA_IZQ, MIN_RUEDA_IZQ, MAX_RUEDA_IZQ);
    rueda_der_.attach(PIN_RUEDA_DER, MIN_RUEDA_DER, MAX_RUEDA_DER);
    ceja_izq_consigna_ = 0;
    ceja_der_consigna_ = 5;
    cresta_consigna_ = 0;
    cuello_consigna_ = 0;
    cuerpo_consigna_ = 0;
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
    ceja_izq_.write(ceja_izq_consigna_);
    ceja_der_.write(ceja_der_consigna_);
    cresta_.write(cresta_consigna_);
    cuello_.write(cuello_consigna_);
    cuerpo_.write(cuerpo_consigna_);
    rueda_izq_.write(rueda_izq_consigna_);
    rueda_der_.write(rueda_der_consigna_);
  }

  void set_period_callback(const sensor_msgs::Joy &msg)
  {
    ceja_izq_consigna_ = (msg.axes[1] + 1) / 2 * 175;
    ceja_der_consigna_ = (msg.axes[1] + 1) / 2 * 175;
    cuello_consigna_ = (msg.axes[3] + 1) / 2 * 175;
    cresta_consigna_ = (msg.axes[4] + 1) / 2 * 175;
    cuerpo_consigna_ = (msg.axes[5] + 1) / 2 * 175;
  }

private:
  Servo ceja_izq_;
  Servo ceja_der_;
  Servo cresta_;
  Servo cuello_;
  Servo cuerpo_;
  Servo rueda_izq_;
  Servo rueda_der_;

  uint16_t ceja_izq_consigna_;
  uint16_t ceja_der_consigna_;
  uint16_t cresta_consigna_;
  uint16_t cuello_consigna_;
  uint16_t cuerpo_consigna_;
  uint16_t rueda_izq_consigna_;
  uint16_t rueda_der_consigna_;

  ros::Subscriber<sensor_msgs::Joy, Body> subscriber_;
};

Body body;

void setup()
{
  nh.initNode();
  body.init(nh);
}

void loop()
{
  body.syncro();
  nh.spinOnce();
  delay(1);
}
