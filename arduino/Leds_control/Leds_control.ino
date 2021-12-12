#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include <Adafruit_NeoPixel.h>
#include <ros.h>
#include <std_msgs/ColorRGBA.h>

#define PIN_LEDS 9
#define NUMPIXELS 24
#define LEDSPRECISION 0.0666667
#define R 20
#define G 60
#define B 150
class Leds
{
private:
  uint16_t setpoint_;
  uint16_t prev_setpoint_;
  int state_;
  int iter_idle_;
  Adafruit_NeoPixel tira_;
  ros::Subscriber<std_msgs::ColorRGBA, Leds> subscriber_;

public:
  Leds() : subscriber_("/leds", &Leds::setPointCallback, this)
  {
    tira_ = Adafruit_NeoPixel(NUMPIXELS, PIN_LEDS, NEO_GRB + NEO_KHZ800);
    setpoint_ = 0;
    state_ = 0;
    iter_idle_= 0;
    prev_setpoint_ = -1;
  }
  void init(ros::NodeHandle &nh);
  void setPointCallback(const std_msgs::ColorRGBA &msg);
  void clearTira();
  void run();
  void setStatus(const int value);
};

void Leds::init(ros::NodeHandle &nh)
{
  tira_.begin();
  tira_.setBrightness(100);
  clearTira();
  nh.subscribe(subscriber_);
}
void Leds::run()
{
  if (state_ == 0) // IDLE
  {
    if(iter_idle_ == 0)
    {
      tira_.setPixelColor(NUMPIXELS-1, 0, 0, 0);
      tira_.setPixelColor(iter_idle_, 255, 0, 0);
      iter_idle_++;
    }
    else if (iter_idle_ == NUMPIXELS-1)
    {
      tira_.setPixelColor(iter_idle_-1, 0, 0, 0);
      tira_.setPixelColor(iter_idle_, 255, 0, 0);
      iter_idle_ = 0;
    }
    else
    {
      tira_.setPixelColor(iter_idle_-1, 0, 0, 0);
      tira_.setPixelColor(iter_idle_, 255, 0, 0);
      iter_idle_++;
    }
  }
  else if (state_ == 1) // FLASH
  {
    for (int i = 0; i < NUMPIXELS; i++)
    {
      tira_.setPixelColor(i, 255, 255, 255);
    } 
  }
  else if (state_ == 2) // FOLLOWER
  {
    tira_.clear();
    if(setpoint_ == 0)
    {
      tira_.setPixelColor(NUMPIXELS-1, R, G, B);
      tira_.setPixelColor(setpoint_,   R, G, B);
      tira_.setPixelColor(setpoint_+1, R, G, B);
    }
    else if (setpoint_ == NUMPIXELS-1)
    {
      tira_.setPixelColor(setpoint_-1, R, G, B);
      tira_.setPixelColor(setpoint_,   R, G, B);
      tira_.setPixelColor(0,           R, G, B);
    }
    else
    {
      tira_.setPixelColor(setpoint_-1, R, G, B);
      tira_.setPixelColor(setpoint_,   R, G, B);
      tira_.setPixelColor(setpoint_+1, R, G, B);
    }
  }
  tira_.show();
}
void Leds::setStatus(const int value)
{
  if (value == 0)
  {
    clearTira();
    state_ = 0;
    iter_idle_= 0;
  }
  else if (value == 1)
  {
    clearTira();
    state_ = 1;
  }
  else if (value == 2)
  {
    clearTira();
    state_ = 2;
  }
  else
  {
    clearTira();
    state_ = 0;
    iter_idle_= 0;
  }
}
void Leds::clearTira()
{
  tira_.clear();
  tira_.show();
}
void Leds::setPointCallback(const std_msgs::ColorRGBA &msg)
{
  setpoint_ = int(msg.a);
}

// DECLARE VARS
ros::NodeHandle nh;
Leds leds;

void setup()
{
  nh.initNode();
  leds.init(nh);
  leds.setStatus(2);
}

void loop()
{
  leds.run();
  nh.spinOnce();
  delay(100);
}
