/*
 * rosserial PubSub Example
 * Prints "hello world!" and toggles led
 */

#include "rosADK.h"
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <util/delay.h>

//ros::NodeHandle_<ArduinoHardware,1,1,100,100>  nh;

ros::NodeHandle  nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char rsp_str[]="responding to toggle";

void messageCb( const std_msgs::Empty& toggle_msg){
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
  
  //str_msg.data = rsp_str;
  //chatter.publish( &str_msg );
}

ros::Subscriber<std_msgs::Empty> sub("toggle_led", messageCb );



char hello[13] = "hello world!";

void setup()
{
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);
}

void loop()
{
  str_msg.data = hello;
  chatter.publish( &str_msg );

  nh.spinOnce();
  _delay_ms(500);
}
