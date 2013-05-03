#include <Wire.h>
#include <Servo.h>
#include <CapSense.h>


#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include <rosserial_adk_demo_msgs/Led.h>
#include <sensor_msgs/Joy.h>

#include "Button.h"
#include "adk_shield.h"



/* *****************  statically defined ROS objects *****************/

ros::NodeHandle nh;

template<Servo* servo, int i> void servo_cb(const std_msgs::UInt16& cmd ){
     if (cmd.data >= 0 && cmd.data <= 180 ) servo[i].write(cmd.data);
}

ros::Subscriber<std_msgs::UInt16>  sub_servo0("servo/0", servo_cb<servos,0>);
ros::Subscriber<std_msgs::UInt16>  sub_servo1("servo/1", servo_cb<servos,1>);
ros::Subscriber<std_msgs::UInt16>  sub_servo2("servo/2", servo_cb<servos,2>);

std_msgs::Bool button_msg;
Button button1(BUTTON1);
ros::Publisher pub_button1 ("button1", &button_msg);

Button button2(BUTTON2);
ros::Publisher pub_button2 ("button2", &button_msg);

Button button3(BUTTON3);
ros::Publisher pub_button3 ("button3", &button_msg);

Button button_joy(JOY_SWITCH);
ros::Publisher pub_button_joy ("button_joy", &button_msg);

Button * buttons[4];
ros::Publisher* pub_buttons[4];


template<int ledr, int ledg, int ledb>
void led_cb(const rosserial_adk_demo_msgs::Led& led){
    analogWrite(ledr, 255 - (unsigned char) led.r);
    analogWrite(ledg, 255 - (unsigned char) led.g);
    analogWrite(ledb, 255 - (unsigned char) led.b);

}

ros::Subscriber<rosserial_adk_demo_msgs::Led>  
            sub_led_1 ("led1", led_cb<LED1_RED, LED1_GREEN, LED1_BLUE>),
            sub_led_2 ("led2", led_cb<LED2_RED, LED2_GREEN, LED2_BLUE>),
            sub_led_3 ("led3", led_cb<LED3_RED, LED3_GREEN, LED3_BLUE>);

template <int relay_pin>
void relay_cb(const std_msgs::Bool& cmd){
    digitalWrite(relay_pin, cmd.data ? HIGH : LOW);
}

ros::Subscriber<std_msgs::Bool> sub_relay1 ("relay1", relay_cb<RELAY1>);
ros::Subscriber<std_msgs::Bool> sub_relay2 ("relay2", relay_cb<RELAY2>);

std_msgs::Float32 temp_msg;
ros::Publisher pub_temp ("temperature", &temp_msg);

std_msgs::UInt16 light_msg;
ros::Publisher pub_light ("light", &light_msg);

sensor_msgs::Joy joy_msg;
float axes[2];
char frame_id[] = "joy_stick";
ros::Publisher pub_joy ("joy", &joy_msg);

std_msgs::Bool touch_msg;
ros::Publisher pub_touch ("touch", &touch_msg);

void setup()
{

    setup_shield();
    
    //lets put the buttons and their publishers in identically indexed
    //arrays so we can access them more easily
    buttons[0] = &button1; buttons[1]=&button2; buttons[2]=&button3;
    buttons[3] = &button_joy;
    
    pub_buttons[0] = &pub_button1;     pub_buttons[1] = &pub_button2; 
    pub_buttons[2] = &pub_button3;     pub_buttons[3] = &pub_button_joy;

    //Set up all the ros subscribers
    
    nh.initNode();
    
    nh.subscribe(sub_servo0);
    nh.subscribe(sub_servo1);
    nh.subscribe(sub_servo2);
    
    nh.subscribe(sub_led_1);
    nh.subscribe(sub_led_2);
    nh.subscribe(sub_led_3);
    
    nh.subscribe(sub_relay1);
    nh.subscribe(sub_relay2);
    
    nh.advertise(pub_button1);
    nh.advertise(pub_button2);
    nh.advertise(pub_button3);
    nh.advertise(pub_button_joy);
    
    nh.advertise(pub_temp);
    nh.advertise(pub_light);
    nh.advertise(pub_touch);
    nh.advertise(pub_joy);
    
    
    joy_msg.axes = axes;
    joy_msg.axes_length =2;
    joy_msg.header.frame_id = frame_id;
}


unsigned long temp_pub_time;
unsigned long light_pub_time;
unsigned long joy_pub_time;
unsigned long touch_robot_time;


void loop()
{
	nh.spinOnce();
    
    //Buttons
    for (int i =0; i<4; ++i){
        if (buttons[i]->changed()){
            button_msg.data = buttons[i]->pressed();
            pub_buttons[i]->publish(&button_msg);
        }
    }    
    
    //temperature sensor
    if (millis() > temp_pub_time){
        temp_msg.data = (analogRead(TEMP_SENSOR)*4.9 -400)/19.5;
        pub_temp.publish(&temp_msg);
        temp_pub_time = millis() + 1000;
    }

    
    //light sensor
    if (millis() > light_pub_time){
        light_msg.data = analogRead(LIGHT_SENSOR);
        pub_light.publish(&light_msg);
        light_pub_time = millis() + 100;
    }
    
    //joystick
    if (millis() > joy_pub_time){
        int x, y;
        read_joystick(&x, &y);
        joy_msg.header.stamp = nh.now();
        joy_msg.axes[0] = y; joy_msg.axes[0] /=100;
        joy_msg.axes[1] = x; joy_msg.axes[1] /=100;
        pub_joy.publish(&joy_msg);
    }
    //touch robot
    if (millis() > touch_robot_time){
        bool c0 = touch_robot.capSense(5) > 750;
			if (c0 != prior_robot_state) {
                touch_msg.data  = c0;
                pub_touch.publish(&touch_msg);
				prior_robot_state = c0;
			}
    }
	delay(10);
}
