/*
 * rosserial Servo Control Example
 *
 * This sketch demonstrates the control of hobby R/C servos
 * using ROS and the arduiono
 * 
 * For the full tutorial write up, visit
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 * For more information on the Arduino Servo Library
 * Checkout :
 * http://www.arduino.cc/en/Reference/Servo
 */

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle  nh;
//ros::NodeHandle  aw;
std_msgs::UInt16 aws_msg;
ros::Publisher aws("lock", &aws_msg);

int start;
int target;

Servo servo;
Servo servo2;
Servo servo3;

void servo_cb( const std_msgs::UInt16& cmd_msg){
  //servo.write(cmd_msg.data); //set servo angle, should be from 0-180  
  start = servo.read();
  target = cmd_msg.data;
  aws_msg.data = 1;
  aws.publish(&aws_msg);
  if (target>180){
    target = target - 180;
    target = target/2;
    if (target > 1){
      target = start + target;
      //target = start + 1;
      for (int pos = start; pos <= target; pos += 1) { // goes from 0 degrees to 180 degrees
        // in steps of 1 degree
        servo.write(pos);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15ms for the servo to reach the position
      }
    }
  }
  else if (target<180){
    //target = target - 180;
    target = target/2;
    if (start-target > 1){
      target = start - target;
      //target = start - 1;
      for (int pos = start; pos >= target; pos -= 1) { // goes from 0 degrees to 180 degrees
        // in steps of 1 degree
        servo.write(pos);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15ms for the servo to reach the position
      }
    }
  }
  aws_msg.data = 0;
  aws.publish(&aws_msg);
  //digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}


ros::Subscriber<std_msgs::UInt16> sub("servo", servo_cb);

//new added
void servo_cb2( const std_msgs::UInt16& cmd_msg)
{
  int current = servo2.read();
  int next = cmd_msg.data;

  if (next > current){
    for (int pos = current; pos <= next; pos += 1) { // goes from 0 degrees to 180 degrees
        // in steps of 1 degree
        servo2.write(pos);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15ms for the servo to reach the position
    }
  }
  else{
    for (int pos = current; pos >= next; pos -= 1) { // goes from 0 degrees to 180 degrees
        // in steps of 1 degree
        servo2.write(pos);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15ms for the servo to reach the position
    }
  }
  
}
ros::Subscriber<std_msgs::UInt16> sub2("servo2", servo_cb2);

void servo_cb3( const std_msgs::UInt16& cmd_msg)
{
  int current = servo3.read();
  int next = cmd_msg.data;

  if (next > current){
    for (int pos = current; pos <= next; pos += 1) { // goes from 0 degrees to 180 degrees
        // in steps of 1 degree
        servo3.write(pos);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15ms for the servo to reach the position
    }
  }
  else{
    for (int pos = current; pos >= next; pos -= 1) { // goes from 0 degrees to 180 degrees
        // in steps of 1 degree
        servo3.write(pos);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15ms for the servo to reach the position
    }
  }
}
ros::Subscriber<std_msgs::UInt16> sub3("servo3", servo_cb3);

void setup(){
  pinMode(13, OUTPUT);

  nh.initNode();
//  aw.initNode();
  nh.advertise(aws);
  nh.subscribe(sub);
  nh.subscribe(sub2);
  nh.subscribe(sub3);
  
  servo.attach(9); //attach it to pin 9
  servo2.attach(8);
  servo3.attach(7);
  //initiallize
  servo.write(100);
  servo2.write(45);
  servo3.write(115);
}

void loop(){
  nh.spinOnce();
  delay(1);
}
