/*
 HC-SR04 Ping distance sensor:
 VCC to arduino 5v 
 GND to arduino GND
 Echo to Arduino pin 7 
 Trig to Arduino pin 8
 
 This sketch originates from Virtualmix: http://goo.gl/kJ8Gl
 Has been modified by Winkle ink here: http://winkleink.blogspot.com.au/2012/05/arduino-hc-sr04-ultrasonic-distance.html
 And modified further by ScottC here: http://arduinobasics.blogspot.com.au/2012/11/arduinobasics-hc-sr04-ultrasonic-sensor.html
 on 10 Nov 2012.
 */
#include <ros.h>
#include <sensor_msgs/Range.h>

#define echoPin1 2 // Echo Pin
#define trigPin1 8 // Trigger Pin
#define echoPin2 3 // Echo Pin
#define trigPin2 9 // Trigger Pin
#define LEDPin 13 // Onboard LED

//ROS serial node for publishing imu data
ros::NodeHandle  nh;
sensor_msgs::Range sensor1_msg;
sensor_msgs::Range sensor2_msg;
ros::Publisher sensor1_pub("sonar1", &sensor1_msg);
ros::Publisher sensor2_pub("sonar2", &sensor2_msg);


int maximumRange = 40; // Maximum range needed
int minimumRange = 5; // Minimum range needed
long duration1, duration2, distance1, distance2; // Duration used to calculate distance

int avgNum = 10;
int iteration;
long output1, output2;
unsigned long pulseWait = 10000;

void setup() {
 nh.getHardware()->setBaud(115200);
 //initalize the ros node
 nh.initNode();
 nh.advertise(sensor1_pub);
 nh.advertise(sensor2_pub);
 
 // Wait for ROSserial to connect
 while (!nh.connected()) 
 {
  nh.spinOnce();
 }
 nh.loginfo("ROS Arduino Sonar started.");
 pinMode(trigPin1, OUTPUT);
 pinMode(echoPin1, INPUT);
 pinMode(trigPin2, OUTPUT);
 pinMode(echoPin2, INPUT);
 pinMode(LEDPin, OUTPUT); // Use LED indicator (if required)
 //set up some constant values for the sensor message
 sensor1_msg.radiation_type = 1;
 sensor2_msg.radiation_type = 1;
 //these values were taken from the datasheet
 sensor1_msg.field_of_view = 0.261799;
 sensor1_msg.min_range = 0.02;
 sensor1_msg.max_range = 5;
 
 sensor1_msg.header.frame_id = "sonar1_link";
 sensor2_msg.field_of_view = 0.261799;
 sensor2_msg.min_range = 0.02;
 sensor2_msg.max_range = 5;
 
 sensor2_msg.header.frame_id = "sonar2_link";

}

void loop() {

//if (iteration == avgNum){
//  iteration = 0;
//  output1 = 0;
//  output2 = 0;
//}
//
// iteration = iteration +1;

  // ....................... SONAR 1 ................................. //

  
/* The following trigPin/echoPin cycle is used to determine the
 distance of the nearest object by bouncing soundwaves off of it. */ 
 digitalWrite(trigPin1, LOW); 
 delayMicroseconds(2); 

 digitalWrite(trigPin1, HIGH);
 delayMicroseconds(10); 
 
 digitalWrite(trigPin1, LOW);
 duration1 = pulseIn(echoPin1, HIGH, pulseWait);
 
 //Calculate the distance (in cm) based on the speed of sound.
 distance1 = duration1/58.2;
 //delay(50);
 // ....................... SONAR 2 ................................. //

 digitalWrite(trigPin2, LOW); 
 delayMicroseconds(2); 

 digitalWrite(trigPin2, HIGH);
 delayMicroseconds(10); 
 
 digitalWrite(trigPin2, LOW);
 duration2 = pulseIn(echoPin2, HIGH, pulseWait);
 
 //Calculate the distance (in cm) based on the speed of sound.
 distance2 = duration2/58.2;

 if(nh.connected())
 {
  sensor1_msg.range = distance1;
  sensor1_msg.header.stamp = nh.now();
  sensor1_pub.publish(&sensor1_msg);
  sensor2_msg.range = distance2;
  sensor2_msg.header.stamp = nh.now();
  sensor2_pub.publish(&sensor2_msg);

 }
 nh.spinOnce();
 
 //delay(50);

// ....................... SERIAL PRINT ................................. //

// if(iteration <= avgNum){
//  output1 = output1 + distance1;
//  output2 = output2 + distance2;
// }
// 
// if(iteration == avgNum){
//  output1 = output1/avgNum;
//  output2 = output2/avgNum;
//
//  Serial.print(output1);
//  Serial.print("/");
//  Serial.println(output2);
// }
//
//if((distance1 <= 230) && (distance1 >= 2)){
//  Serial.println(distance1);
//  delayMicroseconds(100);
// }

// if((distance2 <= 230) && (distance2 >= 2)){
//  Serial.print("  ");
//  Serial.println(distance2);
//  delayMicroseconds(200);
// }

// ......................... LED ........................... //

 if ((distance2 <= maximumRange && distance2 >= minimumRange) || (distance1 >= minimumRange && distance1 <= maximumRange) ) {
   digitalWrite(LEDPin, LOW);
 }
 else{
   digitalWrite(LEDPin, HIGH);
 }

}
