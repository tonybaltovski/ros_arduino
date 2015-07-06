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

#define echoPin1 25 // Echo Pin
#define trigPin1 22 // Trigger Pin
#define echoPin2 29 // Echo Pin
#define trigPin2 26 // Trigger Pin
#define echoPin3 33 // Echo Pin
#define trigPin3 30 // Trigger Pin
#define echoPin4 37 // Echo Pin
#define trigPin4 34 // Trigger Pin
#define echoPin5 41 // Echo Pin
#define trigPin5 38 // Trigger Pin
#define echoPin6 45 // Echo Pin
#define trigPin6 42 // Trigger Pin
#define echoPin7 49 // Echo Pin
#define trigPin7 46 // Trigger Pin
#define echoPin8 53 // Echo Pin
#define trigPin8 50 // Trigger Pin
#define LEDPin 13 // Onboard LED

//ROS serial node for publishing imu data
ros::NodeHandle  nh;
sensor_msgs::Range sensor1_msg;
sensor_msgs::Range sensor2_msg;
sensor_msgs::Range sensor3_msg;
sensor_msgs::Range sensor4_msg;
sensor_msgs::Range sensor5_msg;
sensor_msgs::Range sensor6_msg;
sensor_msgs::Range sensor7_msg;
sensor_msgs::Range sensor8_msg;
sensor_msgs::Range threshold_msg;

ros::Publisher sensor1_pub("sonar1", &sensor1_msg);
ros::Publisher sensor2_pub("sonar2", &sensor2_msg);
ros::Publisher sensor3_pub("sonar3", &sensor3_msg);
ros::Publisher sensor4_pub("sonar4", &sensor4_msg);
ros::Publisher sensor5_pub("sonar5", &sensor5_msg);
ros::Publisher sensor6_pub("sonar6", &sensor6_msg);
ros::Publisher sensor7_pub("sonar7", &sensor7_msg);
ros::Publisher sensor8_pub("sonar8", &sensor8_msg);
ros::Publisher threshold_pub("threshold", &threshold_msg);


int maximumRange = 40; // Maximum range needed
int minimumRange = 5; // Minimum range needed
double duration1, duration2, duration3, duration4, duration5, duration6, duration7, duration8; // Duration used to calculate distance
double distance[8];

int iteration;
long output1, output2, output3, output4, output5, output6, output7, output8;
unsigned long pulseWait = 100000;



void setup() {

 nh.getHardware()->setBaud(115200);
//nh.getHardware()->setBaud(57600);
 //initalize the ros node
 nh.initNode();
 nh.advertise(sensor1_pub);
 nh.advertise(sensor2_pub);
 nh.advertise(sensor3_pub);
 nh.advertise(sensor4_pub);
 nh.advertise(sensor5_pub);
 nh.advertise(sensor6_pub);
 nh.advertise(sensor7_pub);
 nh.advertise(sensor8_pub);
 nh.advertise(threshold_pub);
 
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
 pinMode(trigPin3, OUTPUT);
 pinMode(echoPin3, INPUT);
 pinMode(trigPin4, OUTPUT);
 pinMode(echoPin4, INPUT);
 pinMode(trigPin5, OUTPUT);
 pinMode(echoPin5, INPUT);
 pinMode(trigPin6, OUTPUT);
 pinMode(echoPin6, INPUT);
 pinMode(trigPin7, OUTPUT);
 pinMode(echoPin7, INPUT);
 pinMode(trigPin8, OUTPUT);
 pinMode(echoPin8, INPUT);
 pinMode(LEDPin, OUTPUT); // Use LED indicator (if required)
 
 //set up some constant values for the sensor message
 sensor1_msg.radiation_type = 1;
 sensor2_msg.radiation_type = 1;
 sensor3_msg.radiation_type = 1;
 sensor4_msg.radiation_type = 1;
 sensor5_msg.radiation_type = 1;
 sensor6_msg.radiation_type = 1;
 sensor7_msg.radiation_type = 1;
 sensor8_msg.radiation_type = 1;
 threshold_msg.radiation_type = 1;
 
 //these values were taken from the datasheet
 sensor1_msg.field_of_view = 0.261799;
 sensor1_msg.min_range = 0.02;
 sensor1_msg.max_range = 5; 
 sensor1_msg.header.frame_id = "sonar1_link";
 
 sensor2_msg.field_of_view = 0.261799;
 sensor2_msg.min_range = 0.02;
 sensor2_msg.max_range = 5; 
 sensor2_msg.header.frame_id = "sonar2_link";

 sensor3_msg.field_of_view = 0.261799;
 sensor3_msg.min_range = 0.02;
 sensor3_msg.max_range = 5; 
 sensor3_msg.header.frame_id = "sonar3_link";

 sensor4_msg.field_of_view = 0.261799;
 sensor4_msg.min_range = 0.02;
 sensor4_msg.max_range = 5; 
 sensor4_msg.header.frame_id = "sonar4_link";

 sensor5_msg.field_of_view = 0.261799;
 sensor5_msg.min_range = 0.02;
 sensor5_msg.max_range = 5; 
 sensor5_msg.header.frame_id = "sonar5_link";

 sensor6_msg.field_of_view = 0.261799;
 sensor6_msg.min_range = 0.02;
 sensor6_msg.max_range = 5; 
 sensor6_msg.header.frame_id = "sonar6_link";

 sensor7_msg.field_of_view = 0.261799;
 sensor7_msg.min_range = 0.02;
 sensor7_msg.max_range = 5; 
 sensor7_msg.header.frame_id = "sonar7_link";
 
 sensor8_msg.field_of_view = 0.261799;
 sensor8_msg.min_range = 0.02;
 sensor8_msg.max_range = 5; 
 sensor8_msg.header.frame_id = "sonar8_link";

 threshold_msg.max_range = maximumRange;
 threshold_msg.header.frame_id = "threshold_link";

 
}


void loop() {

int numSonar = 8;
int avgNum = 10;
double run_avg_values[numSonar][avgNum];  // rows, columns
double run_avg[numSonar][2];  // rows, columns. Row 1 is the running average and row 2 is the cell to be replace by new data

int j = 0;
int i = 0;

 // ...................................... Array_SETUP .................................................//

 while (i < numSonar){
    while (j < avgNum){
      run_avg_values[i][j] = 0;
      j = j + 1;
    }
    run_avg[i][0] = 0;  
    run_avg[i][1] = 0;
    i = i + 1; 
  }


 // ................................................................................................//



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
 duration1 = -1;
 duration1 = pulseIn(echoPin1, HIGH, pulseWait);
 while(duration1 < 0){
 }
 
 //Calculate the distance (in cm) based on the speed of sound.
 distance[0] = duration1/58.2;

 
 // ....................... SONAR 2 ................................. //

 digitalWrite(trigPin2, LOW); 
 delayMicroseconds(2); 

 digitalWrite(trigPin2, HIGH);
 delayMicroseconds(10); 
 
 digitalWrite(trigPin2, LOW);
 duration2 = -1;
 duration2 = pulseIn(echoPin2, HIGH, pulseWait);
 while(duration2 < 0){
 }
 
 //Calculate the distance (in cm) based on the speed of sound.
 distance[1] = duration2/58.2;



// ....................... SONAR 3 ................................. //

 digitalWrite(trigPin3, LOW); 
 delayMicroseconds(2); 

 digitalWrite(trigPin3, HIGH);
 delayMicroseconds(10); 
 
 digitalWrite(trigPin3, LOW);
 duration3 = -1;
 duration3 = pulseIn(echoPin3, HIGH, pulseWait);
 while(duration3 < 0){
 }
 //Calculate the distance (in cm) based on the speed of sound.
 distance[2] = duration3/58.2;

 
// ....................... SONAR 4 ................................. //

 digitalWrite(trigPin4, LOW); 
 delayMicroseconds(2); 

 digitalWrite(trigPin4, HIGH);
 delayMicroseconds(10); 
 
 digitalWrite(trigPin4, LOW);
 duration4 = -1;
 duration4 = pulseIn(echoPin4, HIGH, pulseWait);
 while(duration4 < 0){
 }
 
 //Calculate the distance (in cm) based on the speed of sound.
 distance[3] = duration4/58.2;

// ....................... SONAR 5 ................................. //

 digitalWrite(trigPin5, LOW); 
 delayMicroseconds(2); 

 digitalWrite(trigPin5, HIGH);
 delayMicroseconds(10); 
 
 digitalWrite(trigPin5, LOW);
 duration5 = -1;
 duration5 = pulseIn(echoPin5, HIGH, pulseWait);
 while(duration5 < 0){
 }
 
 //Calculate the distance (in cm) based on the speed of sound.
 distance[4] = duration5/58.2;
 
// ....................... SONAR 6 ................................. //

 digitalWrite(trigPin6, LOW); 
 delayMicroseconds(2); 

 digitalWrite(trigPin6, HIGH);
 delayMicroseconds(10); 
 
 digitalWrite(trigPin6, LOW);
 duration6 = -1;
 duration6 = pulseIn(echoPin6, HIGH, pulseWait);
 while(duration6 < 0){
 }
 
 //Calculate the distance (in cm) based on the speed of sound.
 distance[5] = duration6/58.2;
 
// ....................... SONAR 7 ................................. //

 digitalWrite(trigPin7, LOW); 
 delayMicroseconds(2); 

 digitalWrite(trigPin7, HIGH);
 delayMicroseconds(10); 
 
 digitalWrite(trigPin7, LOW);
 duration7 = -1;
 duration7 = pulseIn(echoPin7, HIGH, pulseWait);
 while(duration7 < 0){
 }
 
 //Calculate the distance (in cm) based on the speed of sound.
 distance[6] = duration7/58.2;
 
// ....................... SONAR 8 ................................. //

 digitalWrite(trigPin8, LOW); 
 delayMicroseconds(2); 

 digitalWrite(trigPin8, HIGH);
 delayMicroseconds(10); 
 
 digitalWrite(trigPin8, LOW);
 duration8 = -1;
 duration8 = pulseIn(echoPin8, HIGH, pulseWait);
 while(duration8 < 0){
 }
 
 //Calculate the distance (in cm) based on the speed of sound.
 distance[7] = duration8/58.2;

 

 if(nh.connected())
 {

  i = 0;  //sonar loop counter
  while(i < numSonar){
    if((distance[i] <= 120) && (distance[i] >= 2)){  // only calculate stuff if the new data is good data
      run_avg_values[i][int(run_avg[i][1])] = distance[i];
      run_avg[i][1] = run_avg[i][1] + 1;
      if(run_avg[i][1] >= avgNum){   // if the queue is full of data calculate the average
        run_avg[i][1] = 0;
        j = 0;
        while(j < avgNum){  // add all of the values together in the average value variable for that sonar
          run_avg[i][0] = run_avg[i][0] + run_avg_values[i][j];
          j = j + 1;
        }
        run_avg[i][0] = run_avg[i][0]/avgNum;   // calculate the average from the sum of stored values

        // ...................... put sensor data into sonar messages ............................//

        if(i = 0){
          sensor1_msg.range = run_avg[i][0];
          sensor1_msg.header.stamp = nh.now();
          sensor1_pub.publish(&sensor1_msg);
        }
        else if(i = 1){
          sensor2_msg.range = run_avg[i][0];
          sensor2_msg.header.stamp = nh.now();
          sensor2_pub.publish(&sensor2_msg);
        }
        else if(i = 2){
          sensor3_msg.range = run_avg[i][0];
          sensor3_msg.header.stamp = nh.now();
          sensor3_pub.publish(&sensor3_msg);
        }
        else if(i = 3){
          sensor4_msg.range = run_avg[i][0];
          sensor4_msg.header.stamp = nh.now();
          sensor4_pub.publish(&sensor4_msg);
        }
        else if(i = 4){
          sensor5_msg.range = run_avg[i][0];
          sensor5_msg.header.stamp = nh.now();
          sensor5_pub.publish(&sensor5_msg);
        }
        else if(i = 5){
          sensor6_msg.range = run_avg[i][0];
          sensor6_msg.header.stamp = nh.now();
          sensor6_pub.publish(&sensor6_msg);
        }
        else if(i = 6){
          sensor7_msg.range = run_avg[i][0];
          sensor7_msg.header.stamp = nh.now();
          sensor7_pub.publish(&sensor7_msg);
        }
        else if(i = 7){
          sensor8_msg.range = run_avg[i][0];
          sensor8_msg.header.stamp = nh.now();
          sensor8_pub.publish(&sensor8_msg);
        }


      // ....................... end of sensor message stuffing .......................... //
                
      }
    }
    i = i + 1;
  }

  threshold_msg.header.stamp = nh.now();
  threshold_msg.max_range = maximumRange;
  threshold_pub.publish(&threshold_msg);

 }
 nh.spinOnce();
 

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

// if ((distance2 <= maximumRange && distance2 >= minimumRange) || (distance1 >= minimumRange && distance1 <= maximumRange) ) {
//   digitalWrite(LEDPin, LOW);
// }
// else{
//   digitalWrite(LEDPin, HIGH);
// }
//
}
