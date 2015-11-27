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


#define echoPin1 2 // Echo Pin
#define trigPin1 8 // Trigger Pin
#define echoPin2 3 // Echo Pin
#define trigPin2 9 // Trigger Pin
#define LEDPin 13 // Onboard LED

int maximumRange = 80; // Maximum range needed
int minimumRange = 5; // Minimum range needed
long duration1, duration2, distance1, distance2; // Duration used to calculate distance

int avgNum = 4;
int trigCount = 0;
long output1, output2;
unsigned long pulseWait = 50000;

void setup() {
 Serial.begin (250000);
 pinMode(trigPin1, OUTPUT);
 pinMode(echoPin1, INPUT);
 pinMode(trigPin2, OUTPUT);
 pinMode(echoPin2, INPUT);
 pinMode(LEDPin, OUTPUT); // Use LED indicator (if required)
 
}

void loop() {

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

 delay(50);

 // ....................... SONAR 2 ................................. //

 digitalWrite(trigPin2, LOW); 
 delayMicroseconds(2); 

 digitalWrite(trigPin2, HIGH);
 delayMicroseconds(10); 
 
 digitalWrite(trigPin2, LOW);
 duration2 = pulseIn(echoPin2, HIGH, pulseWait);
 
 //Calculate the distance (in cm) based on the speed of sound.
 distance2 = duration2/58.2;

 delay(50);

// ....................... SERIAL PRINT ................................. //

 if((distance1 <= 230) && (distance1 >= 2)){
  if(distance1 <= maximumRange){
   trigCount = trigCount + 1;
  }
  Serial.println(distance1);
  delayMicroseconds(100);
 }

 if((distance2 <= 230) && (distance2 >= 2)){
  if(distance2 <= maximumRange){
   trigCount = trigCount + 1;
  }
  Serial.print("  ");
  Serial.println(distance2);
  delayMicroseconds(200);
 }


// ......................... LED ........................... //

 if(trigCount >= avgNum){
  trigCount = 0;
  digitalWrite(LEDPin, HIGH);
  Serial.println("STOP!");
  delay(2000);
  digitalWrite(LEDPin, LOW);
 }

}
 

