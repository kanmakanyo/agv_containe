#include <ros.h>
#include <geometry_msgs/TwistStamped.h>
#define BAUD_ROS_SERIAL 500000
ros::NodeHandle  nh;
geometry_msgs::TwistStamped pub_msg;
ros::Publisher pub("nano_ultrasonic_front", &pub_msg);
int ros_period = 50; // Milisecond, 20 Hz
unsigned long ros_time = 0; // Milisecond

//A
#define ECHOPIN_A 2
#define TRIGPIN_A 3
//B
#define ECHOPIN_B 4
#define TRIGPIN_B 5
//C
#define ECHOPIN_C 6
#define TRIGPIN_C 7
//D
#define ECHOPIN_D 8
#define TRIGPIN_D 9
//E
#define ECHOPIN_E 10
#define TRIGPIN_E 11

#define SOUND_SPEED 0.034
#define PULSEIN_TIMEOUT 10000//26000

float duration2distance(unsigned int duration) {
    if (duration == 0) {
        return 999;
    }
    else {
        return float(duration)*SOUND_SPEED/2;
    }
}

void setup(){
    // ROS
    nh.getHardware()-> setBaud(BAUD_ROS_SERIAL);
    nh.initNode();
    nh.advertise(pub);

    // Serial.begin(9600);

    //A
    pinMode(ECHOPIN_A, INPUT);
    pinMode(TRIGPIN_A, OUTPUT);
    digitalWrite(ECHOPIN_A, HIGH);
    //B
    pinMode(ECHOPIN_B, INPUT);
    pinMode(TRIGPIN_B, OUTPUT);
    digitalWrite(ECHOPIN_B, HIGH);
    //C
    pinMode(ECHOPIN_C, INPUT);
    pinMode(TRIGPIN_C, OUTPUT);
    digitalWrite(ECHOPIN_C, HIGH);
    //D
    pinMode(ECHOPIN_D, INPUT);
    pinMode(TRIGPIN_D, OUTPUT);
    digitalWrite(ECHOPIN_D, HIGH);
    //E
    pinMode(ECHOPIN_E, INPUT);
    pinMode(TRIGPIN_E, OUTPUT);

    digitalWrite(ECHOPIN_E, HIGH);
}

void loop(){
  //A
  digitalWrite(TRIGPIN_A, LOW); // Set the trigger pin to low for 2uS
  delayMicroseconds(2);
  digitalWrite(TRIGPIN_A, HIGH); // Send a 10uS high to trigger ranging
  delayMicroseconds(20);
  digitalWrite(TRIGPIN_A, LOW); // Send pin low again
  unsigned int duration1 = pulseIn(ECHOPIN_A, HIGH, PULSEIN_TIMEOUT); // Read in times pulse
//   float distance1 = float(duration1)*SOUND_SPEED/2;
  float distance1 = duration2distance(duration1);

  //B
  digitalWrite(TRIGPIN_B, LOW); // Set the trigger pin to low for 2uS
  delayMicroseconds(2);
  digitalWrite(TRIGPIN_B, HIGH); // Send a 10uS high to trigger ranging
  delayMicroseconds(20);
  digitalWrite(TRIGPIN_B, LOW); // Send pin low again
  unsigned int duration2 = pulseIn(ECHOPIN_B, HIGH, PULSEIN_TIMEOUT); // Read in times pulse
//   float distance2 = float(duration2)*SOUND_SPEED/2;
  float distance2 = duration2distance(duration2);

  //C
  digitalWrite(TRIGPIN_C, LOW); // Set the trigger pin to low for 2uS
  delayMicroseconds(2);
  digitalWrite(TRIGPIN_C, HIGH); // Send a 10uS high to trigger ranging
  delayMicroseconds(20);
  digitalWrite(TRIGPIN_C, LOW); // Send pin low again
  unsigned int duration3 = pulseIn(ECHOPIN_C, HIGH, PULSEIN_TIMEOUT); // Read in times pulse
//   float distance3 = float(duration3)*SOUND_SPEED/2;
  float distance3 = duration2distance(duration3);

  //D
  digitalWrite(TRIGPIN_D, LOW); // Set the trigger pin to low for 2uS
  delayMicroseconds(2);
  digitalWrite(TRIGPIN_D, HIGH); // Send a 10uS high to trigger ranging
  delayMicroseconds(20);
  digitalWrite(TRIGPIN_D, LOW); // Send pin low again
  unsigned int duration4 = pulseIn(ECHOPIN_D, HIGH, PULSEIN_TIMEOUT); // Read in times pulse
//   float distance4 = float(duration4)*SOUND_SPEED/2;
  float distance4 = duration2distance(duration4);

  //E
  digitalWrite(TRIGPIN_E, LOW); // Set the trigger pin to low for 2uS
  delayMicroseconds(2);
  digitalWrite(TRIGPIN_E, HIGH); // Send a 10uS high to trigger ranging
  delayMicroseconds(20);
  digitalWrite(TRIGPIN_E, LOW); // Send pin low again
  unsigned int duration5 = pulseIn(ECHOPIN_E, HIGH, PULSEIN_TIMEOUT); // Read in times pulse
//   float distance5 = float(duration5)*SOUND_SPEED/2;
  float distance5 = duration2distance(duration5);

//   Serial.print("NANO_ULTRASONIK_FRONT\t");
//   Serial.print("Jarak 1: ");
//   Serial.print(distance1);
//   Serial.print(" cm");
//   Serial.print("\t");
//   Serial.print("Jarak 2: ");
//   Serial.print(distance2);
//   Serial.print(" cm ");
//   Serial.print("\t");
//   Serial.print("Jarak 3: ");
//   Serial.print(distance3);
//   Serial.print(" cm ");
//   Serial.print("\t");
//   Serial.print("Jarak 4: ");
//   Serial.print(distance4);
//   Serial.print(" cm ");
//   Serial.print("\t");
//   Serial.print("Jarak 5: ");
//   Serial.print(distance5);
//   Serial.println(" cm ");
//   delay(50);// Wait 50mS before next ranging

  if ((millis() - ros_time) >= ros_period) { // Publish the pub_msg
    ros_time = millis();
    pub_msg.header.stamp = nh.now();
    pub_msg.twist.linear.x = distance1;
    pub_msg.twist.linear.y = distance2;
    pub_msg.twist.linear.z = distance3;
    pub_msg.twist.angular.x = distance4;
    pub_msg.twist.angular.y = distance5;

    pub.publish( &pub_msg);
  }
  nh.spinOnce();
}
