#include <avr/interrupt.h>
#include <ros.h>
  #include <agv_container/Nano_Tacho.h>
#define BAUD 500000

#define INFINITE_PULSE_WIDTH 9999999 // in microsecond
#define TIMEOUT_NEW_PULSE 1000000 // in microsecond
#define pulse_4 8
int sensor = 2; // Hall sensor at pin 2
volatile unsigned long counts1, counts2, counts3, counts4;
volatile unsigned long prev1 = 0;
volatile unsigned long prev2 = 0;
volatile unsigned long prev3 = 0;
volatile unsigned long prev4 = 0;
unsigned int rpm; //unsigned gives only positive values
unsigned long previoustime;

ros::NodeHandle  nh;
agv_container::Nano_Tacho pub_msg;
ros::Publisher pub("tacho_data", &pub_msg);

// int ros_period = 20; // Milisekon, 50 Hz
int ros_period = 20000; // microsecond, 50 Hz
unsigned long ros_time = 0;

// ISR Routine
//void count1_function() {counts1++;}
//void count2_function() {counts2++;}
//ISR(PCINT1_vect) {counts3++;}
//ISR(PCINT0_vect) {counts4++;}

// ISR Routine vD1
unsigned long buffer_now = 0;

void count1_function() {
  // buffer_now = millis();
  buffer_now = micros();
  // counts1 = 1000 / (buffer_now-prev1);
  counts1 = buffer_now-prev1;
  prev1 = buffer_now;
}

void count2_function() {
  // buffer_now = millis();
  buffer_now = micros();
  // counts2 = 1000 / (buffer_now-prev2);
  counts2 = buffer_now-prev2;
  prev2 = buffer_now;
}

ISR(PCINT1_vect) {
  // buffer_now = millis();
  buffer_now = micros();
  // counts3 = 1000 / (buffer_now-prev3);
  counts3 = buffer_now-prev3;
  prev3 = buffer_now;
}

ISR(PCINT0_vect) {
  // buffer_now = millis();
  buffer_now = micros();
  // counts4 = 1000 / (buffer_now-prev4);
  counts4 = buffer_now-prev4;
  prev4 = buffer_now;
}


void setup() {
  // ROS
  nh.getHardware()-> setBaud(BAUD);
  nh.initNode();
  nh.advertise(pub);

  // Serial.begin(9600);
  for (int i = 9; i>5; i--) {pinMode(i, OUTPUT);}
  attachInterrupt(digitalPinToInterrupt(2), count1_function, RISING);
  attachInterrupt(digitalPinToInterrupt(3), count2_function, RISING);
//  attachInterrupt(digitalPinToInterrupt(4), count3_function, LOW);
//  attachInterrupt(digitalPinToInterrupt(5), count4_function, LOW);
  for (int i = 5; i>1; i--) {pinMode(i, INPUT);}
  cli();
  PCICR|=0b00000011; // Enables Ports B and D Pin Change Interrupts
  PCMSK1|=0b00010000; // Enables PCINT20
  PCMSK0|=0b00000100; // Enables PCINT2
  sei(); 
  counts1 = 0; counts2 = 0; counts3 = 0; counts4 = 0;
  rpm = 0;previoustime = 0; //Initialise the values
}

void loop()
{
  //  delay(1000);//Update RPM every second
//  detachInterrupt(0); //Interrupts are disabled
//  detachInterrupt(1); //Interrupts are disabled
  int val1 = analogRead(A0);
  int val2 = analogRead(A1);
  int val3 = analogRead(A2);
  int val4 = analogRead(A3);
//  Serial.println(val2);

  if (val1 > 800) {digitalWrite(9, 1);}
  else {digitalWrite(9,0);}
  
  if (val2 > 800) {digitalWrite(8, 1);}
  else {digitalWrite(8, 0);}
  
  if (val3 > 700) {digitalWrite(7, 1);}
  else {digitalWrite(7, 0);}
  
  if (val4 > 800) {digitalWrite(6, 1);}
  else {digitalWrite(6, 0);}
  
//  if (millis() - previoustime >= 1000) {
//    previoustime = millis();
////    int rpm = (counts4/(2*8))*60;/
//    // Serial.print("count1 : "); Serial.print("\t"); Serial.print(counts1); Serial.print("\t");
//    // Serial.print("count2 : "); Serial.print("\t"); Serial.print(counts2); Serial.print("\t");
//    // Serial.print("count3 : "); Serial.print("\t"); Serial.print(counts3); Serial.print("\t");
//    // Serial.print("count4 : "); Serial.print("\t"); Serial.println(counts4/2);
//    pub_msg.FWR_pulse = counts1;
//    pub_msg.FWL_pulse = counts2;
//    pub_msg.RWR_pulse = counts3/2;
//    pub_msg.RWL_pulse = counts4/2;
//
//    counts1 = 0;counts2 = 0;counts3 = 0;counts4 = 0; // Reset counter
////        Serial.print("rpm :");
////        Serial.println(rpm);
//  }

  // // to ROS
  // if ((millis() - ros_time) >= ros_period) { // Publish the pub_msg
  //   ros_time = millis();
  //   pub_msg.header.stamp = nh.now();

  //   if (ros_time-prev1 > TIMEOUT_NEW_PULSE) {pub_msg.FWR_pulse = INFINITE_PULSE_WIDTH;}
  //   else {pub_msg.FWR_pulse = counts1;}

  //   if (ros_time-prev2 > TIMEOUT_NEW_PULSE) {pub_msg.FWL_pulse = INFINITE_PULSE_WIDTH;}
  //   else {pub_msg.FWL_pulse = counts2;}

  //   if (ros_time-prev3 > TIMEOUT_NEW_PULSE) {pub_msg.RWR_pulse = INFINITE_PULSE_WIDTH;}
  //   else {pub_msg.RWR_pulse = counts3/2;}

  //   if (ros_time-prev4 > TIMEOUT_NEW_PULSE) {pub_msg.RWL_pulse = INFINITE_PULSE_WIDTH;}
  //   else {pub_msg.RWL_pulse = counts4/2;}

  //   pub.publish( &pub_msg);
  // }
  // nh.spinOnce();

  // to ROS
  if ((micros() - ros_time) >= ros_period) { // Publish the pub_msg
    ros_time = micros();
    pub_msg.header.stamp = nh.now();

    if (ros_time-prev1 > TIMEOUT_NEW_PULSE) {pub_msg.FWR_pulse = INFINITE_PULSE_WIDTH;}
    else {pub_msg.FWR_pulse = counts1;}

    if (ros_time-prev2 > TIMEOUT_NEW_PULSE) {pub_msg.FWL_pulse = INFINITE_PULSE_WIDTH;}
    else {pub_msg.FWL_pulse = counts2;}

    if (ros_time-prev3 > TIMEOUT_NEW_PULSE) {pub_msg.RWR_pulse = INFINITE_PULSE_WIDTH;}
    else {pub_msg.RWR_pulse = counts3/2;}

    if (ros_time-prev4 > TIMEOUT_NEW_PULSE) {pub_msg.RWL_pulse = INFINITE_PULSE_WIDTH;}
    else {pub_msg.RWL_pulse = counts4/2;}

    pub.publish(&pub_msg);
  }
  nh.spinOnce();
  
}
