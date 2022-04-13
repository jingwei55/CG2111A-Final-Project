///**************************************************************************************************
// * ultrasonic.h
// * Setup ultrasonic sensor
// * Includes both bare-metal and non-bare-metal code
// * 
// * Functions:
// * void setupUS - call to setup US sensor
// * int getDistUS - returns distance of anything in front
//**************************************************************************************************/
//
////=================================================================================================
//  //Constants:
//  #define USTRIG1 12 //Ultrasonic Sensor 1 Trigger Pin pb4
//  #define USECHO1 13 //Echo Pin 1 pb5
//  #define USTRIG2 7 //Ultrasonic Sensor 2 Trigger Pin pd7
//  #define USECHO2 4 //Echo Pin 2 pd4
//  // blue is 7 T, yellow is 4 E
////=================================================================================================*/
//
////Global Variables
//long duration1, duration2;
//int distance[2];
//
//void setupUS() {
//  DDRB |= 0b00010000;
//  DDRD |= 0b10000000;
//  DDRB &= 0b11011111;
//  DDRD &= 0b11101111;
//}
//
////bare-metal code
//void getDistUS(int *dst) { //returns the length of an object in front
//  PORTB &= 0b11101111; //Clear the trigPin1
//  PORTD &= 0b01111111; //Clear the trigPin2
//  _delay_ms(2);
//  
//  PORTB |= 0b00010000; //Sets the trigPin1 on HIGH state for 10 micro seconds
//  PORTD |= 0b10000000; //Sets the trigPin2 on HIGH state for 10 micro seconds
//  _delay_ms(20);
//  PORTB &= 0b11101111; //Clear
//  PORTD &= 0b01111111; //Clear
//  
//  duration1 = pulseIn(USECHO1, HIGH); //Reads the echoPin1, returns the sound wave travel time in microseconds
//  duration2 = pulseIn(USECHO2, HIGH); //Reads the echoPin2, returns the sound wave travel time in microseconds
//
//  
//  dst[0] = duration1*0.034/2; //Calculate the distance1
//  dst[1] = duration2*0.034/2; //Calculate the distance2
//
//  
//  //return distance;
//}
//
////non-bare-metal code
///*void setupUS() {
//  pinMode(trigPin, OUTPUT);
//  pinMode(echoPin, INPUT);
//}
//
//int getDistUS() {
//PORTB &= 0b11101111; //Clear the trigPin
//  digitalWrite(USTRIG, LOW);
//  _delay_ms(2);
//  // Sets the trigPin on HIGH state for 10 micro seconds
//  PORTB |= 0b00010000;
//  _delay_ms(10);
//
//  digitalWrite(USECHO, LOW);
//  // Reads the echoPin, returns the sound wave travel time in microseconds
//  duration = pulseIn(USECHO, HIGH);
//  // Calculating the distance
//  distance= duration*0.034/2;
//  // Prints the distance on the Serial Monitor
//  if (distance != 0) {
//    Serial.print("Distance: ");
//    Serial.println(distance);
//  }
//  return distance;
//}*/
