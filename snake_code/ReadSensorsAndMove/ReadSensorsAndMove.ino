

#include <AX12A.h>
#include "BluetoothSerial.h" //Header File for Serial Bluetooth, will be added by default into Arduino

BluetoothSerial ESP_BT; //Object for Bluetooth

/* Define some constants (pi, convertion between deg to rad)*/
#define PI 3.1415926535897932384626433832795
#define PI_ 3.1415
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105

#define DirectionPin   (10u)
#define BaudRate      (1000000ul)

/* initial position of motor*/
int initial_pos = 512;

/* set max and min values for the motors */
int maxo = initial_pos + 70;
int mino = initial_pos - 70;

/* positions of motors */
int pos[10];

/* an amplitude of a sine wave */
int ampl=60;

/* position step to control motors, smaller is more acurate */
int delta = 5;

/* an angle for a sin wave */
int phi = 0 ; // [deg]

/***************************
 *  analog multiplexer pins
 *  ************************
*/

/* pins 34 and 35 are for the two analog multiplexer*/
const int potPin = 34;
const int potPinB = 35;

/* Analog Channels selection pins for the two analog multiplexer*/
const int channel[] = {2,4,5,18};
const int channelB[] = {21,22,23,32};

/* Enable pins for the two analog multiplexer*/
const int outputPin = 15;
const int outputPinB = 19;

/* the seadings of 20 flex sensors */
int flexsens[20];


/*
 * initilaization code 
*/
void setup()
{

ax12a.begin(BaudRate, DirectionPin, &Serial2);


Serial.begin(115200);
ESP_BT.begin("SnakeStreaming"); //Bluetooth device name
delay(1000);

for (int i=0;i<4;i++)
pinMode(channel[i], OUTPUT);
pinMode(outputPin, OUTPUT);

for (int i=0;i<4;i++)
pinMode(channelB[i], OUTPUT);
pinMode(outputPinB, OUTPUT);

digitalWrite(outputPin,HIGH);
digitalWrite(outputPinB,HIGH);

for(int i=0;i<=10;i++)  
ax12a.moveSpeed(i+1, initial_pos,100); 
delay(2000);

for(int i=0;i<=10;i++)  
ax12a.moveSpeed(i+1, initial_pos,100); 
delay(2000);


}

void loop()
{
/* 
 * Read from multiplexers 
 * place the readings in "flexsens"
 */

  for (int muxChannel = 0; muxChannel < 16; muxChannel++)
  {
  muxWrite(muxChannel);
  digitalWrite(outputPin,LOW);
  flexsens[muxChannel] = analogRead(potPin);
  digitalWrite(outputPin,HIGH);
  }

  for (int muxChannel = 0; muxChannel < 4; muxChannel++)
  {
  muxBWrite(muxChannel);
  digitalWrite(outputPinB,LOW);
  flexsens[16+muxChannel] = analogRead(potPinB);
  digitalWrite(outputPinB,HIGH);
  }
  
/*
 * Calculate the position for each motor 
*/

/*
 * In this part, the phase is varying by i in each motor to have the motion effect. 
 * As well As, the amplitude is changing with a factor i which represent the number of the motor.
 * The Amplitude is increasing as we go further from the head (i=0,1,2 .. from head to tail) 
*/
  for(int i=0;i<=10;i++)
  pos[i] = initial_pos+ampl*i*0.5*sin( -1*phi*DEG_TO_RAD + i*PI_*0.3 );

/*
 * Control motors 
*/
  for(int i=0;i<=10;i++)  
  {
    ax12a.move(i+1, pos[i]); 
    delay(1);
  }

  
/*
 * Increas the phase "phi" wiht a step "delta"
*/
  phi =phi+delta;

  if(phi==360)
   phi=0;
  delay(10);
  
  /*
   * Sending sensors reading to serial port. 
  */
  for (int i = 0; i < 20; i++)
  {
  //ESP_BT.print(flexsens[i]);    ESP_BT.print("\t");
  Serial.print(flexsens[i]);    Serial.print("\t");
  }
  //ESP_BT.print("\n");
  Serial.print("\n");

} // End of main loop 


/*
 * Reading from the first analog multiplexer
*/

void muxWrite(int whichChannel)
{
for (int inputPin = 0; inputPin < 4; inputPin++)
{
int pinState = bitRead(whichChannel, inputPin);
digitalWrite(channel[inputPin],pinState);
}
}

/*
 * Reading from the analog multiplexer
*/
void muxBWrite(int whichChannel)
{
for (int inputPin = 0; inputPin < 4; inputPin++)
{
int pinState = bitRead(whichChannel, inputPin);
digitalWrite(channelB[inputPin],pinState);
}
}
