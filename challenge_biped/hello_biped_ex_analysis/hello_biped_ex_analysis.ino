
#include <AX12A.h> // Header file for the DYNAMIXEL servo motor librabry 
#include "BluetoothSerial.h" //Header file for Serial Bluetooth, will be added by default into Arduino
BluetoothSerial ESP_BT; //Object for Bluetooth

/*
Define your robot Name as written in the header file: "the_six_heros.h"

The Six Heroes: Trinity, Valkenhayn, Konoe, Mitsuyoshi, Hakumen, and Yuuki.
*/

#define VALKENHAYN
#include "the_six_heros.h"

#define PI 3.1415926535897932384626433832795
#define PI_ 3.1415
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105

#define DirectionPin   (7u)
#define BaudRate      (1000000ul)

/*
 * Set of parameters to be used in the step by step walking
*/
int mySpeed = 100; /* from 0 to 255. 255 is full motor speed*/
int hipstep = 50; /* */
int torsoStep = 120; /* */
int myd = 5; //ms /* a delay between to commands to motors, it should be as small as possible */
int myd2 = 1000;//ms 
int myd3 = 20; /* a delay between to sensors readings sycles */
int hight_factor = 2; /* a multiplication factor for the hight of the feet in swing phase*/

int maxo[7]; /* maximum values for each motro */
int mino[7]; /* minimum values for each motro */
int pos[7]; /* Position values for each motor */

int delta_phi = 3;

/*
 * This function read foot force sensors.
*/
void readSensors()
{
  LFSRFront = analogRead(LFSRFrontPin);
  LFSRRear = analogRead(LFSRRearPin);
  RFSRFront = analogRead(RFSRFrontPin);
  RFSRRear = analogRead(RFSRRearPin);
  
}

/*
 * This function read foot force sensors and print them during the delay.
 * t: is the required delay.
 * tb: is the sensors reading time base. 
*/
void mydelay(unsigned long int t,unsigned int tb=1)
{
  long int ts= millis();
  while((millis()-ts)<t)
  {
  
  readSensors();

  /*
  Serial.print(LFSRFront);  Serial.print("\t");//ESP_BT.print(LFSRFront);  ESP_BT.print("\t"); 
  Serial.print(LFSRRear);   Serial.print("\t");//ESP_BT.print(LFSRRear);   ESP_BT.print("\t");
  Serial.print(RFSRFront);  Serial.print("\t");//ESP_BT.print(RFSRFront);  ESP_BT.print("\t");
  Serial.print(RFSRRear);   Serial.print("\n");//ESP_BT.print(RFSRRear);   ESP_BT.print("\n");
  */

  delay(tb);
  }
}


void setup()
{

for(int i = 0;i<7;i++)
{
  maxo[i]=initial_pos[i]+range[i]/2.0;
  mino[i]=initial_pos[i]-range[i]/2.0;
}

/*
 * Setup a connection to serila port for motor control, please only use "Serial2" to connect to motors
 * we keep using "Serial" for the debugging through the "Serial monitor" 
*/
ax12a.begin(BaudRate, DirectionPin, &Serial2);


/*
 * Each motor has an LED that you can use it as a display.
 * Here, we turn them ON for demonstration.
*/

for(int ID = 0;ID<7;ID++)
{
ax12a.ledStatus(ID, ON);
delay(5);
}

Serial.begin(115200);

/*
 * if you are interested in streaming data on bluetooth, you can use "ESP_BT.print(my_data)"
*/
ESP_BT.begin(BluetoothDeviceName); //Bluetooth device name

mydelay(1000,myd3);

/*
 * move to initial positions as defined in the "the _six_heros.h"
*/
for(int i=0;i<7;i++)  
{ax12a.moveSpeed(i, initial_pos[i],100); delay(2);}
mydelay(5000,myd3);


/*
 * preparation to walk.
 * Here we show an example for step by step walking. initial part is in "setup()", the main code is in "loop()"
 * First, balance to left side by moving the torso "TO" wiht the speed "mySpeed"
*/
ax12a.moveSpeed(TO, initial_pos[TO]+torsoStep,mySpeed); mydelay(myd,myd3);
mydelay(1000,myd3);

}

void loop()
{


// right leg up 
ax12a.moveSpeed(RH, initial_pos[RH]+hipstep,mySpeed/2); mydelay(myd,myd3);
ax12a.moveSpeed(RK, initial_pos[RK]-hight_factor*hipstep,mySpeed); mydelay(myd,myd3);
ax12a.moveSpeed(RA, initial_pos[RA]+hight_factor*0.75*hipstep,mySpeed); mydelay(myd,myd3);
mydelay(myd2,myd3);


// move stance leg 
ax12a.moveSpeed(LH, initial_pos[LH]+hipstep/2,mySpeed/2); mydelay(myd,myd3);
ax12a.moveSpeed(LA, initial_pos[LA]-hipstep/2,mySpeed/2); mydelay(myd,myd3);
//delay(myd2);

// move swing leg to front
ax12a.moveSpeed(RH, initial_pos[RH]+hipstep/2,mySpeed/4); mydelay(myd,myd3);
ax12a.moveSpeed(RK, initial_pos[RK],mySpeed); mydelay(myd,myd3);
ax12a.moveSpeed(RA, initial_pos[RA]-hipstep/2,mySpeed); mydelay(myd,myd3);
mydelay(myd2,myd3);

// balance to right side 
ax12a.moveSpeed(TO, initial_pos[TO]-torsoStep,mySpeed); mydelay(myd,myd3);
mydelay(myd2,myd3);

// move swing leg to stance
ax12a.moveSpeed(RK, initial_pos[RK],mySpeed); mydelay(myd,myd3);
ax12a.moveSpeed(RA, initial_pos[RA],mySpeed); mydelay(myd,myd3);
ax12a.moveSpeed(RH, initial_pos[RH],mySpeed/2); mydelay(myd,myd3);
mydelay(myd2,myd3);

// left leg up 
ax12a.moveSpeed(LH, initial_pos[LH]-hipstep,mySpeed/2); mydelay(myd,myd3);
ax12a.moveSpeed(LK, initial_pos[LK]+hight_factor*hipstep,mySpeed); mydelay(myd,myd3);
ax12a.moveSpeed(LA, initial_pos[LA]-hight_factor*0.75*hipstep,mySpeed); mydelay(myd,myd3);
mydelay(myd2,myd3);

// move stance leg 
ax12a.moveSpeed(RH, initial_pos[RH]-hipstep/2,mySpeed/2); mydelay(myd,myd3);
ax12a.moveSpeed(RA, initial_pos[RA]+hipstep/2,mySpeed/2); mydelay(myd,myd3);
//delay(myd2);
 
// move swing leg to front
ax12a.moveSpeed(LH, initial_pos[LH]-hipstep/2,mySpeed/4); mydelay(myd,myd3);
ax12a.moveSpeed(LK, initial_pos[LK],mySpeed); mydelay(myd,myd3);
ax12a.moveSpeed(LA, initial_pos[LA]+hipstep/2,mySpeed); mydelay(myd,myd3);
mydelay(myd2,myd3);

// balance to left side 
ax12a.moveSpeed(TO, initial_pos[TO]+torsoStep,mySpeed); mydelay(myd,myd3);
mydelay(myd2,myd3);

// move swing leg to stance
ax12a.moveSpeed(LK, initial_pos[LK],mySpeed); mydelay(myd,myd3);
ax12a.moveSpeed(LA, initial_pos[LA],mySpeed); mydelay(myd,myd3);
ax12a.moveSpeed(LH, initial_pos[LH],mySpeed/2); mydelay(myd,myd3);
mydelay(myd2,myd3);

}
