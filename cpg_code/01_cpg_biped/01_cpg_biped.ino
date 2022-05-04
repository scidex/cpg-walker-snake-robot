
#include <AX12A.h> // Header file for the DYNAMIXEL servo motor librabry 
#include "BluetoothSerial.h" //Header file for Serial Bluetooth, will be added by default into Arduino
BluetoothSerial ESP_BT; //Object for Bluetooth

/*
Define your robot Name as written in the header file: "the_six_heros.h"

The Six Heroes: Trinity, Valkenhayn, Konoe, Mitsuyoshi, Hakumen, and Yuuki.
*/

// The following two neuron system is used for the cpg.
#define NUMBER_RS_NEURONS 2

//2 neuron system 

/******************************************************/ 
//struct RSneuron 
/******************************************************/ 
struct RSneuron { 
  float x = 0;       // membrane potential
  float x_old = 0;
  float y = 0;       // output response of neuron
  //float tau = 1;   // time const.
  int s = 50;        // impulse rate
  float b = 2.5;     // adaptation coefficient b = 0, 2.5, inf
  float x_prime = 0; // degree of adaptation
  float x_prime_old = 0;
  float y_old = 0;
  float T = 12;      // time constant for adaptation T = 2.5, 12, inf
  float theta = 0;   // threshold
} rs_neuron[NUMBER_RS_NEURONS];

float time_step = 0.1;

float tau = 1;

int i = 0;
int j = 0;
float net_input = 0;


#define TRINITY
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


int mySpeed = 110;
int hipstep = 80;
int torsoStep = 130;
int myd = 4;
int myd2 = 1000;//ms
int myd3 = 20;
int hight_factor = 2.7;

int maxo[7]; /* maximum values for each motro */
int mino[7]; /* minimum values for each motro */
int pos[7]; /* Position values for each motor */

int delta_phi = 3;

// LA: Left ankle
// 

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

  Serial.print(LFSRFront);  Serial.print("\t");//ESP_BT.print(LFSRFront);  ESP_BT.print("\t"); 
  Serial.print(LFSRRear);   Serial.print("\t");//ESP_BT.print(LFSRRear);   ESP_BT.print("\t");
  Serial.print(RFSRFront);  Serial.print("\t");//ESP_BT.print(RFSRFront);  ESP_BT.print("\t");
  Serial.print(RFSRRear);   Serial.print("\n");//ESP_BT.print(RFSRRear);   ESP_BT.print("\n");

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
  // Here the cpg output is generated. The output of both neuron instances with are
  // temporally shifted are used for controlling the torso motor later in the code.
  for(i = 0; i < NUMBER_RS_NEURONS; i ++){
    
   if (i < NUMBER_RS_NEURONS/2 ){  // lest side neurons update
    
    if(i == 0){
    
        net_input = net_input + 1.5 * rs_neuron[i + NUMBER_RS_NEURONS/2].y + 1.5 * rs_neuron[i + 1].y;
        rs_neuron[i].x = (1 - time_step/tau)*rs_neuron[i].x_old + time_step * (-net_input + 1.0  - (rs_neuron[i].b * rs_neuron[i].x_prime_old))/tau;
        rs_neuron[i].x_prime = (1 - time_step/rs_neuron[i].T)*rs_neuron[i].x_prime_old + time_step * rs_neuron[i].y / rs_neuron[i].T;
        rs_neuron[i].y = max(0.0f, rs_neuron[i].x_old - rs_neuron[i].theta);
        rs_neuron[i].x_old = rs_neuron[i].x;
        rs_neuron[i].x_prime_old = rs_neuron[i].x_prime;
        net_input = 0;
           
      }  
    else if(i!=0){ 
        net_input = net_input + 1.5 * rs_neuron[i-1].y + 1.5 * rs_neuron[i + NUMBER_RS_NEURONS/2].y + 1.5 * rs_neuron[i + 1].y;

        rs_neuron[i].x = (1 - time_step/tau)*rs_neuron[i].x_old + time_step * (-net_input + 1.0  - (rs_neuron[i].b * rs_neuron[i].x_prime_old))/tau;
        rs_neuron[i].x_prime = (1 - time_step/rs_neuron[i].T)*rs_neuron[i].x_prime_old + time_step * rs_neuron[i].y / rs_neuron[i].T;
        rs_neuron[i].y = max(0.0f, rs_neuron[i].x_old - rs_neuron[i].theta);
        rs_neuron[i].x_old = rs_neuron[i].x;
        rs_neuron[i].x_prime_old = rs_neuron[i].x_prime;
        net_input = 0;
      }
   }
    
  else if(i >= NUMBER_RS_NEURONS/2){  //right side neurons update
    if(i == NUMBER_RS_NEURONS/2){
    
        net_input = net_input + 1.5 * rs_neuron[i - NUMBER_RS_NEURONS/2].y + 1.5 * rs_neuron[i + 1].y;
          
        rs_neuron[i].x = (1 - time_step/tau)*rs_neuron[i].x_old + time_step * (-net_input + 1.0  - (rs_neuron[i].b * rs_neuron[i].x_prime_old))/tau;
        rs_neuron[i].x_prime = (1 - time_step/rs_neuron[i].T)*rs_neuron[i].x_prime_old + time_step * rs_neuron[i].y / rs_neuron[i].T;
        rs_neuron[i].y = max(0.0f, rs_neuron[i].x_old - rs_neuron[i].theta);
        rs_neuron[i].x_old = rs_neuron[i].x;
        rs_neuron[i].x_prime_old = rs_neuron[i].x_prime;
        net_input = 0;     
      
      }  
    else if(i!=NUMBER_RS_NEURONS/2){ 
        net_input = net_input + 1.5 * rs_neuron[i-1].y + 1.5 * rs_neuron[i - NUMBER_RS_NEURONS/2].y + 1.5 * rs_neuron[i + 1].y;
                      
        rs_neuron[i].x = (1 - time_step/tau)*rs_neuron[i].x_old + time_step * (-net_input + 1.0  - (rs_neuron[i].b * rs_neuron[i].x_prime_old))/tau;
        rs_neuron[i].x_prime = (1 - time_step/rs_neuron[i].T)*rs_neuron[i].x_prime_old + time_step * rs_neuron[i].y / rs_neuron[i].T;
        rs_neuron[i].y = max(0.0f, rs_neuron[i].x_old - rs_neuron[i].theta);
        rs_neuron[i].x_old = rs_neuron[i].x;
        rs_neuron[i].x_prime_old = rs_neuron[i].x_prime;
        net_input = 0;
    }
      
   }
 }
   
  
  // Only the torso movements are important here. Hence, the control for all other motors was removed.
  Serial.print("Neuron0:"); Serial.print(rs_neuron[0].y); Serial.print("  ");
  Serial.print("Neuron1:"); Serial.print(rs_neuron[1].y); Serial.print("  "); Serial.println("  ");

  delay(100);

  // Torso oscillation definition:
  // The updated output of neuron 2 is subtracted from the cpg output of neuron 1. This leads to a full movement range both
  // to the left and right side. If only one neuron was used, the robot could only bend the torso motor to one side.
  // For a faster movement, the speed could be increased.
  ax12a.moveSpeed(TO, initial_pos[TO]-(torsoStep*(rs_neuron[0].y-rs_neuron[1].y)),mySpeed);


}
