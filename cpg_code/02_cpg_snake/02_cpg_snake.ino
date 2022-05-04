

#include <AX12A.h>
#include "BluetoothSerial.h" //Header File for Serial Bluetooth, will be added by default into Arduino

BluetoothSerial ESP_BT; //Object for Bluetooth

#define PI 3.1415926535897932384626433832795
#define PI_ 3.1415
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105

#define DirectionPin   (10u)
#define BaudRate      (1000000ul)

#define NUMBER_RS_NEURONS 6

//20 neuron system (As represnted in Ijspeert et al paper, we select only 6 neurons to operate 3 motors) 

/******************************************************/ 
//struct RSneuron 
/******************************************************/ 
struct RSneuron { 
  float x = 0;       // membrane potential
  float x_old = 0;
  float y = 0;       // output response of neuron
  //float tau = 1;     // time const.
  int s = 50;     // impulse rate
  float b = 2.5;       // adaptation coefficient b = 0, 2.5, inf
  float x_prime = 0; // degree of adaptation
  float x_prime_old = 0;
  float y_old = 0;
  float T = 12;       // time constant for adaptation T = 2.5, 12, inf
  float theta = 0;   // threshold
} rs_neuron[NUMBER_RS_NEURONS];

float time_step = 0.1;

float tau = 1;

/******************************************************/ 

int i = 0;
int j = 0;
float net_input = 0;

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
delay(1000);// default: 2000

for(int i=0;i<=10;i++)  
ax12a.moveSpeed(i+1, initial_pos,100); 
delay(1000);//default : 2000


}

void loop()
{

 // CPG________________________________________________________________
  
  /***************************
 *  CPG 6 Neurons implementation (Differntial equations and Euler's method solver)
 *  ************************
*/


 for(i = 0; i < NUMBER_RS_NEURONS; i ++){
    
   if (i < NUMBER_RS_NEURONS/2 ){  // left side neurons update
    
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
   
  delay(10);
  
  Serial.print("Neuron0:"); Serial.print(rs_neuron[0].y); Serial.print("  ");
  Serial.print("Neuron1:"); Serial.print(rs_neuron[1].y); Serial.print("  ");
  Serial.print("Neuron2:"); Serial.print(rs_neuron[2].y); Serial.print("  ");
  Serial.print("Neuron3:"); Serial.print(rs_neuron[3].y); Serial.print("  ");
  Serial.print("Neuron4:"); Serial.print(rs_neuron[4].y); Serial.print("  ");
  Serial.print("Neuron5:"); Serial.print(rs_neuron[5].y); Serial.print("  "); Serial.println("  ");

 
 // CPG END____________________________________________________________

  
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
 * Here you can choose the targeted neurons from (first controlled motor) number.
 The following part assigns each targeted motor with combination of two neurons
 Excluding other motors to a constant intial position
 
*/
  
  int first_coontrolled_motor = 5;
  for(int i=first_coontrolled_motor; i<=8; i++){
  if (i<5){
    pos[i] = initial_pos;
  } 
  else {
  
  //pos[i] = initial_pos+ampl*i*0.5*sin( 1*phi*DEG_TO_RAD + i*PI_*0.3 );
  pos[i] = initial_pos + 3 * ampl*(rs_neuron[i-first_coontrolled_motor].y -rs_neuron[i + NUMBER_RS_NEURONS/2-first_coontrolled_motor].y);
  
  }
  }
  

/*
 * Control motors 
 Note: We only contorl the (Last or first) three motors 
*/
  for(int i=first_coontrolled_motor; i<=8; i++){
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
  // Serial.print(flexsens[i]);    Serial.print("\t");
  }
  //ESP_BT.print("\n");
  // Serial.print("\n");

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
