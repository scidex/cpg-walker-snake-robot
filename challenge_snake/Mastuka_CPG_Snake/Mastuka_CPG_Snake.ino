

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

#define NUMBER_RS_NEURONS 20

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

float tau = 0.5;

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
float pos[10]={0};
float pos_sec[10] = {0};
float pos_sum[10] = {0};

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
const double input_outnet = 3.0;
double net_intef = 0.0;
double net_mutpar = 1.0;



/* the seadings of 20 flex sensors */
int flexsens[20];
int flexsens_store[100][20];
int count = 0;
float flexsens_base[20];
float sum = 0;


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
for(int i=0; i<=10; i++){
    ax12a.move(i, initial_pos); 
    delay(2);
  }


}

void loop()
{

 // CPG________________________________________________________________
  
  /***************************
 *  CPG 6 Neurons implementation (Differntial equations and Euler's method solver)
 *  ************************
*/





   //mode 2: a1 = a3 = 1.5, a2= 0

   for(int i = 0; i < NUMBER_RS_NEURONS ; i++){
    for (int j = 0; j < NUMBER_RS_NEURONS; j++){
      if((j == i-1)||((i == 0)&&(j == NUMBER_RS_NEURONS -1 ))||(i == j)){
        continue;
        }
        else{
          net_intef = net_intef + net_mutpar * rs_neuron[j].y;
        }
    }

    rs_neuron[i].x = (1 - time_step)*rs_neuron[i].x_old + time_step * (-net_intef   + input_outnet - (rs_neuron[i].b * rs_neuron[i].x_prime_old));
    rs_neuron[i].x_prime = (1 - time_step/rs_neuron[i].T)*rs_neuron[i].x_prime_old + time_step * rs_neuron[i].y / rs_neuron[i].T;
    rs_neuron[i].y = max(0.0f, rs_neuron[i].x_old - rs_neuron[i].theta);
    net_intef = 0.0;
   }


    


  
//  Serial.print("Neuron0:"); Serial.print(rs_neuron[0].y_old - rs_neuron[3].y_old); Serial.print("  ");
//  Serial.print("Neuron1:"); Serial.print(rs_neuron[1].y_old - rs_neuron[4].y_old); Serial.print("  ");
//  Serial.print("Neuron2:"); Serial.print(rs_neuron[2].y_old - rs_neuron[5].y_old); Serial.print("  ");
//  Serial.print("Neuron3:"); Serial.print(rs_neuron[3].y_old - rs_neuron[6].y_old); Serial.print("  ");
//  Serial.print("Neuron4:"); Serial.print(rs_neuron[4].y_old - rs_neuron[7].y_old); Serial.print("  ");
//  Serial.print("Neuron5:"); Serial.print(rs_neuron[5].y_old - rs_neuron[8].y_old); Serial.print("  "); 
//  Serial.print("Neuron6:"); Serial.print(rs_neuron[6].y_old - rs_neuron[9].y_old); Serial.print("  ");
//  Serial.print("Neuron7:"); Serial.print(rs_neuron[7].y_old- rs_neuron[10].y_old); Serial.print("  ");
//  Serial.print("Neuron8:"); Serial.print(rs_neuron[8].y_old - rs_neuron[11].y_old); Serial.print("  ");
//  Serial.print("Neuron9:"); Serial.print(rs_neuron[9].y_old - rs_neuron[12].y_old); Serial.print("  ");
//  Serial.print("Neuron10:"); Serial.print(rs_neuron[10].y_old - rs_neuron[13].y_old); Serial.print("  ");
//  Serial.print("Neuron11:"); Serial.print(rs_neuron[11].y_old - rs_neuron[14].y_old); Serial.print("  ");
//  Serial.print("Neuron12:"); Serial.print(rs_neuron[12].y_old - rs_neuron[15].y_old); Serial.print("  ");
//  Serial.print("Neuron13:"); Serial.print(rs_neuron[13].y_old - rs_neuron[16].y_old); Serial.print("  "); 
//  Serial.print("Neuron14:"); Serial.print(rs_neuron[14].y_old - rs_neuron[17].y_old); Serial.print("  ");
//  Serial.print("Neuron15:"); Serial.print(rs_neuron[15].y_old - rs_neuron[18].y_old); Serial.print("  ");
//  Serial.print("Neuron16:"); Serial.print(rs_neuron[16].y_old - rs_neuron[19].y_old); Serial.print("  ");
//  Serial.print("Neuron17:"); Serial.print(rs_neuron[17].y_old - rs_neuron[0].y); Serial.print("  ");
//  Serial.print("Neuron18:"); Serial.print(rs_neuron[18].y_old - rs_neuron[1].y); Serial.print("  "); 
//  Serial.print("Neuron19:"); Serial.print(rs_neuron[19].y_old - rs_neuron[2].y); Serial.print("  ");
//          
//
//  Serial.println("  ");

 
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
  
  int first_coontrolled_motor = 0;

  for(int i = 0; i<10; i++)
  {
    pos[i] = initial_pos +  ampl *(rs_neuron[i].y_old - rs_neuron[i + 5].y_old);

  }

   for(int i = 10; i<20; i++)
  {
   if (i > 14 ){
    pos_sec[i - 10] = initial_pos + ampl * (rs_neuron[i].y_old - rs_neuron[i+5-20].y);
    }
    else{
    pos_sec[i-10] = initial_pos + ampl*(rs_neuron[i].y_old - rs_neuron[i + 5].y_old);
      }

   }

  for(int j=0; j< 10; j++){
      pos_sum[j] = (pos[j] + pos_sec[j])/2;
      ax12a.move(j, pos_sum[j]); 
      delay(1);
  }
/*
      Serial.print("Neuron0:"); Serial.print(pos_sum[1]); Serial.print("  ");
      Serial.print("Neuron1:"); Serial.print(pos_sum[1]); Serial.print("  ");
      Serial.print("Neuron2:"); Serial.print(pos_sum[2]); Serial.print("  ");
      Serial.print("Neuron3:"); Serial.print(pos_sum[3]); Serial.print("  ");
      Serial.print("Neuron4:"); Serial.print(pos_sum[4]); Serial.print("  ");
      Serial.print("Neuron5:"); Serial.print(pos_sum[5]); Serial.print("  "); 
      Serial.print("Neuron6:"); Serial.print(pos_sum[6]); Serial.print("  ");
      Serial.print("Neuron7:"); Serial.print(pos_sum[7]); Serial.print("  ");
      Serial.print("Neuron8:"); Serial.print(pos_sum[8]); Serial.print("  ");
      Serial.print("Neuron9:"); Serial.print(pos_sum[9]); Serial.println("  "); 
*/
/*
 * Control motors 
 Note: We only contorl the (Last or first) three motors 
*/


   for(int i = 0; i < NUMBER_RS_NEURONS ; i++){
    rs_neuron[i].x_old = rs_neuron[i].x;
    
    rs_neuron[i].x_prime_old = rs_neuron[i].x_prime;

    rs_neuron[i].y_old = rs_neuron[i].y;
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
// 
//      ESP_BT.print(flexsens[0]);
//   Serial.print("sensor0:");Serial.print(flexsens[0]); Serial.print(" ");
//      ESP_BT.print(flexsens[1]); ESP_BT.print(" ");
//   Serial.print("sensor1:");Serial.print(flexsens[1]); Serial.print(" ");
//      ESP_BT.print(flexsens[2]); ESP_BT.print(" ");
//   Serial.print("sensor2:");Serial.print(flexsens[2]); Serial.print(" ");
//      ESP_BT.print(flexsens[3]); ESP_BT.print(" ");
//   Serial.print("sensor3:");Serial.print(flexsens[3]); Serial.print(" ");
//      ESP_BT.print(flexsens[4]); ESP_BT.print(" ");
//   Serial.print("sensor4:");Serial.print(flexsens[4]); Serial.print(" ");
//      ESP_BT.print(flexsens[5]); ESP_BT.print(" ");
//   Serial.print("sensor5:");Serial.print(flexsens[5]); Serial.print(" ");
//      ESP_BT.print(flexsens[6]); ESP_BT.print(" ");
//   Serial.print("sensor6:");Serial.print(flexsens[6]); Serial.print(" ");
//      ESP_BT.print(flexsens[7]); ESP_BT.print(" ");
//   Serial.print("sensor7:");Serial.print(flexsens[7]); Serial.print(" ");
//      ESP_BT.print(flexsens[8]); ESP_BT.print(" ");
//   Serial.print("sensor8:");Serial.print(flexsens[8]); Serial.print(" ");
//      ESP_BT.print(flexsens[9]); ESP_BT.print(" ");
//   Serial.print("sensor9:");Serial.print(flexsens[9]); Serial.print(" ");
//      ESP_BT.print(flexsens[10]); ESP_BT.print(" ");
//   Serial.print("sensor10:");Serial.print(flexsens[10]); Serial.print(" ");
//      ESP_BT.print(flexsens[11]); ESP_BT.print("\t");
//   Serial.print("sensor11:");Serial.print(flexsens[11]); Serial.print(" ");
//      ESP_BT.print(flexsens[12]); ESP_BT.print(" ");
//   Serial.print("sensor12:");Serial.print(flexsens[12]); Serial.print(" ");
//      ESP_BT.print(flexsens[13]); ESP_BT.print(" ");
//   Serial.print("sensor13:");Serial.print(flexsens[13]); Serial.print(" ");
//      ESP_BT.print(flexsens[14]); ESP_BT.print(" ");
//   Serial.print("sensor14:");Serial.print(flexsens[14]); Serial.print(" ");
//      ESP_BT.print(flexsens[15]); ESP_BT.print(" ");
//   Serial.print("sensor15:");Serial.print(flexsens[15]); Serial.print(" ");
//      ESP_BT.print(flexsens[16]); ESP_BT.print(" ");
//   Serial.print("sensor16:");Serial.print(flexsens[16]); Serial.print(" ");
//      ESP_BT.print(flexsens[17]); ESP_BT.print(" ");
//   Serial.print("sensor17:");Serial.print(flexsens[17]); Serial.print(" ");
//      ESP_BT.print(flexsens[18]); ESP_BT.print(" ");
//   Serial.print("sensor18:");Serial.print(flexsens[18]); Serial.print(" ");
//      ESP_BT.print(flexsens[19]); ESP_BT.print(" ");
//   Serial.print("sensor19:");Serial.print(flexsens[19]); Serial.println(" ");

   
   float mytime = millis();

   for (int j=0;j<20;j++){
    if (count<100){
    flexsens_store[count][j] = flexsens[j];
    }
   }
   count++;

   if(count == 100)
   {
    for (int j=0; j<20;j++){
      for (int k=0;k<100;k++)
      {
        sum = sum + flexsens_store[k][j];
        }
      flexsens_base[j] = sum / 100;
      sum = 0;
    }
    }

    for (int i = 0; i<20; i++){
      if (flexsens[i] - flexsens_base[i]> 200)
      {
        Serial.print(i);
        }
      
    }

   
   

   
  
  
  ESP_BT.print("\n");
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
