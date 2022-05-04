
#include <AX12A.h> // Header file for the DYNAMIXEL servo motor librabry 
#include "BluetoothSerial.h" //Header file for Serial Bluetooth, will be added by default into Arduino
BluetoothSerial ESP_BT; //Object for Bluetooth

/*
Define your robot Name as written in the header file: "the_six_heros.h"

The Six Heroes: Trinity, Valkenhayn, Konoe, Mitsuyoshi, Hakumen, and Yuuki.
*/

// The following two neuron system is used for the cpg.
#define NUMBER_RS_NEURONS 16

/******************************************************/ 
//struct RSneuron 
/******************************************************/
struct RSneuron { 
  float x = 0;       // membrane potential
  float x_old = 0;   // old membrane potential --> for calculating the new one
  float y = 0;       // output response of neuron
  //float tau = 1;   // time const.
  int s = 0.0;       // impulse rate | outside network stimulation (we can set it) // Increase: amplitude increase
  float b = 2.5;     // adaptation coefficient b = 0, 2.5, inf // increase: decreases amplitude, increases frequ.
  /*
   * Adaptation change over time
   */
  float x_prime = 0; // degree of adaptation / rate of change // No interesting behavior changes
  float x_prime_old = 0;
  float y_old = 0;
  float T = 100;      // prev. 12 time constant for adaptation T = 2.5, 12, inf // time period / 1/f
  float threshold = 0.0;   // threshold // derivative at start is lower
} rs_neuron[NUMBER_RS_NEURONS];

  /*
   * Secondly, adaptation is incorporated into them (as a special case, 
     it  includes  the  case  of  no  adaptation).  That  is,  the 
     neuron  has  a  property  such  that  the  firing  rate 
     decreases when given a constant stimulus energy. Our 
     analysis  will  show that  adaptation  has  an  important 
     role  on  the  appearance  of the  oscillation.
     --> amplitude of the adaptation
   */


 // We will need 14 neurons (2sides * 7motors/joints)
 // N*N connection matrix
 int connection_mat[NUMBER_RS_NEURONS][NUMBER_RS_NEURONS] = {0};

 // connection_mat[0][0] = 0;

float time_step = 0.1;
float tau = 1; //same effect as tau

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


int mySpeed = 100;
int hipstep = 50;
int torsoStep = 120;
int myd = 5;
int myd2 = 1000;//ms
int myd3 = 20;
int hight_factor = 2;

int maxo[7]; /* maximum values for each motro */
int mino[7]; /* minimum values for each motro */
int pos[7]; /* Position values for each motor */

int delta_phi = 3;

float TO_current = 0;
float LH_current = 0;
float RH_current = 0;
float LK_current = 0;
float RK_current = 0;
float LA_current = 0;
float RA_current = 0;

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
  /*
  Serial.print(LFSRFront);  Serial.print("\t");//ESP_BT.print(LFSRFront);  ESP_BT.print("\t"); 
  Serial.print(LFSRRear);   Serial.print("\t");//ESP_BT.print(LFSRRear);   ESP_BT.print("\t");
  Serial.print(RFSRFront);  Serial.print("\t");//ESP_BT.print(RFSRFront);  ESP_BT.print("\t");
  Serial.print(RFSRRear);   Serial.print("\n");//ESP_BT.print(RFSRRear);   ESP_BT.print("\n");
  */

  delay(tb);
  }
}

void print_motors()
{
  Serial.printf("TO:"); Serial.print(TO_current); Serial.print("  ");
  Serial.printf("LH:"); Serial.print(LH_current); Serial.print("  ");
  Serial.printf("RH:"); Serial.print(RH_current); Serial.print("  ");
  Serial.printf("LK:"); Serial.print(LK_current); Serial.print("  ");
  Serial.printf("RK:"); Serial.print(RK_current); Serial.print("  ");
  Serial.printf("LA:"); Serial.print(LA_current); Serial.print("  ");
  Serial.printf("RA:"); Serial.print(RA_current); Serial.print("  ");
  Serial.println("  ");
}

float setup_end = 0;
void setup(){
  Serial.begin(115200);


  // NEURON INITIALIZATION
  // Torso neurons: 0, 1 | TO_dt: 3220 instead of 3160
  rs_neuron[0].T = 28;
  rs_neuron[0].s = 1.0;
  rs_neuron[1].T = 28;
  rs_neuron[1].s = 1.0;

  // LK neuron | LK_dt: 
  rs_neuron[2].b = 1.0;
  rs_neuron[2].threshold = 0.75;

  // LH neuron | LH_dt: 
  rs_neuron[3].b = 5.0;

  // RK neuron | RK_dt: 
  rs_neuron[4].b = 1.0;
  rs_neuron[4].threshold = 0.75;

  // RH neuron | RH_dt: 
  rs_neuron[5].b = 5.0;

  // Recurrent connection RH
  rs_neuron[6].b = 0.5;
  //rs_neuron[6].threshold = 0.7;

  // Hip forward neuron
  rs_neuron[7].b = 0.5;
  rs_neuron[7].threshold = 9; // 39.5; //38-39.7 // 7: still too big curve
  rs_neuron[7].T = 100;
  //rs_neuron[7].s = -1;

  // Hip output neuron
  rs_neuron[8].b = 0.00000002;
  rs_neuron[8].threshold = 0.0; // 39.5; //38-39.7 // 7: still too big curve
  rs_neuron[8].T = 1000;
  //rs_neuron[7].s = -1;


  rs_neuron[9].b = 1.0;
  rs_neuron[9].s = 1.0;
  rs_neuron[10].b = 1.0;
  rs_neuron[10].s = 1.0;
  rs_neuron[11].b = 1.0;
  rs_neuron[11].s = 1.0;
  rs_neuron[12].b = 1.0;
  rs_neuron[12].s = 1.0;

  rs_neuron[13].b = 0.7;
  rs_neuron[13].s = 1.0;

  rs_neuron[14].T = 28;
  rs_neuron[14].s = 1.0;
  rs_neuron[15].T = 28;
  rs_neuron[15].s = 1.0;

  // CONNECTION INITIALIZATION
  // Connect both TO neurons:
  connection_mat[0][1] = -2.5;
  connection_mat[1][0] = -2.5;
  
  // Connect LK to TO:
  connection_mat[2][10] = 1.2;
  connection_mat[2][2] = 0.9;
  

  // Connect LH to TO:
  connection_mat[3][0] = 2.5;

  // Connect RK to TO:
  // connection_mat[4][1] = 2.5;
  connection_mat[4][12] = 1.2;
  connection_mat[4][4] = 0.9;
  

  // Connect RH to TO:
  connection_mat[5][1] = 2.5;

  // Recurrent RH
  connection_mat[6][5] = 2.5;
  connection_mat[6][6] = 1.07;

  // RH_move_fw
  connection_mat[7][6] = 2.5;

  // Hip output neuron
  connection_mat[8][7] = 2.5;

  float a1 = -120000;
  float a2 = -120000;
  float a3 = 0.0;
  
  connection_mat[9][10] = a1;
  connection_mat[10][11] = a1;
  connection_mat[11][12] = a1;
  connection_mat[12][9] = a1;

  connection_mat[9][11] = a2;
  connection_mat[10][12] = a2;
  connection_mat[11][9] = a2;
  connection_mat[12][10] = a2;

  connection_mat[9][12] = a3;
  connection_mat[10][9] = a3;
  connection_mat[11][10] = a3;
  connection_mat[12][11] = a3;


  connection_mat[13][12] = -2.5;

  connection_mat[14][15] = -2.5;
  connection_mat[15][14] = -2.5;

 
  


  
  /*
   * Setup a connection to serila port for motor control, please only use "Serial2" to connect to motors
   * we keep using "Serial" for the debugging through the "Serial monitor" 
  */
  ax12a.begin(BaudRate, DirectionPin, &Serial2);
  
  
  /*
   * Each motor has an LED that you can use it as a display.
   * Here, we turn them ON for demonstration.
  */
  for(int ID = 0;ID<7;ID++){
    ax12a.ledStatus(ID, ON);
    delay(5);
  }
  
  /*
   * if you are interested in streaming data on bluetooth, you can use "ESP_BT.print(my_data)"
  */
  ESP_BT.begin(BluetoothDeviceName); //Bluetooth device name
  
  mydelay(1000,myd3);
  
  /*
   * move to initial positions as defined in the "the _six_heros.h"
  */
  for(int i=0;i<7;i++){
    ax12a.moveSpeed(i, initial_pos[i],100);
    delay(2);
  }
  mydelay(5000,myd3);
  
  /*
   * preparation to walk.
   * Here we show an example for step by step walking. initial part is in "setup()", the main code is in "loop()"
   * First, balance to left side by moving the torso "TO" wiht the speed "mySpeed"
  */
  //ax12a.moveSpeed(TO, initial_pos[TO]+torsoStep,mySpeed);
  //mydelay(myd,myd3);
  //mydelay(1000,myd3);
  
  
  // Set and print connection matrix:

   int i, j;
   for(i=0; i<NUMBER_RS_NEURONS; i++) {
       for(j=0; j<NUMBER_RS_NEURONS; j++) {
        /*
          if (i!=j){
            connection_mat[i][j] = -2.5;
          }
          */
          printf("%d", connection_mat[i][j]);
    }
    printf("\n");
  }

  setup_end = millis();
}


float torso_start = millis();
float prev_torso = 0;
bool print_neuron_output = false;
void loop()
{
  // Connection: Input from j to i! (connection_mat[1][2]: If neuron 2 is firing, neuron 1 receives an input depending on the connection strength defined in connection_mat[1][2].
  for(int i = 0; i < NUMBER_RS_NEURONS; i ++){
    float network_stimulus = 0;
    for(int j = 0; j < NUMBER_RS_NEURONS; j ++){
      network_stimulus += connection_mat[i][j] * rs_neuron[j].y; 
    }
    float stimulus = network_stimulus + rs_neuron[i].s;

    // Calculate membrane potential
    rs_neuron[i].x = (1 - time_step / tau) * rs_neuron[i].x_old + time_step * (stimulus - (rs_neuron[i].b * rs_neuron[i].x_prime_old)) / tau;

    // Calculate adaptation rate.
    rs_neuron[i].x_prime = (1 - time_step / rs_neuron[i].T) * rs_neuron[i].x_prime_old + time_step * rs_neuron[i].y / rs_neuron[i].T;

    // Calculate firing rate / "output" activation
    rs_neuron[i].y = max(0.0f, rs_neuron[i].x_old - rs_neuron[i].threshold);

    // Saving values for future calculations.
    rs_neuron[i].x_old = rs_neuron[i].x;
    rs_neuron[i].x_prime_old = rs_neuron[i].x_prime;   
  }


  // TO
  float torso = rs_neuron[0].y-rs_neuron[1].y;
  float left_hip =  0.88*(3.0*rs_neuron[10].y + 2.0*rs_neuron[11].y + 1.0*rs_neuron[12].y);
  float right_hip = - 0.88*(3.0*rs_neuron[12].y + 2.0*rs_neuron[9].y + 1.0*rs_neuron[10].y);
  
  delay(10);
  
  if (print_neuron_output == true){
    /*
    for(int i = 0; i < NUMBER_RS_NEURONS; i ++){
        Serial.printf("Neuron_%d:", i);
        Serial.print(rs_neuron[i].y);
        Serial.print("  "); // Separate between neurons in plotter
    }
      //Serial.println("  "); // Separate between timepoints in plotter

    */

    Serial.printf("TO:");
    Serial.print(torso);
    Serial.print("  "); // Separate between neurons in plotter

    /*
    Serial.printf("LK:");
    Serial.print(-rs_neuron[2].y);
    Serial.print("  "); // Separate between neurons in plotter
    */

    Serial.printf("LH:");
    Serial.print(rs_neuron[3].y/2);
    Serial.print("  "); // Separate between neurons in plotter

    /*
    Serial.printf("RK:");
    Serial.print(rs_neuron[4].y);
    Serial.print("  "); // Separate between neurons in plotter
    */

    Serial.printf("RH:");
    Serial.print(-rs_neuron[5].y/2);
    Serial.print("  "); // Separate between neurons in plotter

    Serial.printf("Rec_RH:");
    Serial.print(rs_neuron[6].y);
    Serial.print("  "); // Separate between neurons in plotter

    Serial.printf("RH_forw:");
    Serial.print(rs_neuron[7].y);
    Serial.print("  "); // Separate between neurons in plotter

    Serial.printf("Hip_int:");
    Serial.print(rs_neuron[8].y);
    Serial.print("  "); // Separate between neurons in plotter
  
    // END PRINTING:
    Serial.println("  "); // Separate between timepoints in plotter
  }


  /*
   * TO: 2 neurons, n0-n1
   * RH: 4 neurons, 2*2neurons,
   * LH: 
   * RK: 1 neuron, connected to TO, higher frequency
   * LK: 1 neuron, reversely connected to TO / = reverse of right knee
   * 
   * 
   */

  // Torso oscillation definition:
  // The updated output of neuron 2 is subtracted from the cpg output of neuron 1. This leads to a full movement range both
  // to the left and right side. If only one neuron was used, the robot could only bend the torso motor to one side.
  // For a faster movement, the speed could be increased.

  
  /*
  Serial.printf("Neuron0:");
  Serial.print(rs_neuron[0].y);
  Serial.print("  ");
  Serial.printf("Neuron1:");
  Serial.print(rs_neuron[1].y);
  Serial.print("  ");
  Serial.printf("Neuron9:");
  Serial.print(rs_neuron[9].y);
  Serial.print("  ");
  Serial.printf("Neuron10:");
  Serial.print(rs_neuron[10].y);
  Serial.print("  ");
  Serial.printf("Neuron11:");
  Serial.print(rs_neuron[11].y);
  Serial.print("  ");
  Serial.printf("Neuron12:");
  Serial.print(rs_neuron[12].y);
  Serial.print("  ");
  Serial.println("  ");
  */
  
  
  if(millis() - setup_end > 5000){
      TO_current = max(405.0, min(645.0, initial_pos[TO] - torso*torsoStep*2.4));
  ax12a.moveSpeed(TO, TO_current, mySpeed);
  delay(2);
  LK_current = initial_pos[LK] - (hipstep * max(-2.0, -4.1 * rs_neuron[2].y));
  ax12a.moveSpeed(LK, LK_current,mySpeed);
  delay(2);
  LH_current = initial_pos[LH] + 20.0 - (hipstep * 0.7* left_hip);
  ax12a.moveSpeed(LH, LH_current,0.3*mySpeed);
  delay(2);
  RK_current = initial_pos[RK] - (hipstep * min(2.0, 4.1 * rs_neuron[4].y));
  ax12a.moveSpeed(RK, RK_current,mySpeed);
  delay(2);
  RH_current = initial_pos[RH] - 20.0 - (hipstep * 0.7* right_hip);
  ax12a.moveSpeed(RH, RH_current,0.3*mySpeed);
  delay(2);
  print_motors();
  
  

  if (prev_torso <= 0 && torso > 0){
    torso_start = millis();
  } else if (prev_torso > 0 && torso <= 0){
    //Serial.print("TO_dt: ");
    //Serial.println(millis() - torso_start);
  }
 prev_torso = torso;
  }


}
