#ifndef THE_SIX_HEROS_h
#define THE_SIX_HEROS_h


#define knee_shift 200
#define hip_shift 75
#define ankle_shift 30
#define torso_shift 0

#define TO 0 
#define LH 1 
#define RH 2 
#define LK 3 
#define RK 4 
#define LA 5 
#define RA 6 

const int LFSRFrontPin = 39;
const int LFSRRearPin = 36;
const int RFSRFrontPin = 35;
const int RFSRRearPin = 34;

int LFSRFront = 0;
int LFSRRear = 0;
int RFSRFront = 0;
int RFSRRear = 0;


#ifdef KONOE 
int initial_pos[7]={ 512-torso_shift-0 , 512-hip_shift , 512+hip_shift+5 , 512-knee_shift , 512+knee_shift-0 , 512-ankle_shift , 512+ankle_shift+2 };
int phi[7] = {40,0,0,-180,-180,135,-135}; // [deg]
int range[7]={200,50,50,50,50, 50,50};
//int range[7]={0,0,0,0,0, 20,20};
String BluetoothDeviceName = "KONOE";
#endif 

#ifdef TRINITY
int initial_pos[7]={ 512-torso_shift+15 , 512-hip_shift , 512+hip_shift+20 , 512-knee_shift , 512+knee_shift , 512-ankle_shift , 512+ankle_shift };
int phi[7] = {60,0,0,-180,-180,135,-135}; // [deg]
int range[7]={150,50,50,50,50,25,25};
String BluetoothDeviceName = "TRINITY";
#endif 

#ifdef VALKENHAYN
int initial_pos[7]={ 512-torso_shift+15 , 512-hip_shift+8, 512+hip_shift-8, 512-knee_shift , 512+knee_shift , 512-ankle_shift , 512+ankle_shift+10};
// Works fine: int initial_pos[7]={ 512-torso_shift+15 , 512-hip_shift+8, 512+hip_shift-8, 512-knee_shift , 512+knee_shift , 512-ankle_shift , 512+ankle_shift+10};
int phi[7] = {60,0,0,-180,-180,135,-135}; // [deg]
int range[7]={150,50,50,50,50,25,25};
String BluetoothDeviceName = "VALKENHAYN";
#endif 


#endif
