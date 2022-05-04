#define NUMBER_RS_NEURONS 20

//20 neuron system

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
void setup() {
  
  Serial.begin(115200);


 
  
}

/******************************************************/ 

int i = 0;
int j = 0;
float net_input = 0;



void loop()
{
  
  for(i = 0; i < NUMBER_RS_NEURONS; i ++){
    
   if (i < NUMBER_RS_NEURONS/2 ){  // lest side neurons update
    
    if(i == 0){
    
        net_input = net_input + 1.5 * rs_neuron[i + 10].y + 1.5 * rs_neuron[i + 1].y;
          
        rs_neuron[i].x = (1 - time_step/tau)*rs_neuron[i].x_old + time_step * (-net_input + 1.0  - (rs_neuron[i].b * rs_neuron[i].x_prime_old))/tau;
        
        rs_neuron[i].x_prime = (1 - time_step/rs_neuron[i].T)*rs_neuron[i].x_prime_old + time_step * rs_neuron[i].y / rs_neuron[i].T;
        
        rs_neuron[i].y = max(0.0f, rs_neuron[i].x_old - rs_neuron[i].theta);
            
        rs_neuron[i].x_old = rs_neuron[i].x;
        
        rs_neuron[i].x_prime_old = rs_neuron[i].x_prime;
        net_input = 0;
           
      }  
    else if(i!=0){ 
        net_input = net_input + 1.5 * rs_neuron[i-1].y + 1.5 * rs_neuron[i + 10].y + 1.5 * rs_neuron[i + 1].y;
          
         
        
        rs_neuron[i].x = (1 - time_step/tau)*rs_neuron[i].x_old + time_step * (-net_input + 1.0  - (rs_neuron[i].b * rs_neuron[i].x_prime_old))/tau;
        
        rs_neuron[i].x_prime = (1 - time_step/rs_neuron[i].T)*rs_neuron[i].x_prime_old + time_step * rs_neuron[i].y / rs_neuron[i].T;
        
        rs_neuron[i].y = max(0.0f, rs_neuron[i].x_old - rs_neuron[i].theta);
        
    
        rs_neuron[i].x_old = rs_neuron[i].x;
        
        rs_neuron[i].x_prime_old = rs_neuron[i].x_prime;
        net_input = 0;
      }
   }
    
  else if(i >= NUMBER_RS_NEURONS/2){  //right side neurons update
    if(i == 10){
    
        net_input = net_input + 1.5 * rs_neuron[i - 10].y + 1.5 * rs_neuron[i + 1].y;
          
        rs_neuron[i].x = (1 - time_step/tau)*rs_neuron[i].x_old + time_step * (-net_input + 1.0  - (rs_neuron[i].b * rs_neuron[i].x_prime_old))/tau;
        
        rs_neuron[i].x_prime = (1 - time_step/rs_neuron[i].T)*rs_neuron[i].x_prime_old + time_step * rs_neuron[i].y / rs_neuron[i].T;
        
        rs_neuron[i].y = max(0.0f, rs_neuron[i].x_old - rs_neuron[i].theta);
            
        rs_neuron[i].x_old = rs_neuron[i].x;
        
        rs_neuron[i].x_prime_old = rs_neuron[i].x_prime;
        net_input = 0;     
      
      }  
    else if(i!=10){ 
        net_input = net_input + 1.5 * rs_neuron[i-1].y + 1.5 * rs_neuron[i - 10].y + 1.5 * rs_neuron[i + 1].y;
          
         
        
        rs_neuron[i].x = (1 - time_step/tau)*rs_neuron[i].x_old + time_step * (-net_input + 1.0  - (rs_neuron[i].b * rs_neuron[i].x_prime_old))/tau;
        
        rs_neuron[i].x_prime = (1 - time_step/rs_neuron[i].T)*rs_neuron[i].x_prime_old + time_step * rs_neuron[i].y / rs_neuron[i].T;
        
        rs_neuron[i].y = max(0.0f, rs_neuron[i].x_old - rs_neuron[i].theta);
        
    
        rs_neuron[i].x_old = rs_neuron[i].x;
        
        rs_neuron[i].x_prime_old = rs_neuron[i].x_prime;
        net_input = 0;
    }
      
   }
 }
   
  

  Serial.print("Neuron0:"); Serial.print(rs_neuron[0].y); Serial.print("  ");
  Serial.print("Neuron1:"); Serial.print(rs_neuron[1].y); Serial.print("  ");
  Serial.print("Neuron2:"); Serial.print(rs_neuron[2].y); Serial.print("  ");
  Serial.print("Neuron3:"); Serial.print(rs_neuron[3].y); Serial.print("  ");
  Serial.print("Neuron4:"); Serial.print(rs_neuron[4].y); Serial.print("  ");
  Serial.print("Neuron5:"); Serial.print(rs_neuron[5].y); Serial.print("  ");
  Serial.print("Neuron6:"); Serial.print(rs_neuron[6].y); Serial.print("  ");
  Serial.print("Neuron7:"); Serial.print(rs_neuron[7].y); Serial.print("  ");
  Serial.print("Neuron8:"); Serial.print(rs_neuron[8].y); Serial.print("  ");
  Serial.print("Neuron9:"); Serial.print(rs_neuron[9].y); Serial.print("  ");
  Serial.print("Neuron10:"); Serial.print(rs_neuron[10].y); Serial.println("  ");
  


  delay(100);
  
    

}

   
  
