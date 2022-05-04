#define NUMBER_RS_NEURONS 3

// step response relation

/******************************************************/ 
//struct RSneuron 
/******************************************************/ 
struct RSneuron { 
  float x = 0;       // membrane potential
  float x_old = 0;
  float y = 0;       // output response of neuron
  float tau = 1;     // time const.
  float s_j = 1;     // impulse rate
  float c_j = 1;     // weight of synaptic connection
  float b = 0;       // adaptation coefficient b = 0, 2.5, inf
  float x_prime = 0; // degree of adaptation
  float x_prime_old = 0;
  float T = 2.5;       // time constant for adaptation T = 2.5, 12, inf
  float theta = 0;   // threshold
} rs_neuron[NUMBER_RS_NEURONS];

  float time_step = 0.1;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(3000);
  // put your main code here, to run repeatedly:
  Serial.println(rs_neuron[0].x);

  rs_neuron[1].b = 2.5;
  rs_neuron[1].T = 2.5;

  
  rs_neuron[2].b = 10000;
  rs_neuron[2].T = 10000;
  
  
}

/******************************************************/ 


void loop() {
  for(int i = 0; i<NUMBER_RS_NEURONS - 1; i++){
    rs_neuron[i].y = max(0.0f, rs_neuron[i].x - rs_neuron[i].theta);
    rs_neuron[i].x_prime = rs_neuron[i].x_prime_old + time_step * ((rs_neuron[i].y - rs_neuron[i].x_prime_old) / rs_neuron[i].T);
    rs_neuron[i].x = rs_neuron[i].x_old + time_step * (((rs_neuron[i].c_j * rs_neuron[i].s_j) - (rs_neuron[i].b * rs_neuron[i].x_prime) - rs_neuron[i].x_old)/ rs_neuron[i].T);

  //k1 = (T * y);
  //k2 = (T/2 * y)
  //x_prime = x_prime_old + (T * y);

  // x = x_old + (tau * ((c_j * s_j) - (b * x_prime_old)));
 // s_j = 0;
  rs_neuron[i].x_old = rs_neuron[i].x;
  rs_neuron[i].x_prime_old = rs_neuron[i].x_prime;
  }

  delay(100);
  
 

  Serial.print("Neuron0:"); Serial.print(rs_neuron[0].y); Serial.print("  ");
  Serial.print("Neuron1:"); Serial.print(rs_neuron[1].y); Serial.print("  ");
  Serial.print("Neuron2:"); Serial.print(rs_neuron[2].y); Serial.println("  ");

  
  
  

}
