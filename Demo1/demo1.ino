//SEED lab mini project 1
//include librairies

//encoder
#define ENCODER_OPTIMIZE_INTERRUPTS
#include<Encoder.h>

//I2C communication
#include<Wire.h>

//from project documentation and for loop
//positions
float desired_pos[2] = {0};
float actual_pos[2] = {0};
float prev_pos[2] = {0};
float pos_error[2] = {0};
float delta_pos[2] = {0};
int correct = 0;

//speeds
float desired_speed[2] = {0};
float actual_speed[2] = {0};
float error[2] = {0};

//voltage and integral error
float voltage[2] = {0};
float integral_error[2] = {0};

//encoders
Encoder motor1(3, 6);
Encoder motor2(2, 5);

//I2C wires
#define addr 8
volatile uint8_t offset = 0;
volatile uint8_t instruction[8]= {0};
volatile uint8_t msgLength = 0;

//pi input
int pi_input[2] = {0};

//motors
#define m1 9
#define m2 10
#define dir_m1 7
#define dir_m2 8
#define en 4
float pwm_m1;
float pwm_m2;

//constants
float Kp_pos = 2.5; //Kp we arrived at from matlab testing
float Ki_pos = 0.3; //Ki we found from matlab
int i;
float pi = 3.141592;
float Kp[2] = {1.5,1.5};
float battery_voltage = 7.8;
float wheel_rad = .25;
float robot_width = 1;

//Desired Stuff
float desired_loc[2] = {10,10};
float desired_phi = pi;
bool rot = true;

//timing
unsigned long desired_Ts_ms = 10;
unsigned long last_time_ms;
unsigned long start_time_ms;
float delta_time;
float prev_time;
float current_time;



void setup() {
  // put your setup code here, to run once:
  //initialize serial
  Serial.begin(115200);

  //initialize I2C
  Wire.begin(addr);

  //set callbacks for I2C interrupts
  Wire.onReceive(receive);

  //timing
  last_time_ms = millis();
  start_time_ms = last_time_ms;

  //enable, set directions
  digitalWrite(en, HIGH);
  digitalWrite(dir_m1, HIGH);
  digitalWrite(dir_m2, HIGH);

  //voltage
  voltage[0] = 0;
  voltage[1] = 0;

  //zero the positions
  prev_pos[0] = 0;
  prev_pos[1] = 0;

  //update the desired position
  desired_pos[0] = desired_loc[0];
  desired_pos[1] = desired_loc[1];

  //Start by rotating
  if (desired_phi != 0){
    desired_pos[0] = desired_phi * robot_width / 2;
    desired_pos[1] = -desired_phi * robot_width / 2;
  }

  //start MATLAB input
  Serial.println("Ready!");

}

// function called when an I2C interrupt event happens
void receive() {
  // Set the offset, this will always be the first byte.
  offset = Wire.read();
  // If there is information after the offset, it is telling us more about the command.
  while (Wire.available()) {
    instruction[msgLength] = Wire.read();
    msgLength++;
  }
}



float getLocation(Encoder motorPin){
  //we no longer need individual counts for m1 or m2 because this is a function
  long pos_counts;
  float pos_rad;

  //read encoder val and compute position of motor
  pos_counts = motorPin.read();
  pos_rad = wheel_rad*2*pi*(float)pos_counts/3200;

  return pos_rad;
}

float getVelocity(float delta_time, float delta_pos){
  //change in position over change in time
  float velocity;

  velocity = delta_pos / delta_time;

  return velocity;
}

void loop() {

  //set the current time
  current_time = (float)(last_time_ms - start_time_ms)/1000;

  //get locations of motors
  actual_pos[0] = getLocation(motor1);
  actual_pos[1] = getLocation(motor2);
  
  //get velocity of motors dx/dt
  delta_time = float(current_time - prev_time)/1000;
  delta_pos[0] = actual_pos[0] - prev_pos[0];
  delta_pos[1] = actual_pos[1] - prev_pos[1];

  actual_speed[0] = getVelocity(delta_time, delta_pos[0]);
  actual_speed[1] = getVelocity(delta_time, delta_pos[1]);

  //integral correction
  for(int j = 0; j<2; j++)
    for(i=0;i<2;i++) {
      pos_error[i] = desired_pos[i] - actual_pos[i];
      integral_error[i] = integral_error[i] + pos_error[i]*((float)desired_Ts_ms /1000);
      desired_speed[i] = Kp_pos * pos_error[i] + Ki_pos * integral_error[i];
      error[i] = desired_speed[i] - actual_speed[i];
      if (i == 1)
        voltage[i] = -1*Kp[i]*error[i];
      else
        voltage[i] = Kp[i]*error[i];

    }

    //derivative correction
    //motor 1
    if(voltage[0] > 0){
      digitalWrite(dir_m1,LOW);
      } else {
      digitalWrite(dir_m1,HIGH);
    }

    // Apply the requested voltage, up to the maximum available
    pwm_m1 = 255*abs(voltage[0])/battery_voltage;
    if(loc_error[0] == 0) pwm_m1 = 0;
    analogWrite(m1,min(pwm_m1,255));

    //motor 2
    if(voltage[1] > 0){
      digitalWrite(dir_m2,LOW);
      } else {
      digitalWrite(dir_m2,HIGH);
    }
    
    // Apply the requested voltage, up to the maximum available
    pwm_m2 = 255*abs(voltage[1])/battery_voltage;
    if(loc_error[1] == 0) pwm_m2 = 0;
    analogWrite(m2,min(pwm_m2,255));

    //Check if we need to rotate, if we do, rotate before moving
    if (desired_phi != 0){
      if ((pos_error[0] > (float)-.05) && (pos_error[0] < (float).05) && (rot == true)){
        correct++;
      }
      //once we reach our desired rotation we wait for a little bit then proceed with moving forward
      if ((correct > 255) && (rot == true)){
        desired_pos[0] = desired_pos[0] + desired_loc[0];
        desired_pos[1] = desired_pos[1] + desired_loc[1];

        for(i = 0; i < 2; i++){
          integral_error[i] = 0;
          pos_error[i] = 0;
          error[i] = 0;
        }

        rot == false;
      }
    }

  //display results only if we are below 3 seconds
  //if(current_time <= 3.00){
    Serial.print(current_time);
    Serial.print("\t");
    Serial.print(desired_loc[0]);
    Serial.print("\t");
    Serial.print(current_loc[0]);
    Serial.print("\t");
    Serial.print(phi_prev);
    Serial.print("\t");
    Serial.print(current_loc[1]);
    Serial.println("");
    
    
    
  //}

  //}

  /*
   //print finished for end of testing
  if((current_time > 10.00) && (current_time < 10.05)){
      Serial.println("Finished");
      digitalWrite(en, LOW);
    }
  */

  //update previous position
  prev_pos[0] = actual_pos[0];
  prev_pos[1] = actual_pos[1];

  //get previous time
  prev_time = millis();

  //time delay for updating values
  while(millis()<last_time_ms + desired_Ts_ms){
    //wait until desired time passes to go out of loop
  }
  last_time_ms = millis();
}
