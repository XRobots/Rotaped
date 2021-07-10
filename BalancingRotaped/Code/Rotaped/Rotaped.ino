#include <PID_v1.h>

// Radio
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 8); // CE, CSN
const byte addresses[][6] = {"00001", "00002"};

//ODrive
#include <ODriveArduino.h>

//ODrive Objects
ODriveArduino odrive1(Serial2);

//**************remote control****************
struct RECEIVE_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
    int16_t menuDown;  
    int16_t Select;    
    int16_t menuUp;  
    int16_t toggleBottom;  
    int16_t toggleTop; 
    int16_t toggle1;
    int16_t toggle2;
    int16_t mode;  
    int16_t RLR;
    int16_t RFB;
    int16_t RT;
    int16_t LLR;
    int16_t LFB;
    int16_t LT;
};

RECEIVE_DATA_STRUCTURE mydata_remote;

float RLR = 0;
float RFB = 0;
float RT = 0;
float LLR = 0;
float LFB = 0;
float LT = 0;

unsigned long currentMillis;
unsigned long previousMillis;

int requested_state;

// Balancing PID

double Pk1 = 4500;  
double Ik1 = 5500;
double Dk1 = 38;

double SetpointAccum;
double Output1a;

double Setpoint1, Input1, Output1;    // PID variables
PID PID1(&Input1, &Output1, &Setpoint1, Pk1, Ik1 , Dk1, DIRECT);    // PID Setup

float pot;
float IMUroll;
float IMUpitch;

int sw1;    // Odrive init
float pot2;   // setpoint trim for single balancing wheel


void setup() {
  // put your setup code here, to run once:

  pinMode(5, INPUT_PULLUP);    // ODrive init

  PID1.SetMode(AUTOMATIC);              
  PID1.SetOutputLimits(-150000, 150000);
  PID1.SetSampleTime(10);
  
  pinMode(A0, INPUT);   // setpoint trim

  radio.begin();
  radio.openWritingPipe(addresses[0]); // 00002
  radio.openReadingPipe(1, addresses[1]); // 00001
  radio.setPALevel(RF24_PA_MIN);

  radio.startListening();
  
  Serial2.begin(115200); // ODrive
  Serial3.begin(115200); // read IMU data
  Serial.begin(115200); // debug
}

void loop() {

    currentMillis = millis();

    if (currentMillis - previousMillis >= 10) {  // start timed loop
          previousMillis = currentMillis;

           // check for radio data
          if (radio.available()) {
                  radio.read(&mydata_remote, sizeof(RECEIVE_DATA_STRUCTURE));   
          }  

          else{ Serial.println("no data");}

          // threshold remote data
          // some are reversed based on stick wiring in remote
          RFB = (thresholdStick(mydata_remote.RFB))*-1;   
          RLR = thresholdStick(mydata_remote.RLR);
          RT = thresholdStick(mydata_remote.RT);   
          LFB = (thresholdStick(mydata_remote.LFB))*-1;   
          LLR = thresholdStick(mydata_remote.LLR);
          LT = thresholdStick(mydata_remote.LT);

          RFB = RFB/25;                         // scale driving stick
          
          IMUroll = Serial3.parseFloat();       // read IMU roll
            if (Serial3.read() == '\n') {     // end of IMU data       
          }

          pot2 = analogRead(A0);              // setpoint trim
          pot2 = (pot2 - 512)/100; 

          sw1 = digitalRead(5);                 // init ODrive
            if (sw1 == 0) {
              OdriveInit1();
            }

          Setpoint1 = pot2+RFB;                   // PID calcs
          Setpoint1 = constrain(Setpoint1,-8,8);  // do not lean further than 8 degrees
          Input1 = IMUroll;
          PID1.Compute();

          LT = LT * 100;          // steering

          odrive1.SetVelocity(0, (Output1*-1)+LT);   // drive motors
          odrive1.SetVelocity(1, (Output1)+LT);

          

    }   // end of 10ms loop

  

}
