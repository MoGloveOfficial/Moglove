/*"MoGlove" machine trained flex based mocap glove
 * 
 * 
 * The MoGlove Project
 * Licensed under Creative Commons
 * 
 */

#include "modelNN.h"
#include "patterns.h"
#include "quaternion.h"
#include <filters.h>
#include <BluetoothSerial.h>


const int fin0 = 34;  // Pin for analog input 1 thumb
const int fin1 = 35;  // Pin for analog input 2 index
const int fin2 = 32;  // Pin for analog input 3 middle
const int fin3 = 33;  // Pin for analog input 4 ring
const int fin4 = 25;  // Pin for analog input 5 pink

float min0;
float min1;
float min2;
float min3;
float min4;

float max0;
float max1;
float max2;
float max3;
float max4;

//Normalization vals
float lower0 = -0.14;
float upper0 = 1.11;
float dif0 = upper0 - lower0;

float lower1 = -0.21;
float upper1 = 1.25;
float dif1 = upper1 - lower1;

float lower2 = -0.11;
float upper2 = 1.25;
float dif2 = upper2 - lower2;

float lower3 = -0.03;
float upper3 = 1.22;
float dif3 = upper3 - lower3;

float lower4 = -0.04;
float upper4 = 1.11;
float dif4 = upper4 - lower4;

float upperThres = 9000.0;
float lowerThres = 6000.0;


//Sample at 100Hz, LPF => 10Hz

const unsigned long calTime = 3000; //Hold for 3 seconds
unsigned long startTime = 0;

const int res = 6;    //Resolution of quaternion output

unsigned long previousMillis = 0;   // Previous time value
const unsigned long interval = 10;  // Sampling time interval in milliseconds
const float cutoff_freq   = 10.0;  //Cutoff frequency in Hz
const float sampling_time = 0.01; //Sampling time in seconds.
IIR::ORDER  order  = IIR::ORDER::OD3; // Order (OD1 to OD4) Higher => Smoother but more latency


// Low-pass filter for each fingers
Filter f0(cutoff_freq, sampling_time, order);
Filter f1(cutoff_freq, sampling_time, order);
Filter f2(cutoff_freq, sampling_time, order);
Filter f3(cutoff_freq, sampling_time, order);
Filter f4(cutoff_freq, sampling_time, order);

Filter qfin00(cutoff_freq, sampling_time, order);
Filter qfin00(cutoff_freq, sampling_time, order);
Filter qfin00(cutoff_freq, sampling_time, order);


BluetoothSerial SerialBT;

void callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param){
  if(event == ESP_SPP_SRV_OPEN_EVT){
    Serial.println("Client Connected");
  }
}

void readFlex(float* fingers) {
  fingers[0] = f0.filterIn(analogRead(fin0));
  fingers[1] = f1.filterIn(analogRead(fin1));
  fingers[2] = f2.filterIn(analogRead(fin2));
  fingers[3] = f3.filterIn(analogRead(fin3));
  fingers[4] = f4.filterIn(analogRead(fin4));
}

void calFlex(){
  float fingers[5];
  float fin0sum;
  float fin1sum;
  float fin2sum;
  float fin3sum;
  float fin4sum;
  int samples = 0;
  float sum;
  //Calibrate open hand first!
  while(1){
    readFlex(fingers);
    //Get sum of fingers
    for(int i =0; i < 5; i++){
      sum += fingers[i];
    }
    Serial.print("sum: ");
    Serial.println(sum);
    if(sum < lowerThres){
      fin0sum += fingers[0];
      fin1sum += fingers[1];
      fin2sum += fingers[2];
      fin3sum += fingers[3];
      fin4sum += fingers[4];
      samples++;
      sum=0;
      if (startTime == 0){
        //Reset
        startTime = millis();
      }
      else{
        unsigned long currentTime = millis();
        if(currentTime-startTime>=calTime){
          min0 = fin0sum/samples;
          min1 = fin1sum/samples;
          min2 = fin2sum/samples;
          min3 = fin3sum/samples;
          min4 = fin4sum/samples;
          samples = 0;
          sum=0;
          Serial.println("Open Calibrated");
          break;
        }
      }
    }
    else{
        startTime = 0;
        sum=0;
        fin0sum = 0.0;
        fin1sum = 0.0;
        fin2sum = 0.0;
        fin3sum = 0.0;
        fin4sum = 0.0;
        samples = 0;
    }
  }
  //Calibrate closed fist next!
  while(1){
    readFlex(fingers);
    //Get sum of fingers
    for(int i =0; i < 5; i++){
      sum += fingers[i];
    }
    Serial.print("sum: ");
    Serial.println(sum);
    if(sum > upperThres){
      fin0sum += fingers[0];
      fin1sum += fingers[1];
      fin2sum += fingers[2];
      fin3sum += fingers[3];
      fin4sum += fingers[4];
      samples++;
      sum=0;
      if (startTime == 0){
        //Reset
        startTime = millis();
      }
      else{
        unsigned long currentTime = millis();
        if(currentTime-startTime>=calTime){
          max0 = fin0sum/samples;
          max1 = fin1sum/samples;
          max2 = fin2sum/samples;
          max3 = fin3sum/samples;
          max4 = fin4sum/samples;
          samples = 0;
          sum=0;
          Serial.println("Closed Calibrated");
          break;
        }
      }
    }
    else{
        startTime = 0;
        sum=0;
        fin0sum = 0.0;
        fin1sum = 0.0;
        fin2sum = 0.0;
        fin3sum = 0.0;
        fin4sum = 0.0;
        samples = 0;
    }
  }
}


void setup() {
  Serial.begin(115200);  // Initialize serial communication for debugging
  SerialBT.begin("MoGloveML");  
  while (!modelNN.begin()) {    //Init tf model
    Serial.print("Error in NN initialization: ");
    Serial.println(modelNN.getErrorMessage());
    }
    
  calFlex();          //Calibrate fingers
  delay(1000);        // Wait for serial to stabilize
}

void loop(){
  unsigned long currentMillis = millis();  // Get the current time
  if (currentMillis - previousMillis >= interval) {
    Serial.print("interval =");
    Serial.println(currentMillis - previousMillis);
    previousMillis = currentMillis;  // Update the previous time
    int val0 = analogRead(fin0);
    int val1 = analogRead(fin1);
    int val2 = analogRead(fin2);
    int val3 = analogRead(fin3);
    int val4 = analogRead(fin4);

    //Filter input
    float fval0 = f0.filterIn(val0);
    float fval1 = f1.filterIn(val1);
    float fval2 = f2.filterIn(val2);
    float fval3 = f3.filterIn(val3);
    float fval4 = f4.filterIn(val4);

    //nm -> normalized in trained dataset
    
    fval0 = map(fval0, min0,max0,0,1000)/1000.0;
    float fval0nm = (fval0-lower0)/dif0;
    fval1 = map(fval1, min1,max1,0,1000)/1000.0;
    float fval1nm = (fval1-lower1)/dif1;
    fval2 = map(fval2, min2,max2,0,1000)/1000.0;
    float fval2nm = (fval2-lower2)/dif2;
    fval3 = map(fval3, min3,max3,0,1000)/1000.0;
    float fval3nm = (fval3-lower3)/dif3;    
    fval4 = map(fval4, min4,max4,0,1000)/1000.0;
    float fval4nm = (fval4-lower4)/dif4;
    
    float in[5] = {fval0nm, fval1nm, fval2nm, fval3nm, fval4nm};
    float out = modelNN.predict(in); 
    int pred = round(out);
    float conf = 1-abs(out - pred);
    Serial.print("Prediction: ");
    Serial.print(pred);
    Serial.print(", conf: ");
    Serial.println(conf);
    
    if((0<=pred)&&(pred<=18)){
      //Qout = (A*Qbend) + (B*conf*qtr)
      //* => Scalar multiplication of quaternion
      //+ => Quaternion addition
      //A&B are tuning variables...
      
      //fin01 = > [1,0,0,-1]
      //fin1 => [1,1.15,0,0]
      //fin2 => [1,0,0,0];
      
      Quaternion qbend01 = {1.0f, 0.0f, 0.0f, -fval0};
      normQuat(qbend01);
      Quaternion qbend1 = {1.0f, 1.15f * fval1, 0.0f, 0.0f};
      normQuat(qbend1);
      Quaternion qbend2 = {1.0f, 1.15f * fval2, 0.0f, 0.0f};
      normQuat(qbend2);
      Quaternion qbend3 = {1.0f, 1.15f * fval3, 0.0f, 0.0f};
      normQuat(qbend3);
      Quaternion qbend4 = {1.0f, 1.15f * fval4, 0.0f, 0.0f};
      normQuat(qbend4);
      
      Quaternion qfin00 = {qfins00[pred][0], qfins00[pred][1], qfins00[pred][2], qfins00[pred][3]};
      Quaternion qfin01 = {qfins01[pred][0], qfins01[pred][1], qfins01[pred][2], qfins01[pred][3]};
      Quaternion qfin1 = {qfins1[pred][0], qfins1[pred][1], qfins1[pred][2], qfins1[pred][3]};
      Quaternion qfin2 = {qfins2[pred][0], qfins2[pred][1], qfins2[pred][2], qfins2[pred][3]};
      Quaternion qfin3 = {qfins3[pred][0], qfins3[pred][1], qfins3[pred][2], qfins3[pred][3]};
      Quaternion qfin4 = {qfins4[pred][0], qfins4[pred][1], qfins4[pred][2], qfins4[pred][3]};
      
      qfin00 = scaleQuat(qfin00, conf);
      qfin01 = scaleQuat(qfin01, conf);
      qfin1 = scaleQuat(qfin1, conf);
      qfin2 = scaleQuat(qfin2, conf);
      qfin3 = scaleQuat(qfin3, conf);
      qfin4 = scaleQuat(qfin4, conf);
      
      qfin01 = add2Quats(qfin01, qbend01);
      qfin1 = add2Quats(qfin1,qbend1);
      qfin2 = add2Quats(qfin2,qbend2);
      qfin3 = add2Quats(qfin3,qbend3);
      qfin4 = add2Quats(qfin4,qbend4);

      //Filter the output
      
      //Send the data
      //[w00,x00,y00,z00...w4,x4,y4,z4]
      SerialBT.print(String(qfin00.w,res)+", "+String(qfin00.x,res)+", "+String(qfin00.y,res)+", "+String(qfin00.z,res)+", ");
      SerialBT.print(String(qfin01.w,res)+", "+String(qfin01.x,res)+", "+String(qfin01.y,res)+", "+String(qfin01.z,res)+", ");
      SerialBT.print(String(qfin1.w,res)+", "+String(qfin1.x,res)+", "+String(qfin1.y,res)+", "+String(qfin1.z,res)+", ");
      SerialBT.print(String(qfin2.w,res)+", "+String(qfin2.x,res)+", "+String(qfin2.y,res)+", "+String(qfin2.z,res)+", ");
      SerialBT.print(String(qfin3.w,res)+", "+String(qfin3.x,res)+", "+String(qfin3.y,res)+", "+String(qfin3.z,res)+", ");
      SerialBT.println(String(qfin4.w,res)+", "+String(qfin4.x,res)+", "+String(qfin4.y,res)+", "+String(qfin4.z,res));
    }
    /*
    Serial.print("Prediction: ");
    Serial.print(pred);
    Serial.print(", conf: ");
    Serial.println(conf);
    //Serial.print(String(fval0,2)+","+String(fval1,2)+","+String(fval2,2)+","+String(fval3,2)+","+String(fval4,2)+"\n");
    */
   }
}
