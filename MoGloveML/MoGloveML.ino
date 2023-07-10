/*"MoGlove" machine trained flex based mocap glove
 * 
 * I would like to sincerly appologise for my terrible code. This was intentionally done to avoid reverse engineering XD
 * 
 * The MoGlove Project
 * Licensed under Creative Commons
 * 
 * 
 * To do 
 *  - Hard smoothing after transitioning to new pattern (LPF? Kolman? 
 *  
 *  
 *  
 *  -Hard coding the grip bones may be bad idea
 *      -User configurable poses (quats may be better)
 *  Done!
 *  
 * -Write a Blender script to adjust hand poses for different models
 *    - fbx file with finger pose data
 *    - frames per pose -> quats
 *    - outputs pattern.h that 
 * Done!
 *    
 * User adjustment patterns
 *    1. Load fbx with finger patterns
 *    2. Adjust the poses to fit the pattern
 *    3. Run the script to obtain new patterns.h
 *    4. Upload the code onto ESP32
 *    5. EZ 
 * 
 * Implement One Euro filter
 * 
 *    
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

const unsigned long debounceTime = 100; //0.1s
unsigned long debounceTimer = 0;

int debounceState = 0;
int prevPred;
int newPred;
int genuinePred;

const int res = 6;    //Resolution of quaternion output

unsigned long previousMillis = 0;   // Previous time value
const unsigned long interval = 10;  // Sampling time interval in milliseconds

const float cutoff_freq   = 2.5;  //Cutoff frequency in Hz
const float sampling_time = 0.01; //Sampling time in seconds.
IIR::ORDER  order  = IIR::ORDER::OD2; //  Filter order for input (Order (OD1 to OD4) Higher => Smoother but more latency and computation)

const float cutoff_freq_out   = 2.5;
  IIR::ORDER  order_out  = IIR::ORDER::OD2; // filter order for output

// (INPUT) Low-pass filter for each fingers
Filter f0(cutoff_freq, sampling_time, order);
Filter f1(cutoff_freq, sampling_time, order);
Filter f2(cutoff_freq, sampling_time, order);
Filter f3(cutoff_freq, sampling_time, order);
Filter f4(cutoff_freq, sampling_time, order);

// (OUTPUT) Low pass filter for each quats of each fingers... (Fuck thats awful)
Filter qout00w(cutoff_freq_out, sampling_time, order_out);
Filter qout00x(cutoff_freq_out, sampling_time, order_out);
Filter qout00y(cutoff_freq_out, sampling_time, order_out);
Filter qout00z(cutoff_freq_out, sampling_time, order_out);

Filter qout01w(cutoff_freq_out, sampling_time, order_out);
Filter qout01x(cutoff_freq_out, sampling_time, order_out);
Filter qout01y(cutoff_freq_out, sampling_time, order_out);
Filter qout01z(cutoff_freq_out, sampling_time, order_out);

Filter qout02w(cutoff_freq_out, sampling_time, order_out);
Filter qout02x(cutoff_freq_out, sampling_time, order_out);
Filter qout02y(cutoff_freq_out, sampling_time, order_out);
Filter qout02z(cutoff_freq_out, sampling_time, order_out);

Filter qout11w(cutoff_freq_out, sampling_time, order_out);
Filter qout11x(cutoff_freq_out, sampling_time, order_out);
Filter qout11y(cutoff_freq_out, sampling_time, order_out);
Filter qout11z(cutoff_freq_out, sampling_time, order_out);

Filter qout12w(cutoff_freq_out, sampling_time, order_out);
Filter qout12x(cutoff_freq_out, sampling_time, order_out);
Filter qout12y(cutoff_freq_out, sampling_time, order_out);
Filter qout12z(cutoff_freq_out, sampling_time, order_out);

Filter qout13w(cutoff_freq_out, sampling_time, order_out);
Filter qout13x(cutoff_freq_out, sampling_time, order_out);
Filter qout13y(cutoff_freq_out, sampling_time, order_out);
Filter qout13z(cutoff_freq_out, sampling_time, order_out);

Filter qout21w(cutoff_freq_out, sampling_time, order_out);
Filter qout21x(cutoff_freq_out, sampling_time, order_out);
Filter qout21y(cutoff_freq_out, sampling_time, order_out);
Filter qout21z(cutoff_freq_out, sampling_time, order_out);

Filter qout22w(cutoff_freq_out, sampling_time, order_out);
Filter qout22x(cutoff_freq_out, sampling_time, order_out);
Filter qout22y(cutoff_freq_out, sampling_time, order_out);
Filter qout22z(cutoff_freq_out, sampling_time, order_out);

Filter qout23w(cutoff_freq_out, sampling_time, order_out);
Filter qout23x(cutoff_freq_out, sampling_time, order_out);
Filter qout23y(cutoff_freq_out, sampling_time, order_out);
Filter qout23z(cutoff_freq_out, sampling_time, order_out);

Filter qout31w(cutoff_freq_out, sampling_time, order_out);
Filter qout31x(cutoff_freq_out, sampling_time, order_out);
Filter qout31y(cutoff_freq_out, sampling_time, order_out);
Filter qout31z(cutoff_freq_out, sampling_time, order_out);

Filter qout32w(cutoff_freq_out, sampling_time, order_out);
Filter qout32x(cutoff_freq_out, sampling_time, order_out);
Filter qout32y(cutoff_freq_out, sampling_time, order_out);
Filter qout32z(cutoff_freq_out, sampling_time, order_out);

Filter qout33w(cutoff_freq_out, sampling_time, order_out);
Filter qout33x(cutoff_freq_out, sampling_time, order_out);
Filter qout33y(cutoff_freq_out, sampling_time, order_out);
Filter qout33z(cutoff_freq_out, sampling_time, order_out);

Filter qout41w(cutoff_freq_out, sampling_time, order_out);
Filter qout41x(cutoff_freq_out, sampling_time, order_out);
Filter qout41y(cutoff_freq_out, sampling_time, order_out);
Filter qout41z(cutoff_freq_out, sampling_time, order_out);

Filter qout42w(cutoff_freq_out, sampling_time, order_out);
Filter qout42x(cutoff_freq_out, sampling_time, order_out);
Filter qout42y(cutoff_freq_out, sampling_time, order_out);
Filter qout42z(cutoff_freq_out, sampling_time, order_out);

Filter qout43w(cutoff_freq_out, sampling_time, order_out);
Filter qout43x(cutoff_freq_out, sampling_time, order_out);
Filter qout43y(cutoff_freq_out, sampling_time, order_out);
Filter qout43z(cutoff_freq_out, sampling_time, order_out);

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
    //Serial.print("sum: ");
    //Serial.println(sum);
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
    //Serial.print("interval =");
    //Serial.println(currentMillis - previousMillis);
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

    //no nm -> normalized  for bendiness
    //nm -> normalized for trained dataset

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
    
    float cw = 1.25 - 2*abs(out - pred);  //125% - offset
    if(cw> 1.00){   //cut-off at 100% max
      cw = 1.00f;
    }
    
    float bw = 1.0 - cw;
    
    //Software debounce when genuine prediction is made for substantial time => 0.1s
    if(debounceState == 1){
      if(newPred != pred){
        //Treated as noise
        debounceState = 0;
      }
      else if(currentMillis - debounceTimer > debounceTime){
        //Genuine change detected
        genuinePred = pred;
        debounceState = 0;
        //Want hard smoothing here... LPF the output for some limited duration
        //Or perhaps...Another filter for pattern?
      }
      else{
        //still under debounce test
        prevPred = pred;
      }
    }

    if(debounceState == 0){
      if(prevPred != pred){   //Change detected
        //Start the timer
        debounceTimer = millis();
        newPred = pred;
        debounceState = 1;
      }
      prevPred = pred;
    }
    /*
    Serial.print("Pred: ");
    Serial.print(pred);
    */
    Serial.print("GenuinePred: ");
    Serial.println(genuinePred);
    Serial.print("Conf: ");
    Serial.println(cw);
    
    Quaternion qout00;
    Quaternion qout01;
    Quaternion qout02;
    
    Quaternion qout11;
    Quaternion qout12;
    Quaternion qout13;
    
    Quaternion qout21;
    Quaternion qout22;
    Quaternion qout23;
    
    Quaternion qout31;
    Quaternion qout32;
    Quaternion qout33;
    
    Quaternion qout41;
    Quaternion qout42;
    Quaternion qout43;

    //Hard coding the curl bone rig so u dont have to rig it urself ;D
    //Probably good idea to have it user adjustable
    Quaternion qbend00 = {qfins00[20][0],qfins00[20][1],qfins00[20][2],qfins00[20][3]*fval0};
    normQuat(qbend00);
    Quaternion qbend01 = {qfins01[20][0],qfins01[20][1],qfins01[20][2],qfins01[20][3]*fval0};
    normQuat(qbend01);
    Quaternion qbend02 = {qfins02[20][0],qfins02[20][1],qfins02[20][2],qfins02[20][3]*fval0};
    normQuat(qbend02);
    
    Quaternion qbend11 = {qfins11[20][0],qfins11[20][1]*fval1 ,qfins11[20][2]*fval1,qfins11[20][3]*fval1};
    normQuat(qbend11);
    Quaternion qbend12 = {qfins12[20][0],qfins12[20][1]*fval1 ,qfins12[20][2],qfins12[20][3]};
    normQuat(qbend12);
    Quaternion qbend13 = {qfins13[20][0],qfins13[20][1]*fval1 ,qfins13[20][2],qfins13[20][3]};
    normQuat(qbend13);
    
    Quaternion qbend21 = {qfins21[20][0],qfins21[20][1]*fval2 ,qfins21[20][2]*fval2,qfins21[20][3]*fval2};
    normQuat(qbend21);
    Quaternion qbend22 = {qfins22[20][0],qfins22[20][1]*fval2 ,qfins22[20][2],qfins22[20][3]};
    normQuat(qbend22);
    Quaternion qbend23 = {qfins23[20][0],qfins23[20][1]*fval2 ,qfins23[20][2],qfins23[20][3]};
    normQuat(qbend23);
    
    Quaternion qbend31 = {qfins31[20][0],qfins31[20][1]*fval3 ,qfins31[20][2]*fval3,qfins31[20][3]*fval3};
    normQuat(qbend31);
    Quaternion qbend32 = {qfins32[20][0],qfins32[20][1]*fval3 ,qfins32[20][2],qfins32[20][3]};
    normQuat(qbend32);
    Quaternion qbend33 = {qfins33[20][0],qfins33[20][1]*fval3 ,qfins33[20][2],qfins33[20][3]};
    normQuat(qbend33);
    
    Quaternion qbend41 = {qfins41[20][0],qfins41[20][1]*fval4 ,qfins41[20][2]*fval4,qfins41[20][3]*fval4};
    normQuat(qbend41);
    Quaternion qbend42 = {qfins42[20][0],qfins42[20][1]*fval4 ,qfins42[20][2],qfins42[20][3]};
    normQuat(qbend42);
    Quaternion qbend43 = {qfins43[20][0],qfins43[20][1]*fval4 ,qfins43[20][2],qfins43[20][3]};
    normQuat(qbend43);
    
    //if(0){
    if((0<=pred)&&(pred<=18)){
      //Qout = (A*Qbend) + (B*conf*qpat)
      //* => Scalar multiplication of quaternion
      //+ => Quaternion addition
      //A&B are tuning variables...
      
      //fin01 = > [1,0,0,-1.15]
      //fin1 => [1,1.25,0,0]
      //Obtain Pattern Quaternion

      Quaternion qpat00 = {qfins00[genuinePred][0], qfins00[genuinePred][1], qfins00[genuinePred][2], qfins00[genuinePred][3]};
      Quaternion qpat01 = {qfins01[genuinePred][0], qfins01[genuinePred][1], qfins01[genuinePred][2], qfins01[genuinePred][3]};
      Quaternion qpat02 = {qfins02[genuinePred][0], qfins02[genuinePred][1], qfins02[genuinePred][2], qfins02[genuinePred][3]};
      
      Quaternion qpat11 = {qfins11[genuinePred][0], qfins11[genuinePred][1], qfins11[genuinePred][2], qfins11[genuinePred][3]};
      Quaternion qpat12 = {qfins12[genuinePred][0], qfins12[genuinePred][1], qfins12[genuinePred][2], qfins12[genuinePred][3]};
      Quaternion qpat13 = {qfins13[genuinePred][0], qfins13[genuinePred][1], qfins13[genuinePred][2], qfins13[genuinePred][3]};

      Quaternion qpat21 = {qfins21[genuinePred][0], qfins21[genuinePred][1], qfins21[genuinePred][2], qfins21[genuinePred][3]};
      Quaternion qpat22 = {qfins22[genuinePred][0], qfins22[genuinePred][1], qfins22[genuinePred][2], qfins22[genuinePred][3]};
      Quaternion qpat23 = {qfins23[genuinePred][0], qfins23[genuinePred][1], qfins23[genuinePred][2], qfins23[genuinePred][3]};

      Quaternion qpat31 = {qfins31[genuinePred][0], qfins31[genuinePred][1], qfins31[genuinePred][2], qfins31[genuinePred][3]};
      Quaternion qpat32 = {qfins32[genuinePred][0], qfins32[genuinePred][1], qfins32[genuinePred][2], qfins32[genuinePred][3]};
      Quaternion qpat33 = {qfins33[genuinePred][0], qfins33[genuinePred][1], qfins33[genuinePred][2], qfins33[genuinePred][3]};

      Quaternion qpat41 = {qfins41[genuinePred][0], qfins41[genuinePred][1], qfins41[genuinePred][2], qfins41[genuinePred][3]};
      Quaternion qpat42 = {qfins42[genuinePred][0], qfins42[genuinePred][1], qfins42[genuinePred][2], qfins42[genuinePred][3]};
      Quaternion qpat43 = {qfins43[genuinePred][0], qfins43[genuinePred][1], qfins43[genuinePred][2], qfins43[genuinePred][3]};

      
      //cw => confidence weight
      //bw => bendiness weight

      //Scale pattern Quaternion by confidence factor and confidence weight
      qpat00 = scaleQuat(qpat00, cw);
      qpat01 = scaleQuat(qpat01, cw);
      qpat02 = scaleQuat(qpat02, cw);
      
      qpat11 = scaleQuat(qpat11, cw);
      qpat12 = scaleQuat(qpat12, cw);
      qpat13 = scaleQuat(qpat13, cw);

      qpat21 = scaleQuat(qpat21, cw);
      qpat22 = scaleQuat(qpat22, cw);
      qpat23 = scaleQuat(qpat23, cw);

      qpat31 = scaleQuat(qpat31, cw);
      qpat32 = scaleQuat(qpat32, cw);
      qpat33 = scaleQuat(qpat33, cw);

      qpat41 = scaleQuat(qpat41, cw);
      qpat42 = scaleQuat(qpat42, cw);
      qpat43 = scaleQuat(qpat43, cw);
      
      //Scale the pattern Quaternion by bendiness weight
      qbend00 = scaleQuat(qbend00, bw);
      qbend01 = scaleQuat(qbend01, bw);
      qbend02 = scaleQuat(qbend02, bw);

      qbend11 = scaleQuat(qbend11, bw);
      qbend12 = scaleQuat(qbend12, bw);
      qbend13 = scaleQuat(qbend13, bw);

      qbend21 = scaleQuat(qbend21, bw);
      qbend22 = scaleQuat(qbend22, bw);
      qbend23 = scaleQuat(qbend23, bw);

      qbend31 = scaleQuat(qbend31, bw);
      qbend32 = scaleQuat(qbend32, bw);
      qbend33 = scaleQuat(qbend33, bw);

      qbend41 = scaleQuat(qbend41, bw);
      qbend42 = scaleQuat(qbend42, bw);
      qbend43 = scaleQuat(qbend43, bw);

      //Combine the effect of Pattern quat and Bend quat
      qout00 = addQuat(qpat00, qbend00);
      qout01 = addQuat(qpat01, qbend01);
      qout02 = addQuat(qpat02, qbend02);

      qout11 = addQuat(qpat11, qbend11);
      qout12 = addQuat(qpat12, qbend12);
      qout13 = addQuat(qpat13, qbend13);

      qout21 = addQuat(qpat21, qbend21);
      qout22 = addQuat(qpat22, qbend22);
      qout23 = addQuat(qpat23, qbend23);

      qout31 = addQuat(qpat31, qbend31);
      qout32 = addQuat(qpat32, qbend32);
      qout33 = addQuat(qpat33, qbend33);

      qout41 = addQuat(qpat41, qbend41);
      qout42 = addQuat(qpat42, qbend42);
      qout43 = addQuat(qpat43, qbend43);
    }
    
    //If no pattern match found
    else{
      qout00 = qbend00;
      qout01 = qbend01;
      qout02 = qbend02;
      
      qout11 = qbend11;
      qout12 = qbend12;
      qout13 = qbend13;
      
      qout21 = qbend21;
      qout22 = qbend22;
      qout23 = qbend23;
      
      qout31 = qbend31;
      qout32 = qbend32;
      qout33 = qbend33;
      
      qout41 = qbend41;
      qout42 = qbend42;
      qout43 = qbend43;
    }

    //Filtering the output. Bad solution use Unscented Kalman filter?
    //It's kinda working tho
    qout00.w = qout00w.filterIn(qout00.w);
    qout00.x = qout00x.filterIn(qout00.x);
    qout00.y = qout00y.filterIn(qout00.y);
    qout00.z = qout00z.filterIn(qout00.z);
    normQuat(qout00);

    qout01.w = qout01w.filterIn(qout01.w);
    qout01.x = qout01x.filterIn(qout01.x);
    qout01.y = qout01y.filterIn(qout01.y);
    qout01.z = qout01z.filterIn(qout01.z);
    normQuat(qout01);
    
    qout02.w = qout02w.filterIn(qout02.w);
    qout02.x = qout02x.filterIn(qout02.x);
    qout02.y = qout02y.filterIn(qout02.y);
    qout02.z = qout02z.filterIn(qout02.z);
    normQuat(qout02);
    
    qout11.w = qout11w.filterIn(qout11.w);
    qout11.x = qout11x.filterIn(qout11.x);
    qout11.y = qout11y.filterIn(qout11.y);
    qout11.z = qout11z.filterIn(qout11.z);
    normQuat(qout11);

    qout12.w = qout12w.filterIn(qout12.w);
    qout12.x = qout12x.filterIn(qout12.x);
    qout12.y = qout12y.filterIn(qout12.y);
    qout12.z = qout12z.filterIn(qout12.z);
    normQuat(qout12);
    

    qout13.w = qout13w.filterIn(qout13.w);
    qout13.x = qout13x.filterIn(qout13.x);
    qout13.y = qout13y.filterIn(qout13.y);
    qout13.z = qout13z.filterIn(qout13.z);
    normQuat(qout13);

    qout21.w = qout21w.filterIn(qout21.w);
    qout21.x = qout21x.filterIn(qout21.x);
    qout21.y = qout21y.filterIn(qout21.y);
    qout21.z = qout21z.filterIn(qout21.z);
    normQuat(qout21);

    qout22.w = qout22w.filterIn(qout22.w);
    qout22.x = qout22x.filterIn(qout22.x);
    qout22.y = qout22y.filterIn(qout22.y);
    qout22.z = qout22z.filterIn(qout22.z);
    normQuat(qout22);

    qout23.w = qout23w.filterIn(qout23.w);
    qout23.x = qout23x.filterIn(qout23.x);
    qout23.y = qout23y.filterIn(qout23.y);
    qout23.z = qout23z.filterIn(qout23.z);
    normQuat(qout23);
    
    qout31.w = qout31w.filterIn(qout31.w);
    qout31.x = qout31x.filterIn(qout31.x);
    qout31.y = qout31y.filterIn(qout31.y);
    qout31.z = qout31z.filterIn(qout31.z);
    normQuat(qout31);
    
    qout32.w = qout32w.filterIn(qout32.w);
    qout32.x = qout32x.filterIn(qout32.x);
    qout32.y = qout32y.filterIn(qout32.y);
    qout32.z = qout32z.filterIn(qout32.z);
    normQuat(qout32);

    qout33.w = qout33w.filterIn(qout33.w);
    qout33.x = qout33x.filterIn(qout33.x);
    qout33.y = qout33y.filterIn(qout33.y);
    qout33.z = qout33z.filterIn(qout33.z);
    normQuat(qout33);

    qout41.w = qout41w.filterIn(qout41.w);
    qout41.x = qout41x.filterIn(qout41.x);
    qout41.y = qout41y.filterIn(qout41.y);
    qout41.z = qout41z.filterIn(qout41.z);
    normQuat(qout41);
    
    qout42.w = qout42w.filterIn(qout42.w);
    qout42.x = qout42x.filterIn(qout42.x);
    qout42.y = qout42y.filterIn(qout42.y);
    qout42.z = qout42z.filterIn(qout42.z);
    normQuat(qout42);
    
    qout43.w = qout43w.filterIn(qout43.w);
    qout43.x = qout43x.filterIn(qout43.x);
    qout43.y = qout43y.filterIn(qout43.y);
    qout43.z = qout43z.filterIn(qout43.z);
    normQuat(qout43);

    //Send the output (Optimzed to remove redundancy)
    SerialBT.print(String(qout00.w,res)+","+String(qout00.x,res)+","+String(qout00.y,res)+","+String(qout00.z,res)+",");
    SerialBT.print(String(qout01.w,res)+",0,0,"+String(qout01.z,res)+",");
    SerialBT.print(String(qout02.w,res)+",0,0,"+String(qout02.z,res)+",");
    
    SerialBT.print(String(qout11.w,res)+","+String(qout11.x,res)+","+String(qout11.y,res)+","+String(qout11.z,res)+",");
    SerialBT.print(String(qout12.w,res)+","+String(qout12.x,res)+",0,0,");
    SerialBT.print(String(qout13.w,res)+","+String(qout13.x,res)+",0,0,");

    SerialBT.print(String(qout21.w,res)+","+String(qout21.x,res)+","+String(qout21.y,res)+","+String(qout21.z,res)+",");
    SerialBT.print(String(qout22.w,res)+","+String(qout22.x,res)+",0,0,");
    SerialBT.print(String(qout23.w,res)+","+String(qout23.x,res)+",0,0,");

    SerialBT.print(String(qout31.w,res)+","+String(qout31.x,res)+","+String(qout31.y,res)+","+String(qout31.z,res)+",");
    SerialBT.print(String(qout32.w,res)+","+String(qout32.x,res)+",0,0,");
    SerialBT.print(String(qout33.w,res)+","+String(qout33.x,res)+",0,0,");
    
    SerialBT.print(String(qout41.w,res)+","+String(qout41.x,res)+","+String(qout41.y,res)+","+String(qout41.z,res)+",");
    SerialBT.print(String(qout42.w,res)+","+String(qout42.x,res)+",0,0,");
    SerialBT.println(String(qout43.w,res)+","+String(qout43.x,res)+",0,0");
   }
}
