/*Model trainer
 * 
 * 
 */
#include "OneButton.h"
#include <filters.h>

const int fin0 = 34;  // Pin for analog input 1 thumb
const int fin1 = 35;  // Pin for analog input 2 index
const int fin2 = 32;  // Pin for analog input 3 middle
const int fin3 = 33;  // Pin for analog input 4 ring
const int fin4 = 25;  // Pin for analog input 5 pinky

const int button = 27;
bool flexState =0;

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


unsigned long previousMillis = 0;   // Previous time value
const unsigned long interval = 10;  // Sampling time interval in milliseconds
const float cutoff_freq   = 10.0;  //Cutoff frequency in Hz
const float sampling_time = 0.01666666; //Sampling time in seconds.
IIR::ORDER  order  = IIR::ORDER::OD3; // Order (OD1 to OD4)


// Low-pass filter for each fingers
Filter f0(cutoff_freq, sampling_time, order);
Filter f1(cutoff_freq, sampling_time, order);
Filter f2(cutoff_freq, sampling_time, order);
Filter f3(cutoff_freq, sampling_time, order);
Filter f4(cutoff_freq, sampling_time, order);

OneButton Button(button, true);

void singleClick(){
  if (flexState == 0){
    min0 = f0.filterIn(analogRead(fin0));
    min1 = f1.filterIn(analogRead(fin1));
    min2 = f2.filterIn(analogRead(fin2));
    min3 = f3.filterIn(analogRead(fin3));
    min4 = f4.filterIn(analogRead(fin4));
  }
  else{
    max0 = f0.filterIn(analogRead(fin0));
    max1 = f1.filterIn(analogRead(fin1));
    max2 = f2.filterIn(analogRead(fin2));
    max3 = f3.filterIn(analogRead(fin3));
    max4 = f4.filterIn(analogRead(fin4));
  }
  flexState = 1-flexState;
}

void setup() {
  Serial.begin(115200);  // Initialize serial communication for debugging
  pinMode(button,INPUT_PULLUP);
  Button.attachClick(singleClick);
  delay(1000);        // Wait for serial to stabilize
}

void loop(){
  Button.tick();
  unsigned long currentMillis = millis();  // Get the current time
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;  // Update the previous time
    
    int val0 = analogRead(fin0);
    int val1 = analogRead(fin1);
    int val2 = analogRead(fin2);
    int val3 = analogRead(fin3);
    int val4 = analogRead(fin4);
    
    float fval0 = f0.filterIn(val0);
    float fval1 = f1.filterIn(val1);
    float fval2 = f2.filterIn(val2);
    float fval3 = f3.filterIn(val3);
    float fval4 = f4.filterIn(val4);
    
    fval0 = map(fval0, min0,max0,0,1000)/1000.0;
    fval0 = (fval0-lower0)/dif0;

    fval1 = map(fval1, min1,max1,0,1000)/1000.0;
    fval1 = (fval1-lower1)/dif1;
    
    fval2 = map(fval2, min2,max2,0,1000)/1000.0;
    fval2 = (fval2-lower2)/dif2;
    
    fval3 = map(fval3, min3,max3,0,1000)/1000.0;
    fval3 = (fval3-lower3)/dif3;
    
    fval4 = map(fval4, min4,max4,0,1000)/1000.0;
    fval4 = (fval4-lower4)/dif4;
    
    Serial.print(String(fval0,2)+","+String(fval1,2)+","+String(fval2,2)+","+String(fval3,2)+","+String(fval4,2)+"\n");
   }
}
