

/*

*/ 



// Download LowPower library from http://github.com/rocketscream/Low-Power


#include <SPI.h>
#include "ADXL362.h"
#include <Arduino.h>
#include "algorithm.h"
#include "max30102.h"
#include <SoftwareSerial.h>

ADXL362 xl;

//  Setup interrupt on Arduino
//  See interrupt example at http://arduino.cc/en/Reference/AttachInterrupt
//
int16_t interruptPin = 6;          //Setup ADXL362 interrupt output to Interrupt 0 (digital pin 2)
int16_t interruptStatus = 0;

int16_t XValue, YValue, ZValue, Temperature;

#define PPG_INTERRUPT 8
SoftwareSerial BTSerial(2, 3); // RX | TX
#define ldrPin A0
  

const int chipSelectPin_Temp = 7;  
const int chipSelectPin_ADXL = 10;
// External variables
const int  signal = 0;    // Pin connected to the filtered signal from the circuit
unsigned long time;   
unsigned long frequency;
// Internal variables
int period = 2000;
int starttime = 2000;
int input = 0;
int lastinput = 0;
//BLE
int ldrValue = 0;
int i=0;
int j=0;
int ECG_DATA_SIZE=500; 
char freq[3]; 
// Three quantities below should be same. 




int16_t temp;


int32_t n_ir_buffer_length; //data length
uint8_t uch_dummy;
uint32_t aun_ir_buffer[30]; //infrared LED sensor data
uint32_t aun_red_buffer[30];  //red LED sensor data
uint32_t timer; 
uint32_t initialtime=2000; 

int32_t n_spo2;  //SPO2 value
int8_t ch_spo2_valid;  //indicator to show if the SPO2 calculation is valid
int32_t n_heart_rate; //heart rate value
int8_t  ch_hr_valid;  //indicator to show if the heart rate calculation is vali


void setup(){

    // Startup, soft reset
    Serial.begin(9600);
   
    delay(1000);    
    
  
    // Setup digital pin 7 for LED observation of awake/asleep  
    //pinMode(7, OUTPUT);    
   //digitalWrite(7, HIGH);
      pinMode(signal, INPUT);
  pinMode(ldrPin, INPUT);
  BTSerial.begin(9600);
  SPI.setDataMode(SPI_MODE3);// ADT7320 Supports only MODE3 SPI  
  // start the SPI library:  
  SPI.begin();  
  // initalize the  data ready and chip select pins:  
  pinMode(chipSelectPin_Temp, OUTPUT);  

 
  maxim_max30102_reset(); //resets the MAX30102

  pinMode(PPG_INTERRUPT, INPUT);  //pin D10 connects to the interrupt output pin of the MAX30102
  delay(1000);
  maxim_max30102_read_reg(REG_INTR_STATUS_1,&uch_dummy);  //Reads/clears the interrupt status register

  uch_dummy=Serial.read();
  maxim_max30102_init();  //initialize the MAX30102
  delay(1000);
      xl.begin();                //soft reset
    //  Setup Activity and Inactivity thresholds
    //     tweaking these values will effect the "responsiveness" and "delay" of the interrupt function
    //     my settings result in a very rapid, sensitive, on-off switch, with a 2 second delay to sleep when motion stops
    //xl.setupDCActivityInterrupt(300, 10);   // 300 code activity threshold.  With default ODR = 100Hz, time threshold of 10 results in 0.1 second time threshold
    //xl.setupDCInactivityInterrupt(80, 200);   // 80 code inactivity threshold.  With default ODR = 100Hz, time threshold of 30 results in 2 second time threshold
    Serial.println();

     //Other possible settings
    //  Motion activated On - stays on for 60 seconds 
    xl.setupDCActivityInterrupt(300, 10);   // 300 code activity threshold.  With default ODR = 100Hz, time threshold of 10 results in 0.1 second time threshold
    xl.setupDCInactivityInterrupt(80, 6000);  // 80 code inactivity threshold.  With default ODR = 100Hz, time threshold of 60000 results in 60 second time threshold
    
  
  
    //
    // Setup ADXL362 for proper autosleep mode
    //
  
    // Map Awake status to Interrupt 1
    // *** create a function to map interrupts... coming soon
    xl.SPIwriteOneRegister(0x2A, 0x40);   
  
    // Setup Activity/Inactivity register
    xl.SPIwriteOneRegister(0x27, 0x3F); // Referenced Activity, Referenced Inactivity, Loop Mode  
        
    // turn on Autosleep bit
    byte POWER_CTL_reg = xl.SPIreadOneRegister(0x2D);
    POWER_CTL_reg = POWER_CTL_reg | (0x04);       // turn on POWER_CTL[2] - Autosleep bit
    xl.SPIwriteOneRegister(0x2D, POWER_CTL_reg);
 


 
    //
    // turn on Measure mode
    //
    xl.beginMeasure();                      // DO LAST! enable measurement mode   
    xl.checkAllControlRegs();               // check some setup conditions    
    delay(100);
 

 
    //
    // Setup interrupt function on Arduino
    //    IMPORTANT - Do this last in the setup, after you have fully configured ADXL.  
    //    You don't want the Arduino to go to sleep before you're done with setup
    //
    pinMode(6, INPUT);    
    //attachInterrupt(0, interruptFunction, RISING);  // A high on output of ADXL interrupt means ADXL is awake, and wake up Arduino 
}



void loop(){

  //Read the temperature data  
      int tempData = readRegister(0x50, 2); // 0x50 is read commad for 0x02 register  
     tempData = tempData/8;// MSB bit15 and LSB bit4 so received value need to be divide/8  
      timer = millis();
     uint32_t elapsedtime = timer-initialtime; 
//     if (elapsedtime >= 20000){
//      Serial.println(F("time for next measurement")); 
//      delay(10);
//      elapsedtime =0; 
//      delay(10); 
//      }
  Serial.println(elapsedtime);
    // convert the temperature to celsius and display it:  
      float realTemp = (float)tempData * 0.0625;  
      String realT = String(realTemp);
  //
  //  Check ADXL362 interrupt status to determine if it's asleep
  //
  interruptStatus = digitalRead(interruptPin);
  Serial.println(interruptStatus);
  
// if ADXL362 is asleep, call LowPower.powerdown  
  if(interruptStatus == 0 && elapsedtime >=40000) { 
    initialtime=millis(); 
    Serial.println(elapsedtime);
    Serial.print(F("\nADXL went to sleep - Put Arduino to sleep now \n"));
    //digitalWrite(7, LOW);    // Turn off LED as visual indicator of asleep
    delay(100);
    //LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);     
    //reset elpased time
       

    delay(10);
      BTSerial.print("ECG");
      Serial.println(F("ECG"));
      for (i=0;i<ECG_DATA_SIZE;i++){
      
      ldrValue = analogRead(ldrPin);
      String ECG = String(ldrValue,DEC); 
      BTSerial.print(ECG);
      Serial.println(ECG); 
      delay(10);
      time = millis();
      if (ldrValue >=378){
    //Serial.println("Beat"); 
    delay(10);
    period = time - starttime;
    starttime = time;
    lastinput = input;
    frequency = 60000/period;

    //char freq[3]; 
    freq[0] = frequency/100+48; // Sort the hundreds character and convert it in ASCII
    freq[1] = (frequency/10)%10+48; // Sort the thents character and convert it in ASCII  
    freq[2] = frequency%10+48; 
    //Serial.println(F("HeartBeatECG"));
    
    
    
    }
      }
      // Send Heart Rate data from ECG
      Serial.println(frequency); 
      delay(10);
      BTSerial.print("EHRB");
      delay(10);
      String ECGHRB = String(frequency,DEC); 
      BTSerial.print(ECGHRB);
      delay(10);
      //Serial.println(HR);
      
      //Send PPG Data getPPG function. Updates two Global Variable arrays with PPG IR and PPG RED data. 

      getPPG(); 
 
      // Calculte Heart Rate and Blood Oxygen level based on PPG data 
      maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_spo2, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid);
      Serial.print(F(", HR="));
      delay(10);
      BTSerial.print("HRB");
      delay(10);
      Serial.print(n_heart_rate, DEC);
      String HRB = String(n_heart_rate,DEC); 
      BTSerial.print(HRB);
      
      Serial.print(F(", HRvalid="));
      Serial.print(ch_hr_valid, DEC);
      delay(10);
      Serial.print(F(", SPO2="));
      Serial.print(n_spo2, DEC);
  delay(10);
      Serial.print(F(", SPO2Valid="));
      BTSerial.print("OXG");
      delay(10);
      Serial.println(ch_spo2_valid, DEC);
      String OXG = String(n_spo2,DEC); 
      BTSerial.print(OXG);
      delay(10);
      BTSerial.print("TEMP");  
      delay(10);
      BTSerial.print(realT); 
      delay(10); 
    
  }
  
// if ADXL362 is awake, report XYZT data to Serial Monitor
  else{
    delay(10);
    //digitalWrite(7, HIGH);    // Turn on LED as visual indicator of awake
    xl.readXYZTData(XValue, YValue, ZValue, Temperature);   
    Serial.println(F("BE stable"));     
  }
  // give circuit time to settle after wakeup
  delay(20);
}


void getPPG(){
  uint32_t un_min, un_max, un_prev_data, un_brightness;  //variables to calculate the on-board LED brightness that reflects the heartbeats
  int32_t i=0;
  float f_temp;
  
  un_brightness=0;
  un_min=0x3FFFF;
  un_max=0;
  
  n_ir_buffer_length=30;  //buffer length of 100 stores 4 seconds of samples running at 25sps
   
//  maxim_max30102_LED_turnON();
  delay(10);   
  //read the first 100 samples, and determine the signal range
  BTSerial.print("PPGRED");
  Serial.println(F("PPG_RED"));
  for(i=0;i<n_ir_buffer_length;i++)
  {
    while(digitalRead(PPG_INTERRUPT)==1);  //wait until the interrupt pin asserts
    maxim_max30102_read_fifo((aun_red_buffer+i), (aun_ir_buffer+i));  //read from MAX30102 FIFO
    
    if(un_min>aun_red_buffer[i])
      un_min=aun_red_buffer[i];  //update signal min
    if(un_max<aun_red_buffer[i])
      un_max=aun_red_buffer[i];  //update signal max
    //Serial.print(F("red="));
    //Serial.print(aun_red_buffer[i], DEC);
     String PPGR = String(aun_red_buffer[i]); 
     BTSerial.print(PPGR);
     Serial.println(aun_red_buffer[i]);
    //Serial.print("\n");
   //Serial.print(F(", ir="));
    //Serial.println(aun_ir_buffer[i], DEC);
    //PPG_IR[i]=aun_ir_buffer[i]; 
  }
  
  delay(100); 
  BTSerial.print("PPGIR");
  Serial.println(F("PPG_IR"));
  i=0; 
  for(i=0;i<n_ir_buffer_length;i++)
  {
    while(digitalRead(PPG_INTERRUPT)==1);  //wait until the interrupt pin asserts
    maxim_max30102_read_fifo((aun_red_buffer+i), (aun_ir_buffer+i));  //read from MAX30102 FIFO
    
    if(un_min>aun_red_buffer[i])
      un_min=aun_red_buffer[i];  //update signal min
    if(un_max<aun_red_buffer[i])
      un_max=aun_red_buffer[i];  //update signal max
    //Serial.print(F("red="));
    //Serial.print(aun_red_buffer[i], DEC);
     String PPGI = String(aun_ir_buffer[i]); 
     BTSerial.print(PPGI);
     Serial.println(aun_ir_buffer[i]);
    //Serial.print("\n");
   //Serial.print(F(", ir="));
    //Serial.println(aun_ir_buffer[i], DEC);
   
  }
//  delay(100); 
//  maxim_max30102_LED_turnOFF();
}


void readADXLRawData(){
  // read all three axis in burst to ensure all measurements correspond to same sample time
  xl.readXYZTData(XValue, YValue, ZValue, Temperature);  
  Serial.print(F("XVALUE="));
  Serial.print(XValue);   
  Serial.print(F("\tYVALUE="));
  Serial.print(YValue);  
  Serial.print(F("\tZVALUE="));
  Serial.print(ZValue);  
  Serial.print(F("\tTEMPERATURE="));
  Serial.println(Temperature);   
  delay(100);                // Arbitrary delay to make serial monitor easier to observe
}
  
//Read from register from the ADT7320:  
unsigned int readRegister(byte thisRegister, int bytesToRead ) {  
  byte inByte = 0;          // incoming byte from the SPI  
  unsigned int result = 0;  // result to return  
  // take the chip select low to select the device:  
  digitalWrite(chipSelectPin_Temp, LOW);  
  // send the device the register you want to read:  
  SPI.transfer(thisRegister);  
  // send a value of 0 to read the first byte returned:  
  result = SPI.transfer(0xFF);  
  // decrement the number of bytes left to read:  
  bytesToRead--;  
  // if you still have another byte to read:  
  if (bytesToRead > 0) {  
    // shift the first byte left, then get the second byte:  
    result = result << 8;  
    inByte = SPI.transfer(0xFF);  
    // combine the byte you just got with the previous one:  
    result = result | inByte;  
    // decrement the number of bytes left to read:  
    bytesToRead--;  
  }  
  // take the chip select high to de-select:  
  digitalWrite(chipSelectPin_Temp, HIGH);  
  // return the result:  
  return(result);  
}  
  
//Sends a write command to ADT7320  
void writeRegister(byte thisRegister, byte thisValue) {  
  // take the chip select low to select the device:  
  digitalWrite(chipSelectPin_Temp, LOW);  
  SPI.transfer(thisRegister); //Send register location  
  SPI.transfer(thisValue);  //Send value to record into register  

  // take the chip select high to de-select:  
  digitalWrite(chipSelectPin_Temp, HIGH);  
}  



