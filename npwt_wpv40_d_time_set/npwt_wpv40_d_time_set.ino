//final code

#include <Wire.h>  // library for i2c communication
#include <Arduino.h>
#include <SparkFun_MicroPressure.h>  // library for pressure sensor
#include "RTClib.h"                  // library for rtc module

// RTC Object and Time Variables
RTC_DS3231 rtc;
// DateTime  startTimeL, pauseTime, startTimeB;
unsigned long startTime = 0;
unsigned long startTime_int_cycle = 0;
// Timing Constants and Variables
const long timerInterval = 1000;
unsigned long lastUpdateTime = 0;
unsigned long previousMillisTimer = 0;
const long intervalTimer = 990;
unsigned long startTimer, previoustimer, duration;
unsigned long previousMillis = 0;
const long interval = 1000;
unsigned long elapsedTime;
unsigned long totalSeconds;
unsigned long totalSecondsL;
unsigned long pausedMillisL = 0;     // Time spent paused in milliseconds
unsigned long startTimeMillisB = 0;  // Start time in milliseconds
unsigned long pausedMillisB = 0;     // Time spent paused in milliseconds
unsigned long startTimeMillisL = 0;  // Start time in milliseconds
unsigned long startTimeMillis5 = 0;  // Timer start time in milliseconds
// unsigned long elapsedMillis5 = 0;  // Elapsed time in milliseconds
unsigned long pausedMillis5 = 0;  // Time spent paused in milliseconds
unsigned long currentMillis = 0;  // Current time in milliseconds
// unsigned long previousMillis5 = 0; // Previous millis for interval tracking
int blockageThreshold = 100;  // Blockage threshold (placeholder)


int settime_hr,settime_min,settime_sec,setdate_yr=2025,setdate_mt=1,setdate_day=1;
// bool isRunning5 = false;           // Tracks if the timer is running
// bool isPaused5 = false;            // Tracks if the timer is paused

// const unsigned long interval5 = 1000; // Update interval for timer (1 second)
// unsigned int minutes5 = 0;         // Stores timer minutes
// Timing Variables for Leakage
unsigned long pauseMillisL = 0, previousMillisL = 0, currentMillisL, elapsedMillisL = 0;
const long intervalL = 1000;
bool isRunningL = false, isPausedL = false;

// Timing Variables for Blockage
bool isRunningB = false, isPausedB = false;
unsigned long previousMillisB = 0, currentMillisB = 0, elapsedMillisB = 0, pauseMillisB = 0, totalSecondsB = 0;
const unsigned long intervalB = 1000;

// Voltage Measurement Variables
#define ANALOG_IN_PIN2 PB1
#define ANALOG_IN_PIN1 PB0
float adc_voltage2 = 0.0, in_voltage2 = 0.0, adc_voltage1 = 0.0, in_voltage1 = 0.0;
float R1 = 100000.0, R2 = 10000.0, ref_voltage = 3.5,ref_voltage2 = 3.5;
int adc_value1 = 0, adc_value2 = 0;

const int numSamples = 10; // Number of samples to average
float sampleBuffer[numSamples]; // Buffer to store samples
int sampleIndex = 0; // Index to track current sample
float total = 0; // Running total for averaging
float smoothedVoltage ;
bool bufferFilled = false; // Tracks if the buffer is fully filled

  // int yearint,monthint,dayint,hourint,minint,secint,settimanddate; 

// DWIN Display Communication Variables
#define CMD_HEAD1 0x5A
#define CMD_HEAD2 0xA5
#define CMD_WRITE 0x82
#define CMD_READ 0x83
#define MIN_ASCII 32
#define MAX_ASCII 255
#define CMD_READ_TIMEOUT 50
#define READ_TIMEOUT 100
byte restartbuffer[] = { CMD_HEAD1, CMD_HEAD2, 0x07, CMD_WRITE, 0x00, 0x04, 0x55, 0xAA, 0x5A, 0xA5 };
byte currentPage; 
int address1 = 0, address2 = 0, data1 = 0, data2 = 0, dataValue, dataIndex = 1;
String addressString = "", separatedData = "", str;

// Motor and Solenoid Valve Control Variables
const int in1Pin = PC15, in2Pin = PC14, pwmPin = PA1, Solenoid_valve = PC13;
const int motorPin1 = PA11, motorPin2 = PA12, motorPWM = PA0, buzzer_pin = PA8;
int motorSpeed = 200, runState;
unsigned long time1 = 200, time2 = 4000, time3 = 6000;

// Pressure Measurement and Control Variables
SparkFun_MicroPressure mpr;
int pressure_mmhg = 0, absPressure, setPressure = 40, setPressureHigh = 40, setPressureLow = 40;
int HighPressure_time = 1, LowPressure_time = 1, intermittent_pressure = 40, variable_pressure = 100;
bool pressureset = false, pressuresetHigh = false, pressuresetLow = false;
bool valaveTriggered = false, resetkey = false, leakagetest = false, blockage_state = false;
int Continous_Mode = 0, Intermittent_Mode, Intermittent_state, valve_state, variable_state, high_presure_time_state = 0;
int setPressure_int_cycle_state, setPressure_int_cycle, startValue;

// Intermittent Mode Variables
int intermittentHigh = 60, intermittentLow = 60;
const int intermittentHighMin = 2, intermittentLowMin = 2;
unsigned long previousMillisHigh = 0, previousMillisLow = 0, intervalint = 1000;
String intermittent_dly_timer = "", timer_continous, timer_intermittent, String_timer_Int_cycle;
int countdownTimeInt, countdownTimeOriginalInt = countdownTimeInt, highrun = 0;
unsigned long previousMillisInt = 0, intervalInt = 75000;
bool countdownRunningInt = true;
unsigned long totalSeconds4 = 0;     // Timer variable
unsigned long currentMillisInt = 0;  // Milliseconds counter for intermittent mode

// RTC-Based Timers for Peristaltic Modes
unsigned long startMillissoak, currentMillissoak, intervalssoak = 1000;
int totalTimesoak;

// Timer Variables for Other Modes
unsigned long previousMillis3, previousMillis4, previousMillis5, totalSeconds5;
unsigned long interval3 = 1000, interval4 = 1000, interval5 = 1000;
unsigned long elapsedMillis3 = 0, elapsedMillis4 = 0, elapsedMillis5 = 0;
bool isRunning3 = false, isPaused3 = false, isRunning4 = false, isPaused4 = false, isRunning5 = false, isPaused5 = false;
unsigned long pauseMillis3 = 0, pauseMillis4 = 0, pauseMillis5 = 0;

// Timer6 Variables for Intermittent Operation
bool isRunning6 = false, isPaused6 = false;
unsigned long elapsedMillis6 = 0, pauseMillis6 = 0, previousMillis6 = 0, interval6 = 1000;

// Miscellaneous Variables
char daysOfTheWeek[7][12] = { "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday" };
unsigned int minutes, minutes4, minutes4_int, minutes5;
int hours = 0, hours4, seconds, seconds4;

// Time Strings for Display
String timeStr, dateStr;

int Device_mode = 0;    // Device mode variable
int Variable_Mode = 0;  // Variable pressure mode




void setup() {
  Serial.begin(115200);             // Initialize Serial for output at 115200 baud rate
  Serial2.begin(115200);            // Initialize Serial2 for input at 115200 baud rate
  pinMode(in1Pin, OUTPUT);          // motor pin 1
  pinMode(in2Pin, OUTPUT);          // motor pin2
  pinMode(pwmPin, OUTPUT);          // npwt motor pwm pin
  pinMode(motorPin1, OUTPUT);       // solenoid valeve pin1
  pinMode(motorPin2, OUTPUT);       // solenoid valeve pin2
  pinMode(motorPWM, OUTPUT);        // // solenoid valeve pwm pin
  pinMode(Solenoid_valve, OUTPUT);  // not used in the code
  // pinMode(PB5, OUTPUT);             // NA not used
  pinMode(ANALOG_IN_PIN2, INPUT);   // battery volatage pin
  pinMode(buzzer_pin, OUTPUT);      // buzzer
                                    //checking he initial pressure to release it
  if (absPressure > 10) {
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
  }

      int initialADC = analogRead(ANALOG_IN_PIN2);
    float initialVoltage = (initialADC * ref_voltage) / 4095.0;
    float initialInVoltage = initialVoltage / (R2 / (R1 + R2));

    for (int i = 0; i < numSamples; i++) {
        sampleBuffer[i] = initialInVoltage;
        total += initialInVoltage;
    }

// digitalWrite(PB3,HIGH);
// digitalWrite(PB5,HIGH);

  // digitalWrite(Solenoid_valve, HIGH);
  duration = 200;


  Wire.begin();
  // Initialize RTC
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

    // Serial.flush();
    // while (1) delay(10);
  }

  if (rtc.lostPower()) {
    Serial.println("RTC lost power, let's set the time!");
    // Set the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
  }


  //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));


  // rtc.adjust(DateTime(2025, 01, 02, 17, 32, 0));


  if (!mpr.begin()) {
    Serial.println("Cannot connect to MicroPressure sensor.");
    // while (1)
    // ;
  }
  // restart coandf for dwin lcd
  Serial2.write(restartbuffer, sizeof(restartbuffer));

smoothedVoltage =11;
  setIntegerVP(0x00, 0x82, 1);
  // setIntegerVP(0x25, 0x, 1);


  // loading icon animation  NA not used
   setText(0x25, 0x00," NPWT026");

  // setIntegerVP(0x43, 0x00, 0);
  delay(1000);
  // setIntegerVP(0x43, 0x00, 1);
  delay(1000);
  // setIntegerVP(0x43, 0x00, 2);
  delay(1000);
  // setIntegerVP(0x43, 0x00, 3);
  delay(1000);


  setPage(1);
  Time_date();
}

void loop() {
  // Check if it's time to run the pressure calculation
  // Serial.println("loop");

  unsigned long currentMillis = millis();  // intialization of the currentMillis variable to start recording the time in milli seconds from when the code started execution

  unsigned long currentTime = millis();  // intialization of the currentTime variable to start recording the time in milli seconds from when the code started execution
                                         // currentTime this variable is not used any where



  startTimer = millis();  // intialization of the startTimer variable to start recording the time in milli seconds from when the code started execution

  if (leakagetest == 1 || blockage_state) {  // cheking if one of the leakagetest or blockage_state states are satisfied

      if (blockage_state) {
        // setIntegerVP(0x41, 0x00, 23);  // blockge  alert icon
        if(Continous_Mode)
        {
        // setPage(6);
        }
        else if(Intermittent_Mode)
        {

        // setPage(10);

        }
      }

      // _______________________  leakge state icon is updating in lekage timer methods_____________________________________________

      // buzzer is common for both leakage and blockage alerts
      digitalWrite(buzzer_pin, 1);
      delay(100);
      digitalWrite(buzzer_pin, 0);
      delay(100);
    }

    else if (!blockage_state && !leakagetest) {  // if both states are zero
      // setIntegerVP(0x41, 0x00, 25);              // clear leakage/blockage alert icon
      digitalWrite(buzzer_pin, 0);
    }

  if (startTimer - previousMillisTimer >= duration) {  // timer code for preesure sensor code. bcs micro pressure sensor will not read the correct values without timer code.

    previousMillisTimer = startTimer;


    pressure_mmhg = (mpr.readPressure(TORR) - 710) * 0.60;

    pressure_mmhg = abs(pressure_mmhg);

    Serial.println("pressure_mmhg pppppppppppppppppppppppppppppppppppppppppppppppppppppppppppppppppppppppppp:"+ String(pressure_mmhg));
    
      // Serial.print("Total Seconds: ");
      // Serial.println(totalSecondsB);

    //absPressure = pressure_mmhg;
// pressure_mmhg = abs((mpr.readPressure(TORR) - 710) * 0.60);


// if (pressure_mmhg <= 130) {
//     absPressure = pressure_mmhg * (0.7 + (pressure_mmhg - 80) * 0.015); // 0.015 is the slope
// }

//  else if (pressure_mmhg > 130 && pressure_mmhg <= 150) {
//     absPressure = pressure_mmhg * 1.15;
// } 


// else {
//     absPressure = pressure_mmhg * 1.25;
// }

if(pressure_mmhg<=80)
{

absPressure = pressure_mmhg*0.70;

}

else if(pressure_mmhg>80 && pressure_mmhg<=90)
{

absPressure = pressure_mmhg*0.85;

}


else if(pressure_mmhg>90 &&  pressure_mmhg<=95)
{


absPressure = pressure_mmhg*0.92;

}

else if(pressure_mmhg>95 &&  pressure_mmhg<=98)
{


absPressure = pressure_mmhg*0.95;

}

else if(pressure_mmhg>98 &&  pressure_mmhg<=100)
{


absPressure = pressure_mmhg*0.95;

}


else if(pressure_mmhg>100 &&  pressure_mmhg<=105)
{


absPressure = pressure_mmhg*1;

}

else if(pressure_mmhg>105 &&  pressure_mmhg<=120)
{


absPressure = pressure_mmhg*1.1;

}


else if(pressure_mmhg>120 &&  pressure_mmhg<130)
{


absPressure = pressure_mmhg*1.15;

}


else if(pressure_mmhg>=130 &&  pressure_mmhg<=140)
{


absPressure = pressure_mmhg*1.2;

}

else if(pressure_mmhg>140 &&  pressure_mmhg<=150)
{


absPressure = pressure_mmhg*1.25;

}

else if(pressure_mmhg>150 &&  pressure_mmhg<160)
{


absPressure = pressure_mmhg*1.25;

}

else if(pressure_mmhg>=160 &&  pressure_mmhg<=170)
{


absPressure = pressure_mmhg*1.28;

}

else if(pressure_mmhg>170)
{


absPressure = pressure_mmhg*1.3;

}


Serial.println("absPressure mmhg: " + String(absPressure));

Serial.println("pressure mmhg: " + String(pressure_mmhg));


if(setPressure_int_cycle_state==2){
    /* code for updating the pressure value to the display when the continous mode absPressure is near to setPressure  */
    if ((absPressure > (setPressure - 10) && absPressure <= setPressure) || (absPressure < (setPressure + 15) && absPressure >= setPressure)) {
      //   // if(absPressure>50){
      //   if (absPressure % 5 == 0) {
      setIntegerVP(0x10, 0x01, setPressure);
      Serial.println("pressure rounding %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
    }

    
        else {
      //   setIntegerVP(0x10, 0x01, absPressure);
      updatePressureDisplay(absPressure);

      
    }

}
  

    /* code for updating the pressure value to the display when the intermittent high mode mode absPressure is near to setPressureHigh  */
if(setPressure_int_cycle_state==1){
    if ((absPressure > (setPressureHigh - 10) && absPressure <= setPressureHigh) || (absPressure < (setPressureHigh + 15) && absPressure >= setPressureHigh)) {
      //   // if(absPressure>50){
      //   if (absPressure % 5 == 0) {
      setIntegerVP(0x10, 0x01, setPressureHigh);
    }

        
        else {
      //   setIntegerVP(0x10, 0x01, absPressure);
      updatePressureDisplay(absPressure);
    }

}


    /* code for updating the pressure value to the display when the intermittent low mode mode absPressure is near to setPressureLow  */
if(setPressure_int_cycle_state==0){
    if ((absPressure > (setPressureLow - 10) && absPressure <= setPressureLow) || (absPressure < (setPressureLow + 15) && absPressure >= setPressureLow)) {
      //   // if(absPressure>50){
      //   if (absPressure % 5 == 0) {
      setIntegerVP(0x10, 0x01, setPressureLow);
    }

            else {
      //   setIntegerVP(0x10, 0x01, absPressure);
      updatePressureDisplay(absPressure);
    }

}

    /*written multiple if conditions for updating the absPressure value in both continous and intermittent modes with 5 number differences*/




       Serial.print("timer_continous_______________________________________________________________________________________:");

      Serial.println(timer_continous);

         Serial.print("timer_intermittent__________________________________________________________________________________++++:");

      Serial.println(timer_intermittent);
    /*
Here’s what’s happening step by step:

Timer Condition:

startTimer - previousMillisTimer calculates the elapsed time since the last sensor reading.
duration is the minimum time interval you want between consecutive readings.
The if condition ensures the sensor is read only after the desired time interval has passed.
Resetting the Timer:

previousMillisTimer = startTimer; resets the reference point for the next interval.
Reading and Processing Data:

The mpr.readPressure() function communicates with the sensor to fetch the pressure reading.
Adjustments are made to calibrate and normalize the data.

*/
// setIntegerVP(0x00, 0x02, 1);
    if (leakagetest == 1 || blockage_state) {  // cheking if one of the leakagetest or blockage_state states are satisfied

      if (blockage_state) {
        setIntegerVP(0x41, 0x00, 23);  // blockge  alert icon
        if(Continous_Mode)
        {
        // setPage(6);
        }
        else if(Intermittent_Mode)
        {

        // setPage(10);

        }
      }

      // _______________________  leakge state icon is updating in lekage timer methods_____________________________________________

      // buzzer is common for both leakage and blockage alerts
      // digitalWrite(buzzer_pin, 1);
      // delay(100);
      // digitalWrite(buzzer_pin, 0);
      // delay(100);
    }

    else if (!blockage_state && !leakagetest) {  // if both states are zero
      setIntegerVP(0x41, 0x00, 25);              // clear leakage/blockage alert icon
      // digitalWrite(buzzer_pin, 0);
    }





    // setIntegerVP(0x00, 0x82, 100);
  }
  if (currentMillis - previousMillis >= interval) {  // timer for all the operations except the display communications

    previousMillis = currentMillis;
    // Run the pressure calculation
    // if(absPressure<200)
    // {
    // }
    // Serial.println("totalSeconds5-----------: " + String(totalSeconds5));

    battery_icon();  //battery icon update method based on the voltage values of pb1 and pb0

   setText(0x25, 0x00," NPWT026");





    Serial.print("currentPage:---------");  // preesure feed abck in serial monitor
    Serial.println(currentPage);


    // based on ther current page in display written and set the modes for the device like  intermittent or continous or else pause play and also used them for releasing the pressures.


    if (currentPage == 7) {  // in this page continous mode is running
      Continous_Mode = 1;
      Intermittent_Mode = 0;
      // Variable_Mode = 0;
      // setPressureHigh =0;
      // setPressureLow =0;
      valve_state = 0;

      valveOFF();
    }


    else if (currentPage == 6) {  // in this page continous mode is stopped
      Continous_Mode = 0;
      Intermittent_Mode = 0;
      // Variable_Mode = 0;
      runState = 1;
      // stopped both leakage and blockage timers when motor is not running
      stopLeakageTimer();
      stopBlockageTimer();
      // release_pressure();

    }




    else if (currentPage == 9) {  // in this page Intermittent mode is running
      Continous_Mode = 0;
      // Variable_Mode = 0;
      Intermittent_Mode = 1;
      // setPressure =0;
      valve_state = 0;

      valveOFF();

    }

    else if (currentPage == 8) {  // in this page Intermittent mode is stopped
      Continous_Mode = 0;
      // Variable_Mode = 0;
      Intermittent_Mode = 0;

      // stopped both leakage and blockage timers when motor is not running
      stopLeakageTimer();   // leakge timer and blockage timer starting code is inside the continous mode and intermittent mode running codes
      stopBlockageTimer();  // leakge timer and blockage timer starting code is inside the continous mode and intermittent mode running codes
      // release_pressure();



    }

    else if (currentPage == 10) {
      Continous_Mode = 0;
      // Variable_Mode = 0;
      Intermittent_Mode = 0;

      // stopped both leakage and blockage timers when motor is not running
      stopLeakageTimer();   // leakge timer and blockage timer starting code is inside the continous mode and intermittent mode running codes
      stopBlockageTimer();  // leakge timer and blockage timer starting code is inside the continous mode and intermittent mode running codes

      // stopTimer_int();
      // release_pressure();


    } else if (currentPage == 11) {
      Continous_Mode = 0;
      // Variable_Mode = 0;
      Intermittent_Mode = 0;
      // stopped both leakage and blockage timers when motor is not running

      stopLeakageTimer();   // leakge timer and blockage timer starting code is inside the continous mode and intermittent mode running codes
      stopBlockageTimer();  // leakge timer and blockage timer starting code is inside the continous mode and intermittent mode running codes

      stopTimer_int();
      stopTimer_Int_cycle();
      // release_pressure();

      Serial.println("timer cleared +++++++++++++++++++++++++++++++++++++++++++++");
      // resatrt_int_timer=1;


    }

    else if (currentPage == 1) {

      // both timers are stoped in the main menu screen
      stopTimer_int();
      stopTimer_cont();
      // release_pressure();

    }
    // else if (currentPage == 17) {
    //   Continous_Mode = 0;
    //   Variable_Mode = 1;
    //   Intermittent_Mode = 0;
    // }

    else if (currentPage == 16) {
      Continous_Mode = 0;
      // Variable_Mode = 0;
      Intermittent_Mode = 0;
      stopLeakageTimer();   // leakge timer and blockage timer starting code is inside the continous mode and intermittent mode running codes
      stopBlockageTimer();  // leakge timer and blockage timer starting code is inside the continous mode and intermittent mode running codes
      // release_pressure();

    }

    else if (currentPage == 17) {
      Continous_Mode = 4;  // pause continous mode
      // Variable_Mode = 0;
      Intermittent_Mode = 0;
      // stopLeakageTimer();
      // stopBlockageTimer();

      // release_pressure();

    }

    else if (currentPage == 18) {
      Continous_Mode = 0;
      // Variable_Mode = 0;
      Intermittent_Mode = 4;  // pause intermittent mode
      // stopLeakageTimer();
      // stopBlockageTimer();

      // release_pressure();
    }


     if (currentPage == 19) {
     rtc.adjust(DateTime(setdate_yr, setdate_mt, setdate_day, settime_hr, settime_min, settime_sec));
    
    }


    // else{
    //   release_pressure();

    // }


    if (currentPage != 9 && currentPage != 7  && currentPage != 17  && currentPage != 18 ) {

      release_pressure();  // pressure will be released if the diplay running other than page 9 or 7  which are intermitent and contious mode running pages respectively
    }


    intermittentmode();  // this is the method which handels all the running conditions of intermirttent mode

    continousmode();  // this is the method which handels all the running conditions of continous mode



    // variablepressuremode();


    DateTime now = rtc.now();  // getting the current time

    // Create date and time strings
    dateStr = String(now.year(), DEC) + '/' + String(now.month(), DEC) + '/' + String(now.day(), DEC);  // date string variable

    // Serial.println("dateStr------------------------>:"+timeStr);
    timeStr = String(now.hour(), DEC) + ':' + String(now.minute(), DEC);  // current time string variable


    // sending the time and date values strings to the display
    // yearint = String(now.year(), DEC).toInt();
    // monthint = String(now.month(), DEC).toInt();
    // dayint = String(now.day(), DEC).toInt();
    // hourint = String(now.hour(), DEC).toInt();
    // minint = String(now.minute(), DEC).toInt();
    // secint = String(now.second(), DEC).toInt();

    // if(settimanddate=0){
    // setIntegerVP(0x16, 0x00, yearint);
    // setIntegerVP(0x17, 0x00, monthint);
    // setIntegerVP(0x18, 0x00, dayint);
    // setIntegerVP(0x11, 0x80, hourint);
    // setIntegerVP(0x13, 0x00, minint);
    // setIntegerVP(0x14, 0x00, secint);
    // settimanddate =1;

    // }

    Serial.println("__________________________________________---------------->" + timeStr);
    setText(0x36, 0x00, String("0" + timeStr + "  "));  //00


    setText(0x35, 0x00, String("0" + dateStr));  //0

    // setting the initial values of the all setpressure values to the display
    setIntegerVP(0x10, 0x00, setPressure);
    setIntegerVP(0x15, 0x05, setPressureHigh);
    setIntegerVP(0x15, 0x04, setPressureLow);
    setIntegerVP(0x15, 0x02, LowPressure_time);
    setIntegerVP(0x15, 0x00, HighPressure_time);

    // setIntegerVP(0x10, 0x00, setPressure);

        setIntegerVP(0x16, 0x00, setdate_yr);
    setIntegerVP(0x17, 0x00, setdate_mt);
    setIntegerVP(0x18, 0x00, setdate_day);
    setIntegerVP(0x13, 0x00, settime_min);
    setIntegerVP(0x14, 0x00, settime_sec);

    setIntegerVP(0x11, 0x80, settime_hr);



    // measuring the battery volatge and charging volatge values.


    // adc_value2 = analogRead(ANALOG_IN_PIN2);
    // adc_voltage2 = (adc_value2 * ref_voltage) / 4095.0;

    // in_voltage2 = adc_voltage2 / (R2 / (R1 + R2));

    
     total -= sampleBuffer[sampleIndex];

    // Read the new ADC value
     adc_value2 = analogRead(ANALOG_IN_PIN2);

    // Convert ADC value to voltage
     adc_voltage2 = (adc_value2 * ref_voltage) / 4095.0;

    // Calculate in_voltage2
    in_voltage2 = adc_voltage2 / (R2 / (R1 + R2));

    // Store the new sample in the buffer
    sampleBuffer[sampleIndex] = in_voltage2;

    // Add the new sample to the total
    total += in_voltage2;

    // Increment the index and wrap around if necessary
    sampleIndex = (sampleIndex + 1) % numSamples;
    if (sampleIndex == 0) {
        bufferFilled = true;
    }
    // Calculate the average
     smoothedVoltage = total / numSamples;


    adc_value1 = analogRead(ANALOG_IN_PIN1);
    adc_voltage1 = (adc_value1 * ref_voltage2) / 4095.0;
    in_voltage1 = adc_voltage1 / (R2 / (R1 + R2));



    Serial.print(F("Voltage: ++++++++++++++++++++++++++++++++++++++++++++++++++++++"));
    Serial.println(in_voltage2);
    Serial.println(in_voltage1);

    // getting the current page feedback from getpage method

    currentPage = getPage();
    Serial.print("Current Page: ");
    Serial.println(currentPage);


    // switch case for handling the timer values of the intermittent mode and continous modes

    switch (runState) {
      case -1:

        battery_icon();  // tested here
        Serial.println("icon updating...");
        break;
      case -2:
        // Timer is stopped
        // setText(0x70, 0x00, String("000:00:00 "));
        //  Serial.print("Device_Mode:");
        // Serial.println("No run");
        // isRunning3 = false;
        break;

      case -3:
        // Timer is stopped
        // setText(0x80, 0x00, String("000:00:00 "));
        //  Serial.print("Device_Mode:");
        // Serial.println("No run");
        // isRunning3 = false;
        break;
      case 1:
        // Timer is running


        //  Serial.print("Device_Mode:");
        //  Serial.println("continous Mode");



        if (!isRunning3) {
          start_cont_timer();  // Start the timer if not already running
        }
        updateTimer_cont();  // leakage and blockage timers for continous mode are started and updated inside it

        break;

      case 2:
        // Timer is running

        // device running in itermittent mode bcs runstate is 2

        if (!isRunning4) {
          startTimer_int();  // Start the timer if not already running
        }
        updateTimer_int();  // leakage and blockage timers for intermittent mode are started and updated inside it

        break;

      case 3:

        // for future purpose if variable mode is used
        break;
      default:
        // Handle other states if necessary

        break;
    }



    // if (currentTime - lastUpdateTime >= timerInterval) {  // use it for future purposes
    //   // Update the last time the timer was updated
    //   lastUpdateTime = currentTime;
    // }
  }


  // Main loop for reading and processing serial dwin display data
  // this code handels the display communications and reading the addresses and its data values

  if (Serial2.available()) {
    // delayMicroseconds(20);
    // Read the incoming byte data from the software serial
    String receivedData = "";
    while (Serial2.available()) {
      int byteReceived = Serial2.read();  // all the data red saved to this variable in  format
      if (byteReceived < 16) {
        receivedData += "0";  // Add leading zero for single digit bytes
      }
      receivedData += String(byteReceived, HEX) + " ";
    }
    // Remove the trailing space from the string
    receivedData.trim();

    /* Purpose: Adds a leading zero for single-digit hexadecimal values.
        Hexadecimal numbers between 0x0 and 0xF (decimal 0–15) are represented as a single character (0 to F). To make them two characters (e.g., 0A instead of A), a leading zero is prepended. */

    // Separate and label each byte
    dataIndex = 1;  // Initialize data index to keep track of the byte position in the sequence.

    for (int i = 0; i < receivedData.length(); i += 3) {  // Loop through receivedData, incrementing by 3 to process one byte (2 characters + 1 space) at a time.

      // Extract a substring representing the current hex byte (2 characters from index i).
      String hexByte = receivedData.substring(i, i + 2);

      // Convert the hex string into an integer value using base 16.
      int byteValue = strtol(hexByte.c_str(), NULL, 16);

      // Check the current data index and assign the byte value to the appropriate variable.
      if (dataIndex == 4) {
        address1 = byteValue;  // Assign the 4th byte to address1.
      } else if (dataIndex == 5) {
        address2 = byteValue;  // Assign the 5th byte to address2.
      } else if (dataIndex == 7) {
        data1 = byteValue;  // Assign the 7th byte to data1.
      } else if (dataIndex == 8) {
        data2 = byteValue;  // Assign the 8th byte to data2.
      }

      // Append the current byte's details to separatedData for display/logging purposes.
      // Format: "d<dataIndex>: <hexByte> "
      separatedData += "d" + String(dataIndex) + ": " + hexByte + " ";

      dataIndex++;  // Increment the data index for the next byte in the sequence.
    }


    // Print the separated and labeled data
    // Serial.println(separatedData);

    // Prepare address string
    addressString = "";
    if (address1 < 16) addressString += "0";  // Add leading zero if necessary
    addressString += String(address1, HEX);
    if (address2 < 16) addressString += "0";  // Add leading zero if necessary
    addressString += String(address2, HEX);

    // Print the combined address string
    // if(addressString!="4f4b")
    // {
    Serial.print("Combined Address: 0x");
    Serial.println(addressString);

    // Print the converted addresses and data
    Serial.print("address1: 0x");
    Serial.println(address1, HEX);
    Serial.print("address2: 0x");
    Serial.println(address2, HEX);
    Serial.print("data1: 0x");
    Serial.println(data1, HEX);
    Serial.print("data2: 0x");
    Serial.println(data2, HEX);
    // }

    // Combine d7 and d8 into a single string
    // if(addressString!="4f4b")
    // {

    String dataString = "";
    if (data1 < 16) dataString += "0";  // Add leading zero if necessary
    dataString += String(data1, HEX);
    if (data2 < 16) dataString += "0";  // Add leading zero if necessary
    dataString += String(data2, HEX);

    // Convert the combined data string to an integer
    dataValue = strtol(dataString.c_str(), NULL, 16);



    Serial.print("Data Value: ");
    Serial.println(dataValue);




    if (addressString == "1000") {
      setPressure = dataValue;



    }


    else if (addressString == "1505") {
      setPressureHigh = dataValue;
      Serial.println("setPressureHigh:");
      Serial.println(setPressureHigh);


    } else if (addressString == "1504") {
      setPressureLow = dataValue;
      Serial.println("setPressureLow:");
      Serial.println(setPressureLow);


    }

    else if (addressString == "1500") {
      HighPressure_time = dataValue;
      Serial.println("HighPressure_time:");
      Serial.println(HighPressure_time);

      countdownTimeInt = LowPressure_time + HighPressure_time;
      Serial.println(countdownTimeInt);
    } else if (addressString == "1502") {
      LowPressure_time = dataValue;
      Serial.println("LowPressure_time:");
      Serial.println(LowPressure_time);
      countdownTimeInt = LowPressure_time + HighPressure_time;
      Serial.println(countdownTimeInt);
    }

     else if (addressString == "1180") {
      settime_hr = dataValue;
      Serial.println("settiime_hr:");
      Serial.println(settime_hr);

      // countdownTimeInt = LowPressure_time + HighPressure_time;
      // Serial.println(settiime_hr);
    }

     else if (addressString == "1300") {
      settime_min = dataValue;
      Serial.println("settiime_min:");
      Serial.println(settime_min);
      // countdownTimeInt = LowPressure_time + HighPressure_time;
      // Serial.println(settiime_hr);
    }

       else if (addressString == "1400") {
      settime_sec = dataValue;
      Serial.println("settiime_sec:");
      Serial.println(settime_sec);
      // countdownTimeInt = LowPressure_time + HighPressure_time;
      // Serial.println(settiime_hr);
    }

       else if (addressString == "1600") {
      setdate_yr = dataValue;
      Serial.println("setdate_yr:");
      Serial.println(setdate_yr);
      // countdownTimeInt = LowPressure_time + HighPressure_time;
      // Serial.println(settiime_hr);
    }

      else if (addressString == "1700") {
      setdate_mt = dataValue;
      Serial.println("setdate_mt:");
      Serial.println(setdate_mt);
      // countdownTimeInt = LowPressure_time + HighPressure_time;
      // Serial.println(settiime_hr);
    }

      else if (addressString == "1800") {
      setdate_day = dataValue;
      Serial.println("setdate_day:");
      Serial.println(setdate_day);
      // countdownTimeInt = LowPressure_time + HighPressure_time;
      // Serial.println(settiime_hr);
    }


    //-------------------------------------------------peristalic code comented----------------------------------------------

    //   else if (addressString == "6000") {
    //     // sollutionVolume = dataValue;
    //     // Serial.print("sollutionVolume-------------------------------------------------------------:");
    //     // Serial.println(sollutionVolume);
    //     // currentVolume123456 = sollutionVolume;

    //   }

    //   else if (addressString == "6100") {
    //     // soakTime = dataValue;
    //     // totalTimesoak=soakTime;
    //     // Serial.print("soak time---------------------------------------------------------------------:");
    //     // Serial.println(soakTime);

    //   }


    //   else if (addressString == "6120") {
    //     // instillationRun123456 = dataValue;
    //     // Serial.print(" Instillation run---------------------------------------------------------------------:");
    //     // Serial.println(instillationRun123456);
    //     // currentVolume123456 = sollutionVolume;
    //   }



    separatedData = "";
  }
}



void setfloat(long address, long subaddress, float data) {
  // Define the byte sequence directly
  byte sendBuffer[] = { CMD_HEAD1, CMD_HEAD2, 0x05, CMD_WRITE, address, subaddress, 0x00, 0x00, 0x00, 0x00 };
  // byte  sendBuffer[] = {CMD_HEAD1, CMD_HEAD2, 0x05, CMD_WRITE, address, subaddress, 0x00, 0x00,0x00, 0x00};

  int t = data * 100;

  sendBuffer[6] = highByte(t);
  sendBuffer[7] = lowByte(t);

  // Send the bytes over the _dwinSerial
  Serial2.write(sendBuffer, sizeof(sendBuffer));
}


void setIntegerVP(long address, long subaddress, int data) {
  // byte sendBuffer[] = {0x5A, 0xA5, 0x05, 0x82, (byte)address, (byte)subaddress, 0x00, 0x00, 0x00, 0x00};
  byte sendBuffer[] = { CMD_HEAD1, CMD_HEAD2, 0x05, CMD_WRITE, address, subaddress, 0x00, 0x00, 0x00, 0x00 };


  sendBuffer[6] = highByte(data);
  sendBuffer[7] = lowByte(data);

  Serial2.write(sendBuffer, sizeof(sendBuffer));
  // readDWIN();
}


// Set Text on VP Address
void setText(long address, long subaddress, String textData) {

  int dataLen = textData.length();
  byte startCMD[] = { CMD_HEAD1, CMD_HEAD2, dataLen + 4, CMD_WRITE,
                      address, subaddress, 0x00 };
  byte dataCMD[dataLen];
  textData.getBytes(dataCMD, dataLen + 1);
  byte sendBuffer[7 + dataLen];

  memcpy(sendBuffer, startCMD, sizeof(startCMD));
  memcpy(sendBuffer + 7, dataCMD, sizeof(dataCMD));


  Serial2.write(sendBuffer, sizeof(sendBuffer));
}


// String formatTime(int hours, int minutes, int seconds) {
//   String timeString = "";
//   if (hours < 10) timeString += "0";
//   timeString += hours;
//   timeString += ":";
//   if (minutes < 10) timeString += "0";
//   timeString += minutes;
//   timeString += ":";
//   if (seconds < 10) timeString += "0";
//   timeString += seconds;
//   return timeString;
// }



void setVP(long address, byte data) {
  // byte sendBuffer[] = {0x5A, 0xA5, 0x05, 0x82, (byte)((address >> 8) & 0xFF), (byte)(address & 0xFF), 0x00, data};
  byte sendBuffer[] = { CMD_HEAD1, CMD_HEAD2, 0x05, CMD_WRITE, address, 0x00, 0x00, 0x00, 0x00 };

  Serial2.write(sendBuffer, sizeof(sendBuffer));
  // readDWIN();
}
void start_cont_timer() {
  if (isPaused3) {
    // Resume from pause
    isPaused3 = false;
    isRunning3 = true;
    startTime = millis() - pauseMillis3;  // Adjust the start time
    previousMillis3 = millis();           // Reset the timer
    Serial.println("Timer Resumed");
  } else {
    // Start the timer from scratch
    currentMillis = millis();
    startTime = currentMillis;
    isRunning3 = true;
    isPaused3 = false;
    elapsedMillis3 = 0;
    previousMillis3 = currentMillis;  // Reset the timer
    Serial.println("Timer Started");
  }
}

void pauseTimer_cont() {
  if (isRunning3 && !isPaused3) {
    isPaused3 = true;
    isRunning3 = false;
    pauseMillis3 = elapsedMillis3;  // Store the elapsed time when paused
    Serial.println("Timer Paused");
  }
}

void stopTimer_cont() {
  // Reset all relevant variables
  isRunning3 = false;
  isPaused3 = false;
  elapsedMillis3 = 0;
  pauseMillis3 = 0;
  totalSeconds = 0;
  previousMillis3 = millis();  // Reset to current time
  startTime = 0;               // Reset the start time

  // Reset the timer string to "00:00:00"
  timer_continous = "00:00";

  // Print to Serial to verify
  Serial.println("Timer stopped and reset to 00:00:00");
}

void updateTimer_cont() {
  if (isRunning3 && !isPaused3) {
    currentMillis = millis();
    if (currentMillis - previousMillis3 >= interval3) {
      previousMillis3 = currentMillis;

      // Calculate elapsed time in milliseconds
      elapsedMillis3 = currentMillis - startTime;

      // Convert elapsedMillis3 to HH:MM:SS format
      totalSeconds = elapsedMillis3 / 1000;
      hours = totalSeconds / 3600;
      minutes = (totalSeconds % 3600) / 60;
      seconds = totalSeconds % 60;

      // Format time as HH:MM:SS
      char timeString[9];
      snprintf(timeString, sizeof(timeString), "%02u:%02u:%02u", hours, minutes, seconds);
      timer_continous = String(timeString);

      // Print the timer value to Serial Monitor
      // Serial.print("Timer: ");
      // Serial.println(timer_continous);

      Serial.print("timer_continous_______________________________________________________________________________________:");

      Serial.println(timer_continous);
      // if(absPressure>5 || blockage_state){
            if (!isRunningB ) {
        startBlockageTimer();  // Start the timer if not already running
      }
      blockageTimer();
      // }

      

// if (totalSecondsB < 5 && absPressure > blockageThreshold){
   
// }
      


      if (absPressure < setPressure - 5) {
        if (!isRunningL) {
          startLeakageTimer();  // Start the timer if not already running
        }
        leakagetimer();
      } else {
        stopLeakageTimer();
      }
      setText(0x30, 0x00, String("0" + timer_continous.substring(0, 5)));
    }
  }
}


void startTimer_int() {
  if (isPaused4) {
    // Resume from pause
    isPaused4 = false;
    isRunning4 = true;
    startTime_int_cycle = millis() - pauseMillis4;  // Adjust the start time
    previousMillis4 = millis();                     // Reset the timer
    Serial.println("Timer Resumed");
  } else {
    // Start the timer from scratch
    currentMillis = millis();
    startTime_int_cycle = currentMillis;
    isRunning4 = true;
    isPaused4 = false;
    elapsedMillis4 = 0;
    previousMillis4 = currentMillis;  // Reset the timer
    Serial.println("Timer Started");
  }
}

void pauseTimer_int() {
  if (isRunning4 && !isPaused4) {
    isPaused4 = true;
    isRunning4 = false;
    pauseMillis4 = elapsedMillis4;  // Store the elapsed time when paused
    Serial.println("Timer Paused");
  }
}

void stopTimer_int() {
  // Ensure the timer is properly reset
  isRunning4 = false;   // Set the timer to not running
  isPaused4 = false;    // Make sure it's not paused
  elapsedMillis4 = 0;   // Reset the elapsed time
  pauseMillis4 = 0;     // Clear any paused duration
  previousMillis4 = 0;  // Reset the previous time
  currentMillis = 0;    // Reset the current time to avoid stale values
  totalSeconds4 = 0;
  startTime_int_cycle = 0;  // Reset the start time

  // Optionally update the display or any output to reflect the reset state
  Serial.println("Timer Stopped and Reset");
}

void updateTimer_int() {
  if (isRunning4 && !isPaused4) {
    currentMillis = millis();
    if (currentMillis - previousMillis4 >= interval4) {
      previousMillis4 = currentMillis;

      // Calculate elapsed time in milliseconds
      elapsedMillis4 = currentMillis - startTime_int_cycle;

      // Convert elapsedMillis4 to HH:MM:SS format
      totalSeconds4 = elapsedMillis4 / 1000;
      hours4 = totalSeconds4 / 3600;
      minutes4 = (totalSeconds4 % 3600) / 60;
      seconds4 = totalSeconds4 % 60;

      // Format time as HH:MM:SS
      char timeString4[9];
      snprintf(timeString4, sizeof(timeString4), "%02u:%02u:%02u", hours4, minutes4, seconds4);
      timer_intermittent = String(timeString4);

      // Display timer value
      Serial.print("timer_intermittent_______________________________________________________________________________________:");

      Serial.println(timer_intermittent);
      if (!isRunningB) {
        startBlockageTimer();  // Start the timer if not already running
      }
      blockageTimer();

      //        if (absPressure < 230) {
      //   blockage_state = 0;  // blockage feedback
      //   stopBlockageTimer();
      // }

      if (absPressure < setPressure_int_cycle - 5) {
        if (!isRunningL) {
          startLeakageTimer();  // Start the timer if not already running
        }
        leakagetimer();
      } else {
        stopLeakageTimer();
      }

      setText(0x70, 0x00, String("0" + timer_intermittent.substring(0, 5)));
    }
  }
}


// Start the timer
void startTimer_Int_cycle() {
  if (isPaused5) {
    // Resume from pause
    isPaused5 = false;
    isRunning5 = true;

    // Adjust start time by accounting for paused duration
    startTimeMillis5 = millis() - pausedMillis5;
    Serial.println("Timer Resumed");
  } else {
    // Start from the beginning
    startTimeMillis5 = millis();
    elapsedMillis5 = 0;
    pausedMillis5 = 0;
    isRunning5 = true;
    isPaused5 = false;
    Serial.println("Timer Started");
  }
}

// Pause the timer
void pauseTimer_Int_cycle() {
  if (isRunning5 && !isPaused5) {
    isPaused5 = true;
    isRunning5 = false;

    // Store elapsed time when paused
    pausedMillis5 = millis() - startTimeMillis5;
    Serial.println("Timer Paused");
  }
}

// Stop the timer
void stopTimer_Int_cycle() {
  isRunning5 = false;
  isPaused5 = false;
  elapsedMillis5 = 0;
  pausedMillis5 = 0;
  minutes5 = 0;
  String_timer_Int_cycle = "00";
  Serial.println("Timer Stopped and Reset");
}

// Update the timer
void updateTimer_Int_cycle() {
  if (isRunning5 && !isPaused5) {
    currentMillis = millis();
    if (currentMillis - previousMillis5 >= interval5) {
      previousMillis5 = currentMillis;

      // Calculate elapsed time
      elapsedMillis5 = currentMillis - startTimeMillis5;

      // Convert elapsed time to minutes
      unsigned long totalSeconds5 = elapsedMillis5 / 1000;
      minutes5 = (totalSeconds5 % 3600) / 60;

      // Reset to zero after reaching combined high and low pressure time
      if (minutes5 >= HighPressure_time + LowPressure_time) {
        minutes5 = 0;
        startTimeMillis5 = currentMillis;  // Reset start time
      }

      // Format time as MM
      char timeString5[3];
      snprintf(timeString5, sizeof(timeString5), "%02u", minutes5);
      String_timer_Int_cycle = String(timeString5);

      // Print the timer value
      Serial.print("Minutes: ");
      Serial.println(String_timer_Int_cycle);
    }
  }
}


void Time_date() {


  DateTime now = rtc.now();

  // Create date and time strings
  dateStr = String(now.year(), DEC) + '/' + String(now.month(), DEC) + '/' + String(now.day(), DEC);


  timeStr = String(now.hour(), DEC) + ':' + String(now.minute(), DEC);

  setText(0x36, 0x00, String("0" + timeStr + " "));  //00


  setText(0x35, 0x00, String("0" + dateStr));  //0
}


void continousmode() {



  if (Continous_Mode == 1) {
    runState = 1;
    Device_mode = 1;
    Intermittent_Mode = 0;
    Variable_Mode = 0;


    Run_icon();  // runnig icon update in display


    MaintainPressure();

  }


  else if (Intermittent_Mode == 0 && Continous_Mode == 0 && Variable_Mode == 0) {

    runState = -1;

    stopTimer_cont();
    pauseLeakageTimer();

    // digitalWrite(Solenoid_valve,HIGH);

    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
    analogWrite(pwmPin, 0);  // Stop
    // if (!valaveTriggered) {
    //     digitalWrite(motorPin1, HIGH);
    //     digitalWrite(motorPin2, LOW);
    //     delay(1000);
    //     valaveTriggered = 1;
    //   }

    //   else {
    //     digitalWrite(motorPin1, LOW);
    //     digitalWrite(motorPin2, LOW);
    //   }

    // Serial.println("stopped");
  }

  else if (Continous_Mode == 4) {

    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
    analogWrite(pwmPin, 0);  // Stop
    pauseTimer_cont();

    stopBlockageTimer();
    stopLeakageTimer();
  }
}

void intermittentmode() {

  Serial.println(minutes4_int);
  if (Intermittent_Mode == 1) {
    runState = 2;
    Device_mode = 2;

    Continous_Mode = 0;
    Variable_Mode = 0;

    currentMillisInt = millis();
    Serial.print("int cycle  time: ----------------------------------------------------------");
    Serial.print(minutes4_int);
    Serial.println(" minute(s).");


    Run_icon();



    if (high_presure_time_state == 0) {
      if ((minutes5 < HighPressure_time)) {

        MaintainPressure_intermittentHigh();
        // Serial.println("highrunning++++++++++++++++++++++++");
        // highrun = 1;
        if (pressuresetHigh) {
          if (!isRunning5) {
            startTimer_Int_cycle();  // Start the timer if not already running
          }
          updateTimer_Int_cycle();
        }
      }

      else {

        if (absPressure > setPressureLow) {

          digitalWrite(in1Pin, LOW);
          digitalWrite(in2Pin, LOW);
          analogWrite(pwmPin, 0);  // Stop

          // valveON();

          // delay(1000);

          if (!valve_state) {
            // valveON();
            if (absPressure > setPressureLow + 15) {
              digitalWrite(motorPin1, HIGH);
              digitalWrite(motorPin2, LOW);
              // delay(1000);
            } else {
              valve_state = 1;
            }
          }

          else {
            valveOFF();
          }

          // if(absPressure>setPressureLow+20){
          //   valve_state = 0;

          // }


          // delay(50);
          //  valveOFF();
          // delay(50);
          // pauseTimer_Int_cycle();
          // pauseTimer_int();

          pauseTimer_Int_cycle();
          // stopTimer_Int_cycle();
        }

        else if (absPressure < setPressureLow) {
          valveOFF();

          high_presure_time_state = 1;  // jumps to intermitten low maintain pressure
        }
      }

    }

    else if (high_presure_time_state == 1 && minutes5 > 0) {

      // resumeTimer_Int_cycle();
      if (!isRunning5) {
        startTimer_Int_cycle();  // Start the timer if not already running
      }
      updateTimer_Int_cycle();

      // startTimer_Int_cycle

      MaintainPressure_intermittentLow();
      // Serial.println("low running -----------------------------------------------");
    }


    if (high_presure_time_state == 1 && minutes5 == 0) {


      minutes4_int = LowPressure_time + HighPressure_time;
      countdownRunningInt = true;
      Serial.println("time restart##########################################################################################");
      previousMillisInt = millis();  // Reset the timer
      high_presure_time_state = 0;
      delay(10);
    }

    if (absPressure > setPressureLow + 10) {
      high_presure_time_state = 0;  //for activating the valve again and again until it reaches thesetlo pressure
    }


  }

  else if (Intermittent_Mode == 0 && Continous_Mode == 0 && Variable_Mode == 0) {

    runState = -2;
    // if(currentPage!=11)
    // {
    // pauseTimer_int();
    stopTimer_int();

    // }

    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
    analogWrite(pwmPin, 0);  // Stop
  }

  else if (Intermittent_Mode == 4) {

    pauseTimer_int();
    pauseTimer_Int_cycle();
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
    analogWrite(pwmPin, 0);  // Stop

    stopBlockageTimer();
    stopLeakageTimer();
  }
}






// void variablepressuremode() {


//   if (Variable_Mode == 1) {
//     runState = 3;

//     Device_mode = 3;

//     Continous_Mode = 0;
//     Intermittent_Mode = 0;
//     if (variable_state == 0) {
//       // Intermittent_state=0;
//       MaintainPressure();

//     } else if (variable_state == 1) {
//       digitalWrite(in1Pin, LOW);
//       digitalWrite(in2Pin, LOW);
//       analogWrite(pwmPin, 0);  // Stop
//     }

//     if (absPressure < variable_pressure) {
//       variable_state = 0;
//     }



//     if (absPressure > setPressure) {
//       variable_state = 1;
//     }

//   }

//   else if (Intermittent_Mode == 0 && Continous_Mode == 0 && Variable_Mode == 0) {

//     runState = -3;
//     // pauseTimer_Int_cycle();
//     digitalWrite(in1Pin, LOW);
//     digitalWrite(in2Pin, LOW);
//     analogWrite(pwmPin, 0);  // Stop

//     // Serial.println("stopped");
//   }
// }

void setPage(byte page) {
  // Construct the command buffer
  byte sendBuffer[] = { CMD_HEAD1, CMD_HEAD2, 0x07, CMD_WRITE, 0x00, 0x84, 0x5A, 0x01, 0x00, page };
  // Send the command to the DWIN display
  Serial2.write(sendBuffer, sizeof(sendBuffer));
  // Read the response from the DWIN display (assuming readDWIN() is implemented)
  // readDWIN();
}


byte getPage() {
  byte sendBuffer[] = { CMD_HEAD1, CMD_HEAD2, 0x04, CMD_READ, 0x00, 0x14, 0x01 };
  Serial2.write(sendBuffer, sizeof(sendBuffer));
  return readCMDLastByte();
}

byte readCMDLastByte() {
  byte lastByte = -1;
  unsigned long startTime = millis();  // Start time for Timeout
  while ((millis() - startTime < CMD_READ_TIMEOUT)) {
    while (Serial2.available() > 0) {
      lastByte = Serial2.read();
    }
  }
  return lastByte;
}



void valveON() {


  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);
  // delay(1800);
  // valve_state = 1;
}

void valveOFF() {
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
}


void MotorRunL() {
  // startTimer = millis();
  // duration = time1;

  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, HIGH);
  analogWrite(pwmPin, 120);  //  speed

  delay(100);
  analogWrite(pwmPin, 0);  //  speed
  delay(25);
}



void MotorRunveryLow() {
  // startTimer = millis();
  // duration = time1;

  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, HIGH);

  analogWrite(pwmPin, 130);  //  speed

  delay(100);
  analogWrite(pwmPin, 100);  //  speed
  delay(25);
}

void MotorRunveryLowst() {
  // startTimer = millis();
  // duration = time1;

  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, LOW);

  analogWrite(pwmPin, 120);  //  speed

  delay(100);
  analogWrite(pwmPin, 0);  //  speed
  delay(25);
}

void MotorRunH() {
  // startTimer = millis();
  // duration = time1;

  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, HIGH);





  analogWrite(pwmPin, 150);  //  speed

  delay(100);
  analogWrite(pwmPin, 0);  //  speed
  delay(25);
}

void initialRun() {

  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, HIGH);


  // analogWrite(pwmPin, 200);  //  speed
  if (setPressure < 30) {
    // Code for setPressure between 0 and 29 (No analogWrite actions)
  } else if (setPressure >= 30 && setPressure < 40) {
    analogWrite(pwmPin, 240);  // speed
    // delay(10);
    // analogWrite(pwmPin, 160);  // speed
    // delay(10);
  } else if (setPressure >= 40 && setPressure < 50) {
    analogWrite(pwmPin, 240);  // speed
    // delay(10);
    // analogWrite(pwmPin, 180);  // speed
    // delay(10);
  } 
  else if (setPressure >= 50 && setPressure < 100) {
    // analogWrite(pwmPin, 220);  // speed
    // delay(10);
    // analogWrite(pwmPin, 210);  // speed
    // delay(10);

      analogWrite(pwmPin, 240);  // speed
    // delay(10);
    // analogWrite(pwmPin, 190);  // speed
    // delay(10);
  }
  else if (setPressure >= 100 && setPressure < 170) {
    // analogWrite(pwmPin, 220);  // speed
    // delay(10);
    // analogWrite(pwmPin, 210);  // speed
    // delay(10);

      analogWrite(pwmPin, 255);  // speed
    // delay(10);
    // analogWrite(pwmPin, 230);  // speed
    // delay(10);
  } else if (setPressure >= 170 && setPressure <= 200) {
    // analogWrite(pwmPin, 230);  // speed
    // delay(10);
    // analogWrite(pwmPin, 220);  // speed
    // delay(10);


        analogWrite(pwmPin, 255);  // speed
    // delay(10);
    // analogWrite(pwmPin, 240);  // speed
    // delay(10);
  }
}

void finalRun() {

  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, HIGH);


  // analogWrite(pwmPin, 200);  //  speed


  if (setPressure > 120 && setPressure <= 170) {
    // Code for setPressure between 0 and 9
    analogWrite(pwmPin, 255);  //  speed
    // delay(500);
    // analogWrite(pwmPin, 0);  //  speed
    // delay(10);
  }

  else if (setPressure > 170 && setPressure <= 200) {
    // Code for setPressure between 0 and 9
    analogWrite(pwmPin, 255);  //  speed
    // delay(500);
    // analogWrite(pwmPin, );  //  speed
    // delay(10);
  }

  else if (setPressure <= 80) {

    analogWrite(pwmPin, 240);  //  speed
    // delay(500);
    // analogWrite(pwmPin, 0);  //  speed
    // delay(10);
  }

  
  else if (setPressure <= 120 && setPressure > 80) {

    analogWrite(pwmPin, 240);  //  speed
    // delay(500);
    // analogWrite(pwmPin, 0);  //  speed
    // delay(10);
  }
}

void initialRun_low() {

  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, HIGH);



  // analogWrite(pwmPin, 200);  //  speed
  if (setPressureLow < 30) {
    // Code for setPressure between 0 and 29 (No analogWrite actions)
  } else if (setPressureLow >= 30 && setPressureLow < 40) {
    analogWrite(pwmPin, 240);  // speed
    // delay(10);
    // analogWrite(pwmPin, 160);  // speed
    // delay(10);
  } else if (setPressureLow >= 40 && setPressureLow < 50) {
 analogWrite(pwmPin, 240);  // spee
  } 
  else if (setPressureLow >= 50 && setPressureLow < 100) {
    // analogWrite(pwmPin, 220);  // speed
    // delay(10);
    // analogWrite(pwmPin, 210);  // speed
    // delay(10);

     analogWrite(pwmPin, 240);  // spee
  }
  else if (setPressureLow >= 100 && setPressureLow < 170) {
    // analogWrite(pwmPin, 220);  // speed
    // delay(10);
    // analogWrite(pwmPin, 210);  // speed
    // delay(10);
 analogWrite(pwmPin, 255);  // spee

  } else if (setPressureLow >= 170 && setPressureLow <= 200) {
    // analogWrite(pwmPin, 230);  // speed
    // delay(10);
    // analogWrite(pwmPin, 220);  // speed
    // delay(10);

         analogWrite(pwmPin, 255);  // speed
    // delay(10);
    // analogWrite(pwmPin, 240);  // speed
    // delay(10);
  }
}


void finalRun_low() {

  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, HIGH);





  if (setPressureLow > 120 && setPressureLow <= 170) {
    // Code for setPressure between 0 and 9
    analogWrite(pwmPin, 255);  //  speed
    // delay(500);
    // analogWrite(pwmPin, 0);  //  speed
    // delay(10);
  }

  else if (setPressureLow > 170 && setPressureLow <= 200) {
    // Code for setPressure between 0 and 9
    analogWrite(pwmPin, 255);  //  speed
    // delay(500);
    // analogWrite(pwmPin, 0);  //  speed
    // delay(10);
  }

  else if (setPressureLow <= 80) {

    analogWrite(pwmPin, 240);  //  speed
    // delay(500);
    // analogWrite(pwmPin, 0);  //  speed
    // delay(10);
  }

  
  else if (setPressureLow <= 120 && setPressureLow > 80) {

    analogWrite(pwmPin, 240);  //  speed
    // delay(500);
    // analogWrite(pwmPin, 0);  //  speed
    // delay(10);
  }
}

void initialRun_high() {

  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, HIGH);

  // analogWrite(pwmPin, 200);  //  speed
  if (setPressureHigh < 30) {
    // Code for setPressure between 0 and 29 (No analogWrite actions)
  } else if (setPressureHigh >= 30 && setPressureHigh < 40) {
 analogWrite(pwmPin, 240);  //  speed
  }  else if (setPressureHigh >= 40 && setPressureHigh < 50) {
 analogWrite(pwmPin, 240);  //  speed
  } 
  else if (setPressureHigh >= 50 && setPressureHigh < 100) {
 analogWrite(pwmPin, 240);  //  speed
  }
  else if (setPressureHigh >= 100 && setPressureHigh < 170) {
 analogWrite(pwmPin, 255);  //  speed
  } else if (setPressureHigh >= 170 && setPressureHigh <= 200) {
    // analogWrite(pwmPin, 230);  // speed
    // delay(10);
    // analogWrite(pwmPin, 220);  // speed
    // delay(10);

        analogWrite(pwmPin, 255);  // speed
 
  }
}


void finalRun_high() {

  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, HIGH);



  if (setPressureHigh > 120 && setPressureHigh <= 170) {
    // Code for setPressure between 0 and 9
    analogWrite(pwmPin, 255);  //  speed

  }

  else if (setPressureHigh > 170 && setPressureHigh <= 200) {
    // Code for setPressure between 0 and 9
    analogWrite(pwmPin, 255);  //  speed

  }

  else if (setPressureHigh <= 80) {

    analogWrite(pwmPin, 240);  //  speed
    // delay(500);
    // analogWrite(pwmPin, 0);  //  speed
    // delay(10);
  }

  
  else if (setPressureHigh <= 120 && setPressureHigh > 80) {

    analogWrite(pwmPin, 240);  //  speed
    // delay(500);
    // analogWrite(pwmPin, 0);  //  speed
    // delay(10);
  }
}

void MaintainPressure() {
  setPressure_int_cycle_state = 2;

  if (setPressure > 10) {
    if (absPressure < setPressure && !pressureset && absPressure < setPressure * 0.95) {


      initialRun();

    }

    else if (absPressure < setPressure && !pressureset && absPressure >= setPressure * 0.95) {

      finalRun();
      // initialRun();

    }

    else if (absPressure < setPressure - 3 && pressureset) {


      //  initialRun();
      finalRun();
      // initialRun();



      //           analogWrite(pwmPin, 180);  //  speed
      // // delay(80);
      // // analogWrite(pwmPin, 0);  //  speed
      // // delay(25);

    }

    else {
      // valveOFF();
      // if(absPressure>setPressure+6)
      // {

      //      valveON();
      //       delay(8);
      //        valveOFF();
      //       delay(8);
      // }

      digitalWrite(in1Pin, LOW);
      digitalWrite(in2Pin, LOW);
      analogWrite(pwmPin, 0);  // Stop

      //MotorRunveryLowst();
    }
  }

  else if (setPressure <= 10) {



    if (absPressure < setPressure) {


      initialRun();

    }


    else {
      // valveOFF();
      // if(absPressure>setPressure+6)
      // {

      //      valveON();
      //       delay(8);
      //        valveOFF();
      //       delay(8);
      // }

      digitalWrite(in1Pin, LOW);
      digitalWrite(in2Pin, LOW);
      analogWrite(pwmPin, 0);  // Stop

      //MotorRunveryLowst();
    }
  }





  if (absPressure >= setPressure) {


    pressureset = true;

  }

  else if (absPressure < setPressure - 20) {


    pressureset = false;
  }
}


void MaintainPressure_intermittentHigh() {

  setIntegerVP(0x42, 0x00, 0);
  setPressure_int_cycle = setPressureHigh;
  setPressure_int_cycle_state = 1;

  if (absPressure < setPressureHigh && !pressuresetHigh && absPressure < setPressureHigh * 0.95) {


    initialRun_high();

  }

  else if (absPressure < setPressureHigh && !pressuresetHigh && absPressure >= setPressureHigh * 0.95) {

    finalRun_high();
    // initialRun();

  }

  else if (absPressure < setPressureHigh - 3 && pressuresetHigh) {


    //  initialRun();
    finalRun_high();
    // initialRun();

  }

  else {
    // valveOFF();
    // if(absPressure>setPressure+6)
    // {

    //      valveON();
    //       delay(8);
    //        valveOFF();
    //       delay(8);
    // }

    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
    analogWrite(pwmPin, 0);  // Stop

    //MotorRunveryLowst();
  }



  if (absPressure >= setPressureHigh) {


    pressuresetHigh = true;

  }

  else if (absPressure < setPressureHigh - 20) {


    pressuresetHigh = false;
  }
}


void MaintainPressure_intermittentLow() {
  setIntegerVP(0x42, 0x00, 1);
  setPressure_int_cycle = setPressureLow;
  setPressure_int_cycle_state = 0;

  if (absPressure < setPressureLow && !pressuresetLow && absPressure < setPressureLow * 0.95) {


    initialRun_low();

  }

  else if (absPressure < setPressureLow && !pressuresetLow && absPressure >= setPressureLow * 0.95) {

    finalRun_low();
    // initialRun();

  }

  else if (absPressure < setPressureLow - 3 && pressuresetLow) {


    //  initialRun();
    finalRun_low();
    // initialRun();

  }

  else {
    // valveOFF();
    // if(absPressure>setPressure+6)
    // {

    //      valveON();
    //       delay(8);
    //        valveOFF();
    //       delay(8);
    // }

    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
    analogWrite(pwmPin, 0);  // Stop

    //MotorRunveryLowst();
  }


  if (absPressure >= setPressureLow) {


    pressuresetLow = true;

  }

  else if (absPressure < setPressureLow - 20) {


    pressuresetLow = false;
  }
}



void MaintainPressureIntermittent() {


  if (absPressure < setPressure * 0.9) {


    // initialRun();
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);


    analogWrite(pwmPin, 130);  //  speed
    delay(100);
    analogWrite(pwmPin, 0);  //  speed
    delay(50);
    valaveTriggered = 0;

    // digitalWrite(motorPin1, LOW);
    // digitalWrite(motorPin2, LOW);
  }

  else if (absPressure > setPressure * 0.9 && absPressure < setPressure && !pressureset)

  {
    if (setPressure <= 150) {
      MotorRunL();
    }

    else if (setPressure > 150) {
      MotorRunH();
    }




  }

  else if (absPressure > setPressure * 0.9 && absPressure < setPressure - 3 && pressureset) {
    // MotorRun();
    if (setPressure <= 150) {
      MotorRunL();
    }

    else if (setPressure > 150) {
      MotorRunH();
    }
  }
  if (absPressure >= setPressure) {


    pressureset = true;
  }

  else if (absPressure < setPressure - 10) {


    pressureset = false;

  }

  else {
    // valveOFF();

    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
    analogWrite(pwmPin, 0);  // Stop
  }
}



// Start the leakage timer
void startLeakageTimer() {
  if (isPausedL) {
    // Resume from pause
    isPausedL = false;
    isRunningL = true;

    // Adjust the start time to account for the paused duration
    startTimeMillisL = millis() - pausedMillisL;
    Serial.println("Leakage Timer Resumed");
  } else {
    // Start from scratch
    startTimeMillisL = millis();
    elapsedMillisL = 0;
    pausedMillisL = 0;
    isRunningL = true;
    isPausedL = false;
    Serial.println("Leakage Timer Started");
  }
}

// Pause the leakage timer
void pauseLeakageTimer() {
  if (isRunningL && !isPausedL) {
    isPausedL = true;
    isRunningL = false;

    // Save the elapsed time when paused
    pausedMillisL = millis() - startTimeMillisL;
    Serial.println("Leakage Timer Paused");
  }
}

// Stop the leakage timer
void stopLeakageTimer() {
  isRunningL = false;
  isPausedL = false;
  elapsedMillisL = 0;
  pausedMillisL = 0;
  totalSecondsL = 0;

  leakagetest = 0;               // Reset leakage test state
  setIntegerVP(0x41, 0x00, 25);  // Clear leakage alert icon
  Serial.println("Leakage Timer Stopped and Reset");
}

// Update the leakage timer
void leakagetimer() {
  if (isRunningL && !isPausedL) {
    currentMillisL = millis();
    if (currentMillisL - previousMillisL >= intervalL) {
      previousMillisL = currentMillisL;

      // Calculate elapsed time
      elapsedMillisL = currentMillisL - startTimeMillisL;

      // Convert elapsed time to seconds
      totalSecondsL = elapsedMillisL / 1000;

      Serial.print("Total Seconds: ");
      Serial.println(totalSecondsL);

      // Leakage detection logic
      if (totalSecondsL > 119 && absPressure < setPressure * 0.9) {
        runState = -1;
        leakagetest = 1;               // Leakage detected
        setIntegerVP(0x41, 0x00, 24);  // Show leakage alert icon
        Serial.println("Leakage Detected");
      }

      // Reset if pressure exceeds set point
      if (absPressure > setPressure) {
        leakagetest = 0;  // Clear leakage state
        stopLeakageTimer();
      }
    }
  }
}


// Start the blockage timer
void startBlockageTimer() {
  if (isPausedB) {
    // Resume from pause
    isPausedB = false;
    isRunningB = true;

    // Adjust the start time to account for the paused duration
    startTimeMillisB = millis() - pausedMillisB;
    Serial.println("Blockage Timer Resumed");
  } else {
    // Start from scratch
    startTimeMillisB = millis();
    elapsedMillisB = 0;
    pausedMillisB = 0;
    isRunningB = true;
    isPausedB = false;
    Serial.println("Blockage Timer Started");
  }
}

// Pause the blockage timer
void pauseBlockageTimer() {
  if (isRunningB && !isPausedB) {
    isPausedB = true;
    isRunningB = false;

    // Save the elapsed time when paused
    pausedMillisB = millis() - startTimeMillisB;
    Serial.println("Blockage Timer Paused");
  }
}

// Stop the blockage timer
void stopBlockageTimer() {
  isRunningB = false;
  isPausedB = false;
  elapsedMillisB = 0;
  pausedMillisB = 0;
  totalSecondsB = 0;

  blockage_state = 0;            // Reset blockage state
  setIntegerVP(0x41, 0x00, 25);  // Clear blockage alert icon
  Serial.println("Blockage Timer Stopped and Reset");
}

// Update the blockage timer
void blockageTimer() {
  if (isRunningB && !isPausedB) {
    currentMillisB = millis();
    if (currentMillisB - previousMillisB >= intervalB) {
      previousMillisB = currentMillisB;

      // Calculate elapsed time
      elapsedMillisB = currentMillisB - startTimeMillisB;

      // Convert elapsed time to seconds
      totalSecondsB = elapsedMillisB / 1000;

      Serial.print("Total Seconds: ");
      Serial.println(totalSecondsB);


 
      // Blockage detection logic
      if ( totalSeconds < 3 && absPressure > blockageThreshold &&Continous_Mode) {
        // if (totalSecondsBabsPressure > blockageThreshold){
        blockage_state = 1;            // Blockage detected
        setIntegerVP(0x41, 0x00, 23);  // Show blockage alert icon
        Serial.println("Blockage Detected!");
        }

             else  if ( totalSeconds4 < 3 && absPressure > blockageThreshold &&Intermittent_Mode) {
        // if (totalSecondsBabsPressure > blockageThreshold){
        blockage_state = 1;            // Blockage detected
        setIntegerVP(0x41, 0x00, 23);  // Show blockage alert icon
        Serial.println("Blockage Detected!");
        }

        // else{

        // blockage_state =0 ;            // Blockage detected

        // }


        else if(blockage_state && totalSeconds > 5 && absPressure < blockageThreshold){ // cheking if the blockage is cleared

        stopBlockageTimer();           // Stop timer if blockage is cleared
        setIntegerVP(0x41, 0x00, 25);  // Clear blockage alert icon
        blockage_state =0 ;            // Blockage detected


        }

           else if(blockage_state && totalSeconds4 > 5 && absPressure < blockageThreshold){ // cheking if the blockage is cleared

        stopBlockageTimer();           // Stop timer if blockage is cleared
        setIntegerVP(0x41, 0x00, 25);  // Clear blockage alert icon
        blockage_state =0 ;            // Blockage detected


        }

  //             if (blockage_state && absPressure < blockageThreshold) {
  //       blockage_state = 0;            // No blockage
  //       stopBlockageTimer();           // Stop timer if blockage is cleared
  //       setIntegerVP(0x41, 0x00, 25);  // Clear blockage alert icon
  //       Serial.println("Blockage Cleared");
  //     }

  //     if(absPressure==0)
  //     {
  // stopBlockageTimer();           // Stop timer if blockage is cleared
  //       setIntegerVP(0x41, 0x00, 25);  // Clear blockage alert icon
  //       blockage_state = 0;            // No blockage

  //     }




      // }

      // Reset blockage detection if pressure goes below threshold

    }
  }
}








void Run_icon() {

  setIntegerVP(0x48, 0x50, 21);
  delay(100);

  setIntegerVP(0x48, 0x50, 19);
  delay(100);

  setIntegerVP(0x48, 0x50, 17);

  delay(100);

  setIntegerVP(0x48, 0x50, 15);
  delay(100);

  setIntegerVP(0x48, 0x50, 13);
  delay(10);
}
void release_pressure() {
  if (!valve_state) {
    // valveON();
    if (absPressure > 10) {
      digitalWrite(motorPin1, HIGH);
      digitalWrite(motorPin2, LOW);
      // delay(1000);
    } else {
      valve_state = 1;
    }
  }

  else {
    valveOFF();
  }

  if (absPressure > 10) {
    valve_state = 0;
  }
}

void updatePressureDisplay(int absPressureparm) {
  if (absPressureparm >= 0 && absPressureparm < 5)
    setIntegerVP(0x10, 0x01, 5);
  else if (absPressureparm >= 5 && absPressureparm < 10)
    setIntegerVP(0x10, 0x01, 10);
  else if (absPressureparm >= 10 && absPressureparm < 15)
    setIntegerVP(0x10, 0x01, 15);
  else if (absPressureparm >= 15 && absPressureparm < 20)
    setIntegerVP(0x10, 0x01, 20);
  else if (absPressureparm >= 20 && absPressureparm < 25)
    setIntegerVP(0x10, 0x01, 25);
  else if (absPressureparm >= 25 && absPressureparm < 30)
    setIntegerVP(0x10, 0x01, 30);
  else if (absPressureparm >= 30 && absPressureparm < 35)
    setIntegerVP(0x10, 0x01, 35);
  else if (absPressureparm >= 35 && absPressureparm < 40)
    setIntegerVP(0x10, 0x01, 40);
  else if (absPressureparm >= 40 && absPressureparm < 45)
    setIntegerVP(0x10, 0x01, 45);
  else if (absPressureparm >= 45 && absPressureparm < 50)
    setIntegerVP(0x10, 0x01, 50);
  else if (absPressureparm >= 50 && absPressureparm < 55)
    setIntegerVP(0x10, 0x01, 55);
  else if (absPressureparm >= 55 && absPressureparm < 60)
    setIntegerVP(0x10, 0x01, 60);
  else if (absPressureparm >= 60 && absPressureparm < 65)
    setIntegerVP(0x10, 0x01, 65);
  else if (absPressureparm >= 65 && absPressureparm < 70)
    setIntegerVP(0x10, 0x01, 70);
  else if (absPressureparm >= 70 && absPressureparm < 75)
    setIntegerVP(0x10, 0x01, 75);
  else if (absPressureparm >= 75 && absPressureparm < 80)
    setIntegerVP(0x10, 0x01, 80);
  else if (absPressureparm >= 80 && absPressureparm < 85)
    setIntegerVP(0x10, 0x01, 85);
  else if (absPressureparm >= 85 && absPressureparm < 90)
    setIntegerVP(0x10, 0x01, 90);
  else if (absPressureparm >= 90 && absPressureparm < 95)
    setIntegerVP(0x10, 0x01, 95);
  else if (absPressureparm >= 95 && absPressureparm < 100)
    setIntegerVP(0x10, 0x01, 100);
  else if (absPressureparm >= 100 && absPressureparm < 105)
    setIntegerVP(0x10, 0x01, 105);
  else if (absPressureparm >= 105 && absPressureparm < 110)
    setIntegerVP(0x10, 0x01, 110);
  else if (absPressureparm >= 110 && absPressureparm < 115)
    setIntegerVP(0x10, 0x01, 115);
  else if (absPressureparm >= 115 && absPressureparm < 120)
    setIntegerVP(0x10, 0x01, 120);
  else if (absPressureparm >= 120 && absPressureparm < 125)
    setIntegerVP(0x10, 0x01, 125);
  else if (absPressureparm >= 125 && absPressureparm < 130)
    setIntegerVP(0x10, 0x01, 130);
  else if (absPressureparm >= 130 && absPressureparm < 135)
    setIntegerVP(0x10, 0x01, 135);
  else if (absPressureparm >= 135 && absPressureparm < 140)
    setIntegerVP(0x10, 0x01, 140);
  else if (absPressureparm >= 140 && absPressureparm < 145)
    setIntegerVP(0x10, 0x01, 145);
  else if (absPressureparm >= 145 && absPressureparm < 150)
    setIntegerVP(0x10, 0x01, 150);
  else if (absPressureparm >= 150 && absPressureparm < 155)
    setIntegerVP(0x10, 0x01, 155);
  else if (absPressureparm >= 155 && absPressureparm <= 160)
    setIntegerVP(0x10, 0x01, 160);
  else if (absPressureparm > 160 && absPressureparm <= 165)
    setIntegerVP(0x10, 0x01, 165);
  else if (absPressureparm >= 165 && absPressureparm < 170)
    setIntegerVP(0x10, 0x01, 170);
  else if (absPressureparm >= 170 && absPressureparm < 175)
    setIntegerVP(0x10, 0x01, 175);
  else if (absPressureparm >= 175 && absPressureparm < 180)
    setIntegerVP(0x10, 0x01, 180);
  else if (absPressureparm >= 180 && absPressureparm < 185)
    setIntegerVP(0x10, 0x01, 185);
  else if (absPressureparm >= 185 && absPressureparm < 190)
    setIntegerVP(0x10, 0x01, 190);
  else if (absPressureparm >= 190 && absPressureparm < 195)
    setIntegerVP(0x10, 0x01, 195);
  else if (absPressureparm >= 195 && absPressureparm <= 200)
    setIntegerVP(0x10, 0x01, 200);


     else if (absPressureparm >220)
    setIntegerVP(0x10, 0x01, absPressureparm);
}


void battery_icon() {

  // setText(0x25, 0x00,"0"+String(in_voltage2));


if(smoothedVoltage<=10.5 && in_voltage1<10)
{

    setIntegerVP(0x46, 0x60, 3);

          digitalWrite(buzzer_pin, 1);
      delay(100);
      digitalWrite(buzzer_pin, 0);
      delay(100);


}


else if(smoothedVoltage<=10.5 && in_voltage1>10)
{

    setIntegerVP(0x46, 0x60, 1);


}

else if(smoothedVoltage>10.5 &&in_voltage1<10){

    setIntegerVP(0x46, 0x60, 2);


}


else if(smoothedVoltage>10.5 &&in_voltage1>10){

    setIntegerVP(0x46, 0x60, 1);


}



  // if (in_voltage1 >= 10) {



  //   setIntegerVP(0x46, 0x56, 29);


  // } 
  
  
  // else {

  //   setIntegerVP(0x46, 0x56, 30);
  // }



  if (smoothedVoltage > 12.5) {

    //       //   // setfloat(0x70, 0x00, 5);
    setIntegerVP(0x40, 0x08, 10);
    Serial.println("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$.................5");

  }


  else if (smoothedVoltage <= 12.5 && smoothedVoltage > 11.5) {

    //       //   // setfloat(0x70, 0x00, 5);
    setIntegerVP(0x40, 0x08, 9);
    Serial.println("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$.................4");

  }

  else if (smoothedVoltage <= 11.5 && smoothedVoltage > 11) {

    //       //   // setfloat(0x70, 0x00, 5);
    setIntegerVP(0x40, 0x08, 8);
    Serial.println("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$.................3");


  }

  else if (smoothedVoltage <= 11 && smoothedVoltage > 10) {

    //       //   // setfloat(0x70, 0x00, 5);
    setIntegerVP(0x40, 0x08, 7);
    Serial.println("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$.................2");


  }

  else if (smoothedVoltage <= 10) {

    //       //   // setfloat(0x70, 0x00, 2);
    setIntegerVP(0x40, 0x08, 6);
    Serial.println("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$.................1");
  }
  //       // }
  // }
}
