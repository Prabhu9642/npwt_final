// only leakage and blockage indications pending
//10/22/2024
                                          
#include <Wire.h>
#include <Arduino.h>
#include <SparkFun_MicroPressure.h>
unsigned long lastUpdateTime = 0;  // Stores the last time the timer was updated
const long timerInterval = 1000;   // Interval at which to run the code (in milliseconds)

#include "RTClib.h"

RTC_DS3231 rtc;
unsigned long startMillissoak;       // Start time
unsigned long currentMillissoak;     // Current time
unsigned long intervalssoak = 1000;  // 1-second intervalna
int totalTimesoak;                   // Total time in seconds
char daysOfTheWeek[7][12] = { "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday" };
#define ANALOG_IN_PIN2 PB1
String str;
float adc_voltage2 = 0.0;
float in_voltage2 = 0.0;

// Floats for resistor values in divider (in ohms)
float R1 = 100000.0;
float R2 = 10000.0;

float ref_voltage = 3.3;
int Device_mode = 0;
// Integer for ADC value
int adc_value = 0;
int adc_value2 = 0;

#define CMD_HEAD1 0x5A
#define CMD_HEAD2 0xA5
#define CMD_WRITE 0x82
#define CMD_READ 0x83

#define MIN_ASCII 32
#define MAX_ASCII 255

#define CMD_READ_TIMEOUT 50
#define READ_TIMEOUT 100
int runState;
unsigned long pauseMillisL = 0;  // To store the time when the timer was paused
unsigned long pauseMillis3 = 0;
unsigned long previousMillisL = 0;      // will store last time LED was updated
unsigned long previousMillisTimer = 0;  // will store last time timer was updated
const long intervalL = 1000;            // interval at which to blink LED (milliseconds)
const long intervalTimer = 990;         // interval at which to update timer (milliseconds)
unsigned long startTimer;
unsigned long previoustimer;
unsigned long duration;
int motorSpeed = 200;        // PWM value (0-255)
unsigned long time1 = 200;   // 2 seconds
unsigned long time2 = 4000;  // 4 seconds
unsigned long time3 = 6000;  // 6 seconds
// int seconds = 0;             // Timer seconds
// int minutes = 0;             // Timer minutes
unsigned int minutes;
int hours = 0;              // Timer hours
bool timerRunning = false;  // Timer running flag
unsigned long totalSeconds, totalSecondsL;
byte restartbuffer[] = {
  CMD_HEAD1, CMD_HEAD2, 0x07, CMD_WRITE, 0x00, 0x04, 0x55, 0xAA, 0x5A, 0xA5
};

int address1 = 0;
int address2 = 0;
int data1 = 0;

int data2 = 0;

int dataValue;
int dataIndex = 1;
String addressString = "";
String separatedData = "";
int dataint = 0;
unsigned long elapsedTime;
// unsigned long startTime = 0;

byte currentPage;
int setPressure = 40, startValue, setPressureHigh = 40, setPressureLow = 40, HighPressure_time = 1, LowPressure_time = 1;
const int in1Pin = PC15;          //npwt
const int in2Pin = PC14;          // npwt
const int pwmPin = PA1;           //npwt
const int Solenoid_valve = PC13;  // LED connected to digital pin PC13

const int motorPin1 = PA11;
const int motorPin2 = PA12;
const int motorPWM = PA4;
const int buzzer_pin = PB5;


String timeStr, dateStr, timer_continous, timer_intermittent, String_timer_Int_cycle;
SparkFun_MicroPressure mpr;
int pressure_mmhg = 0, absPressure;
unsigned long previousMillis = 0;
const long interval = 1000;  // interval at which to run the function (milliseconds)


unsigned long currentMillis;
unsigned long currentMillisL;
unsigned long previousMillis3;
unsigned long previousMillis4;
unsigned long previousMillis5;
unsigned long totalSeconds5;
unsigned long interval3 = 1000;  // Interval in milliseconds
unsigned long interval4 = 1000;  // Interval in milliseconds
unsigned long interval5 = 1000;  // Interval in milliseconds
unsigned long elapsedMillis3 = 0;
unsigned long elapsedMillis4 = 0;
unsigned long elapsedMillis5 = 0;
unsigned long elapsedMillisL = 0;
bool isRunning3 = false;
bool isRunningL = false;
bool isPaused3 = false;
bool isRunning4 = false;
bool isPaused4 = false;
bool isPausedL = false;
unsigned long pauseMillis4 = 0;
unsigned long pauseMillis5 = 0;
bool isRunning5 = false;
bool isPaused5 = false;
// rtc time objects to start the timrs for idividual opeartion.
// every timer object should be dedicated to that operation mixing of timer objects cause conflicts.
DateTime startTime;                // object for the rtc timer
DateTime startTimeL;               // issue got with timer in leakage. due leakge timer "startTime" using same object effecting the main timer object. which continues to run the time.now() method to give the background run issue.
DateTime startTimeint, pauseTime;  // issue got with timer in leakage. due leakge timer "startTime" using same object effecting the main timer object. which continues to run the time.now() method to give the background run issue.
DateTime startTime_int_cycle;

int Continous_Mode = 0,valve_state;
int Intermittent_Mode, Intermittent_state, intermittent_pressure = 40, variable_state;
int Variable_Mode = 0, variable_pressure = 100, setPressure_int_cycle;
// int sollutionVolume, soakTime;
// int instillationRun;
// const int solutionVolume123456 = sollutionVolume; // Starting volume in ml
// int currentVolume123456;
// unsigned long previousMillis123456 = 0;
// const long interval123456 = 3000;  // 3 seconds per ml
// peristalic mode
// int soakrun, instillationSoakrun;

// bool instillationRun123456 = false;  // Variable to control the timer
int internittentHigh, internittentLow;

const int intermittentHighMin = 2;

const int intermittentLowMin = 2;

int high_presure_time_state = 0;

unsigned int minutes4;
unsigned long totalSeconds4;
unsigned int seconds4;
unsigned int minutes5;
int intermittentHigh = 60;
int intermittentLow = 60;

// Timing variables for the down counters
unsigned long previousMillisHigh = 0;
unsigned long previousMillisLow = 0;


// Time interval in milliseconds (1 second = 1000 milliseconds)
const unsigned long intervalint = 1000;  // 1 second
bool valaveTriggered, pressureset, resetkey, leakagetest, pressuresetHigh, pressuresetLow, blockage_state;


// timer6 variables for intermittent timer

bool isRunning6 = false;
bool isPaused6 = false;
unsigned long elapsedMillis6 = 0;
unsigned long pauseMillis6 = 0;
unsigned long previousMillis6 = 0;
unsigned long interval6 = 1000;  // Example interval for 1 second
String intermittent_dly_timer = "";

int countdownTimeInt;                             // Set to the initial value (5 minutes)
int countdownTimeOriginalInt = countdownTimeInt;  // Store original countdown value to reset later

unsigned long previousMillisInt = 0;  // Store the last time a minute passed
unsigned long intervalInt = 75000;    // 1 minute in milliseconds (60,000 ms)   its nmore than 60000 ms because of the code delay
bool countdownRunningInt = true;      // Flag to check if the countdown is still running
int highrun = 0;
unsigned long currentMillisInt;

unsigned int minutes4_int;
int speed;
unsigned int seconds;

// Blockage Timer Variables
bool isRunningB = false;               // Whether the blockage timer is running
bool isPausedB = false;                // Whether the blockage timer is paused
unsigned long previousMillisB = 0;     // To store the last time the timer was updated
unsigned long currentMillisB = 0;      // To store the current time in milliseconds
unsigned long elapsedMillisB = 0;      // To store the elapsed time in milliseconds
unsigned long pauseMillisB = 0;        // To store the time when the timer was paused
unsigned long totalSecondsB = 0;       // To store the total elapsed time in seconds
const unsigned long intervalB = 1000;  // Interval for the blockage timer (in milliseconds)
DateTime startTimeB;                   // Start time of the blockage timer
// int blockage_state = 0;           // Blockage detection status (0 = no blockage, 1 = blockage)


void setup() {
  Serial.begin(115200);   // Initialize Serial for output at 115200 baud rate
  Serial2.begin(115200);  // Initialize Serial2 for input at 115200 baud rate
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  pinMode(pwmPin, OUTPUT);
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPWM, OUTPUT);
  pinMode(Solenoid_valve, OUTPUT);
  pinMode(PB4, OUTPUT);
  pinMode(ANALOG_IN_PIN2, INPUT);
  pinMode(buzzer_pin, OUTPUT);

      if(absPressure>10){
        digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);
      }


  digitalWrite(Solenoid_valve, HIGH);
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


  // rtc.adjust(DateTime(2024, 10, 5, 15, 47, 0));


  if (!mpr.begin()) {
    Serial.println("Cannot connect to MicroPressure sensor.");
    // while (1)
    // ;
  }
  Serial2.write(restartbuffer, sizeof(restartbuffer));


  setIntegerVP(0x43, 0x00, 0);

  delay(1000);
  setIntegerVP(0x43, 0x00, 1);
  delay(1000);
  setIntegerVP(0x43, 0x00, 2);
  delay(1000);
  setIntegerVP(0x43, 0x00, 3);
  delay(1000);






  setPage(1);
  Time_date();
}

void loop() {
  // Check if it's time to run the pressure calculation
  //Serial.println("loop");

  unsigned long currentMillis = millis();

  unsigned long currentTime = millis();



  startTimer = millis();


  if (startTimer - previousMillisTimer >= duration) {

    previousMillisTimer = startTimer;
    pressure_mmhg = (mpr.readPressure(TORR) - 710) ;
    absPressure = abs(pressure_mmhg);
    //   if (leakagetest == 1) {
    //     digitalWrite(buzzer_pin, 1);
    //     delay(100);
    //     digitalWrite(buzzer_pin, 0);
    //     delay(100);
    //   }

    //   else {

    //     digitalWrite(buzzer_pin, 0);
    //   }
    // }

    // if(blockage_state)
    // {
    //     setIntegerVP(0x41, 0x00, 23);  // clear leakage alert icon
    //        digitalWrite(buzzer_pin, 1);
    //     delay(100);
    //     digitalWrite(buzzer_pin, 0);
    //     delay(100);

    // }
    // else if(!blockage_state && !leakagetest){

    //     setIntegerVP(0x41, 0x00, 25);  // clear leakage alert icon

    //     digitalWrite(buzzer_pin, 0);

    // }

    if (leakagetest == 1 || blockage_state) {
      if (blockage_state) {
        setIntegerVP(0x41, 0x00, 23);  // blockge  alert icon
      }
      digitalWrite(buzzer_pin, 1);
      delay(100);
      digitalWrite(buzzer_pin, 0);
      delay(100);
    } else if (!blockage_state && !leakagetest) {
      setIntegerVP(0x41, 0x00, 25);  // clear leakage/blockage alert icon
      digitalWrite(buzzer_pin, 0);
    }
  }
  if (currentMillis - previousMillis >= interval) {

    previousMillis = currentMillis;
    // Run the pressure calculation
    // if(absPressure<200)
    // {
    // }
    Serial.println("totalSeconds5-----------: " + String(totalSeconds5));


    if (absPressure > 30) {

      //if ((absPressure > (setPressure - 10) && absPressure < setPressure) || (absPressure < (setPressure+10) && absPressure > setPressure)) {
      // if(absPressure>50){
      if (absPressure % 5 == 0) {
        setIntegerVP(0x10, 0x01, absPressure);
      }


    }

    else {
      setIntegerVP(0x10, 0x01, absPressure);
    }

    // else if (absPressure % 10 == 0) {
    //  setIntegerVP(0x10, 0x01,absPressure);
    // }n
    //}
    //else {
    // setIntegerVP(0x10, 0x01, absPressure);
    //}

    // }
    // else {
    //   setIntegerVP(0x10, 0x01, absPressure);
    // }


    Serial.print("pressure_mmhg:---------");
    Serial.println(absPressure);
    // else if(){
    //     pressure_mmhg = (mpr.readPressure(TORR)-710)*0.70;
    // }


    if (currentPage == 7) {
      Continous_Mode = 1;
      Intermittent_Mode = 0;
      Variable_Mode = 0;
      // setPressureHigh =0;
      // setPressureLow =0;
      valve_state =0;

      valveOFF();
    }


    else if (currentPage == 6) {
      Continous_Mode = 0;
      Intermittent_Mode = 0;
      Variable_Mode = 0;
      runState = 1;
      stopLeakageTimer();
      stopBlockageTimer();
      // release_pressure();

    }




    else if (currentPage == 9) {
      Continous_Mode = 0;
      Variable_Mode = 0;
      Intermittent_Mode = 1;
      // setPressure =0;
         valve_state =0;

      valveOFF();

    }

    else if (currentPage == 8) {
      Continous_Mode = 0;
      Variable_Mode = 0;
      Intermittent_Mode = 0;
      stopLeakageTimer();
      stopBlockageTimer();
      // release_pressure();



    }

    else if (currentPage == 10) {
      Continous_Mode = 0;
      Variable_Mode = 0;
      Intermittent_Mode = 0;
      stopLeakageTimer();
      stopBlockageTimer();

      // stopTimer_int();
      // release_pressure();


    } else if (currentPage == 11) {
      Continous_Mode = 0;
      Variable_Mode = 0;
      Intermittent_Mode = 0;
      stopLeakageTimer();
      stopBlockageTimer();

      stopTimer_int();
      stopTimer_Int_cycle();
      // release_pressure();

      Serial.println("timer cleared +++++++++++++++++++++++++++++++++++++++++++++");
      // resatrt_int_timer=1;


    } else if (currentPage == 1) {
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
      Variable_Mode = 0;
      Intermittent_Mode = 0;
      stopLeakageTimer();
      stopBlockageTimer();
      // release_pressure();

    }

    // else{
    //   release_pressure();

    // }


  if (currentPage != 9 && currentPage != 7)
 {

      release_pressure();

 }

    intermittentmode();

    continousmode();



    variablepressuremode();


    DateTime now = rtc.now();

    // Create date and time strings
    dateStr = String(now.year(), DEC) + '/' + String(now.month(), DEC) + '/' + String(now.day(), DEC);

    Serial.println("dateStr------------------------>:"+dateStr);
    timeStr = String(now.hour(), DEC) + ':' + String(now.minute(), DEC);

    setText(0x36, 0x00, String("0" + timeStr + "  "));  //00


    setText(0x35, 0x00, String("0" + dateStr));  //0
                                                 // pressure_mmhg = ((mpr.readPressure(INHG) - 28) * 25.4);
                                                 // setText(0x10, 0x00, pressure_mmhg);  //0

    setIntegerVP(0x10, 0x00, setPressure);
    setIntegerVP(0x15, 0x05, setPressureHigh);
    setIntegerVP(0x15, 0x04, setPressureLow);
    setIntegerVP(0x15, 0x02, LowPressure_time);
    setIntegerVP(0x15, 0x00, HighPressure_time);

    adc_value2 = analogRead(ANALOG_IN_PIN2);
    adc_voltage2 = (adc_value2 * ref_voltage) / 4095.0;

    in_voltage2 = adc_voltage2 / (R2 / (R1 + R2));
    // in_voltage2 = 9;
    //  Serial.print(F("Voltage: "));
    // Serial.println(in_voltage2);
    if (in_voltage2 > 12) {

      //       //   // setfloat(0x70, 0x00, 5);
      setIntegerVP(0x43, 0x00, 10);

    }


    else if (in_voltage2 <= 12 || in_voltage2 > 11.5) {

      //       //   // setfloat(0x70, 0x00, 5);
      setIntegerVP(0x43, 0x00, 9);

    }

    else if (in_voltage2 >= 11 || in_voltage2 < 11.5) {

      //       //   // setfloat(0x70, 0x00, 5);
      setIntegerVP(0x43, 0x00, 8);

    } else if (in_voltage2 >= 10 || in_voltage2 < 11) {

      //       //   // setfloat(0x70, 0x00, 5);
      setIntegerVP(0x43, 0x00, 7);

    }

    else if (in_voltage2 < 10) {

      //       //   // setfloat(0x70, 0x00, 2);
      setIntegerVP(0x43, 0x00, 6);


      //       // }
    }

    currentPage = getPage();
    Serial.print("Current Page: ");
    Serial.println(currentPage);

    // Serial.println("runState");
    // Serial.println(runState);

    // Print date and time strings


    switch (runState) {
      case -1:
        // Timer is stopped
        // setText(0x30, 0x00, String("000:00:00 "));
        //  Serial.print("Device_Mode:");
        // Serial.println("No run");
        // isRunning3 = false;
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
        updateTimer_cont();

        break;

      case 2:
        // Timer is running


        //  Serial.print("Device_Mode:");
        //  Serial.println("continous Mode");

        if (!isRunning4) {
          startTimer_int();  // Start the timer if not already running
        }
        updateTimer_int();


        //  if(pressuresetHigh || pressuresetLow){

        //  }
        break;

      case 3:




        break;
      default:
        // Handle other states if necessary

        break;
    }



    if (currentTime - lastUpdateTime >= timerInterval) {
      // Update the last time the timer was updated
      lastUpdateTime = currentTime;
    }
  }


  // Main loop for reading and processing serial data
  if (Serial2.available()) {
    // delayMicroseconds(20);
    // Read the incoming byte data from the software serial
    String receivedData = "";
    while (Serial2.available()) {
      int byteReceived = Serial2.read();
      if (byteReceived < 16) {
        receivedData += "0";  // Add leading zero for single digit bytes
      }
      receivedData += String(byteReceived, HEX) + " ";
    }
    // Remove the trailing space from the string
    receivedData.trim();

    // Print the received data to the Serial Monitor
    // Serial.print("Received: ");
    // Serial.println(receivedData);

    // Separate and label each byte
    dataIndex = 1;
    for (int i = 0; i < receivedData.length(); i += 3) {
      String hexByte = receivedData.substring(i, i + 2);
      int byteValue = strtol(hexByte.c_str(), NULL, 16);

      // Assign to address1, address2, data1, data2 based on the data index
      if (dataIndex == 4) {
        address1 = byteValue;
      } else if (dataIndex == 5) {
        address2 = byteValue;
      } else if (dataIndex == 7) {
        data1 = byteValue;
      } else if (dataIndex == 8) {
        data2 = byteValue;
      }

      separatedData += "d" + String(dataIndex) + ": " + hexByte + " ";
      dataIndex++;
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

    // Serial.print("Data String: ");
    // Serial.println(dataString);

    Serial.print("Data Value: ");
    Serial.println(dataValue);




    if (addressString == "1000") {
      setPressure = dataValue;
      // Serial.println("setPressure:");
      // Serial.println(setPressure);


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


    // else if (addressString == "2000") {
    //   Continous_Mode = dataValue;
    //   Intermittent_Mode = 0;
    //   Variable_Mode = 0;
    //   // Device_Mode =1;  // continous mode

    //   // Serial.println("Device_Mode:");
    //   // Serial.println(Continous_Mode);

    // }

    // else if (addressString == "2520") {
    //   Continous_Mode = 0;
    //   Variable_Mode = 0;
    //   Intermittent_Mode = dataValue;

    //   // Serial.println("Device_Mode:intermittent");
    //   // Serial.println(Intermittent_Mode);
    // } else if (addressString == "3530") {
    //   // startValue = dataValue;
    //   Continous_Mode = 0;

    //   Variable_Mode = dataValue;
    //   Intermittent_Mode = 0;
    // }





    // else if (addressString == "2620") {


    //   resetkey = dataValue;
    // }
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
    startTime = rtc.now() - TimeSpan(pauseMillis3 / 1000);  // Adjust the start time
    previousMillis3 = millis();                             // Reset the timer
    // Serial.println("Timer Resumed");
  } else {
    // Start the timer from scratch
    currentMillis = millis();
    startTime = rtc.now();
    isRunning3 = true;
    isPaused3 = false;
    elapsedMillis3 = 0;
    previousMillis3 = currentMillis;  // Reset the timer
    // Serial.println("Timer Started");
  }
}

void pauseTimer_cont() {
  if (isRunning3 && !isPaused3) {
    isPaused3 = true;
    isRunning3 = false;
    pauseMillis3 = elapsedMillis3;  // Store the elapsed time when paused
    // Serial.println("Timer Paused");
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
  startTime = rtc.now();       // Reset the start time

  // Reset the timer string to "00:00:00"
  timer_continous = "00:00";

  // Update the display with the reset time
  setText(0x30, 0x00, String("0" + timer_continous));

  // Print to Serial to verify
  // Serial.println("Timer stopped and reset to 00:00:00");
}

void updateTimer_cont() {
  if (isRunning3 && !isPaused3) {
    currentMillis = millis();
    if (currentMillis - previousMillis3 >= interval3) {
      previousMillis3 = currentMillis;
      DateTime now = rtc.now();
      elapsedMillis3 = (now.unixtime() - startTime.unixtime()) * 1000;  // Convert to milliseconds

      // Convert elapsedMillis3 to HH:MM:SS format
      totalSeconds = elapsedMillis3 / 1000;
      unsigned int hours = totalSeconds / 3600;
      unsigned int minutes = (totalSeconds % 3600) / 60;
      seconds = totalSeconds % 60;

      char timeString[9];  // Buffer to store the time string in HH:MM:SS format
      snprintf(timeString, sizeof(timeString), "%02u:%02u:%02u", hours, minutes, seconds);
      timer_continous = String(timeString);


      // if(totalSeconds<10 && absPressure > setPressure)
      // {
      //   blockage_state =1;
      // }

      // else{
      //   blockage_state =0;



      // }


      if (!isRunningB) {
        startBlockageTimer();  // Start the timer if not already running
      }
      blockagetimer();


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
    startTime_int_cycle = rtc.now() - TimeSpan(pauseMillis4 / 1000);  // Adjust the start time
    previousMillis4 = millis();                                       // Reset the timer
    // Serial.println("Timer 4 Resumed");
  } else {
    // Start the timer from scratch
    currentMillis = millis();
    startTime_int_cycle = rtc.now();
    isRunning4 = true;
    isPaused4 = false;
    elapsedMillis4 = 0;
    previousMillis4 = currentMillis;  // Reset the timer
    // Serial.println("Timer 4 Started");
  }
}

void pauseTimer_int() {
  if (isRunning4 && !isPaused4) {
    isPaused4 = true;
    isRunning4 = false;
    pauseMillis4 = elapsedMillis4;  // Store the elapsed time when paused
    Serial.println("Timer 4 Paused---------------------------------------");
  }
}

// void stopTimer_int() {
//   // Reset all relevant variables
//   isRunning4 = false;
//   isPaused4 = false;
//   elapsedMillis4 = 0;
//   pauseMillis4 = 0;
//   previousMillis4 = millis();  // Reset to current time
//   startTime = rtc.now();       // Reset the start time
//   // Serial.println("Timer 4 Stopped and Reset");
// }

void stopTimer_int() {
  // Ensure the timer is properly reset
  isRunning4 = false;   // Set the timer to not running
  isPaused4 = false;    // Make sure it's not paused
  elapsedMillis4 = 0;   // Reset the elapsed time
  pauseMillis4 = 0;     // Clear any paused duration
  previousMillis4 = 0;  // Reset the previous time
  currentMillis = 0;    // Reset the current time to avoid stale values
  totalSeconds4 = 0;
  startTime_int_cycle = rtc.now();  // Reset the startTime using a default rtc DateTime object

  // Optionally update the display or any output to reflect the reset state
  setText(0x70, 0x00, String(" 00:00"));  // Reset the display to "00:00:00"

  Serial.println("Timer 4 Stopped and Reset");  // Debugging
}


void updateTimer_int() {
  Serial.println("timer updating....................................");
  if (isRunning4 && !isPaused4) {
    currentMillis = millis();
    if (currentMillis - previousMillis4 >= interval4) {
      previousMillis4 = currentMillis;
      DateTime now = rtc.now();
      elapsedMillis4 = (now.unixtime() - startTime_int_cycle.unixtime()) * 1000;  // Convert to milliseconds

      // Convert elapsedMillis4 to HH:MM:SS format
      totalSeconds4 = elapsedMillis4 / 1000;
      unsigned int hours4 = totalSeconds4 / 3600;
      minutes4 = (totalSeconds4 % 3600) / 60;
      seconds4 = totalSeconds4 % 60;

      char timeString4[9];  // Buffer to store the time string in HH:MM:SS format
      snprintf(timeString4, sizeof(timeString4), "%02u:%02u:%02u", hours4, minutes4, seconds4);
      timer_intermittent = String(timeString4);

      if (!isRunningB) {
        startBlockageTimer();  // Start the timer if not already running
      }
      blockagetimer();

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


void startTimer_Int_cycle() {
  if (isPaused5) {
    // Resume from pause
    isPaused5 = false;
    isRunning5 = true;

    // Adjust the start time by subtracting the paused duration
    TimeSpan pausedDuration = rtc.now() - pauseTime;  // Calculate how long the timer was paused
    startTimeint = startTimeint + pausedDuration;     // Adjust start time to account for the pause

    previousMillis5 = millis();  // Reset the timer to account for delay during pause
    Serial.println("Timer 5 Resumed");
  } else {
    // Start the timer from scratch
    currentMillis = millis();
    startTime = rtc.now();
    startTimeint = startTime;  // Set startTimeint when starting
    isRunning5 = true;
    isPaused5 = false;
    elapsedMillis5 = 0;
    previousMillis5 = currentMillis;  // Reset the timer
    Serial.println("Timer 5 Started");
  }
}

void pauseTimer_Int_cycle() {
  if (isRunning5 && !isPaused5) {
    isPaused5 = true;
    isRunning5 = false;

    // Save the current time when paused
    pauseTime = rtc.now();          // Store when the timer was paused
    pauseMillis5 = elapsedMillis5;  // Store the elapsed time when paused
    Serial.println("Timer int cycle Paused");
  }
}

void stopTimer_Int_cycle() {
  // Reset all relevant variables
  isRunning5 = false;
  isPaused5 = false;
  elapsedMillis5 = 0;
  pauseMillis5 = 0;
  previousMillis5 = millis();  // Reset to current time
  startTimeint = rtc.now();    // Reset the start time
  Serial.println("Timer 5 Stopped and Reset");
}

void updateTimer_Int_cycle() {
  // Only update the timer if it's running and not paused
  if (isRunning5 && !isPaused5) {
    currentMillis = millis();
    if (currentMillis - previousMillis5 >= interval5) {
      previousMillis5 = currentMillis;
      DateTime now = rtc.now();
      elapsedMillis5 = (now.unixtime() - startTimeint.unixtime()) * 1000;  // Convert to milliseconds

      // Convert elapsedMillis5 to minutes
      totalSeconds5 = elapsedMillis5 / 1000;
      minutes5 = (totalSeconds5 % 3600) / 60;

      // Reset to zero after reaching 10 minutes
      if (minutes5 >= HighPressure_time + LowPressure_time) {
        minutes5 = 0;
        startTimeint = now;  // Reset start time to now
      }

      char timeString5[3];  // Buffer to store the time string for minutes only
      snprintf(timeString5, sizeof(timeString5), "%02u", minutes5);
      String_timer_Int_cycle = String(timeString5);

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


    Run_icon();

    // if (totalSecondsL <5 && absPressure > setPressure) {
    //         runState = -1;
    //         leakagetest = 1;  // leakge feedback


    //         // delay(100);
    //         // setPage(15);
    //         setIntegerVP(0x41, 0x00, 3);  // show leakage alert icon

    //         // alarm alert
    //       }

    //       else {

    //    runState = 1;
    //         leakagetest = 0;  // leakge feedback
    //         setIntegerVP(0x41, 0x00, 4);  // show leakage alert icon


    //       }
    MaintainPressure();

  }


  else if (Intermittent_Mode == 0 && Continous_Mode == 0 && Variable_Mode == 0) {

    runState = -1;

    pauseTimer_cont();
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
        if (!isRunning5) {
          startTimer_Int_cycle();  // Start the timer if not already running
        }
        updateTimer_Int_cycle();
      }

      else {

        if (absPressure > setPressureLow-40) {

          digitalWrite(in1Pin, LOW);
          digitalWrite(in2Pin, LOW);
          analogWrite(pwmPin, 0);  // Stop

          valveON();
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

          high_presure_time_state = 1;
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



  }

  else if (Intermittent_Mode == 0 && Continous_Mode == 0 && Variable_Mode == 0) {

    runState = -2;
    // if(currentPage!=11)
    // {
    pauseTimer_int();
    // }

    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
    analogWrite(pwmPin, 0);  // Stop
  }
}






void variablepressuremode() {


  if (Variable_Mode == 1) {
    runState = 3;

    Device_mode = 3;

    Continous_Mode = 0;
    Intermittent_Mode = 0;
    if (variable_state == 0) {
      // Intermittent_state=0;
      MaintainPressure();

    } else if (variable_state == 1) {
      digitalWrite(in1Pin, LOW);
      digitalWrite(in2Pin, LOW);
      analogWrite(pwmPin, 0);  // Stop
    }

    if (absPressure < variable_pressure) {
      variable_state = 0;
    }



    if (absPressure > setPressure) {
      variable_state = 1;
    }

  }

  else if (Intermittent_Mode == 0 && Continous_Mode == 0 && Variable_Mode == 0) {

    runState = -3;
    // pauseTimer_Int_cycle();
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
    analogWrite(pwmPin, 0);  // Stop

    // Serial.println("stopped");
  }
}

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
    analogWrite(pwmPin, 170);  // speed
    delay(10);
    analogWrite(pwmPin, 160);  // speed
    delay(10);
  } else if (setPressure >= 40 && setPressure < 50) {
    analogWrite(pwmPin, 180);  // speed
    delay(10);
    analogWrite(pwmPin, 170);  // speed
    delay(10);
  } else if (setPressure >= 50 && setPressure < 170) {
    analogWrite(pwmPin, 190);  // speed
    delay(10);
    analogWrite(pwmPin, 180);  // speed
    delay(10);
  } else if (setPressure >= 170 && setPressure <= 200) {
    analogWrite(pwmPin, 210);  // speed
    delay(10);
    analogWrite(pwmPin, 200);  // speed
    delay(10);
  }
}

void finalRun() {

  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, HIGH);


  // analogWrite(pwmPin, 200);  //  speed


  if (setPressure > 120 && setPressure <= 200) {
    // Code for setPressure between 0 and 9
    analogWrite(pwmPin, 230);  //  speed
    delay(500);
    analogWrite(pwmPin, 0);  //  speed
    delay(10);
  }

  else if (setPressure <= 120) {

    analogWrite(pwmPin, 210);  //  speed
    delay(250);
    analogWrite(pwmPin, 0);  //  speed
    delay(10);
  }
}

void initialRun_low() {

  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, HIGH);



  // analogWrite(pwmPin, 200);  //  speed
  if (setPressureLow < 30) {
    // Code for setPressure between 0 and 29 (No analogWrite actions)
  } else if (setPressureLow >= 30 && setPressureLow < 40) {
    analogWrite(pwmPin, 170);  // speed
    delay(10);
    analogWrite(pwmPin, 160);  // speed
    delay(10);
  } else if (setPressureLow >= 40 && setPressureLow < 50) {
    analogWrite(pwmPin, 180);  // speed
    delay(10);
    analogWrite(pwmPin, 170);  // speed
    delay(10);
  } else if (setPressureLow >= 50 && setPressureLow < 170) {
    analogWrite(pwmPin, 190);  // speed
    delay(10);
    analogWrite(pwmPin, 180);  // speed
    delay(10);
  } else if (setPressureLow >= 170 && setPressureLow <= 200) {
    analogWrite(pwmPin, 210);  // speed
    delay(10);
    analogWrite(pwmPin, 200);  // speed
    delay(10);
  }
}


void finalRun_low() {

  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, HIGH);




  if (setPressureLow > 120 && setPressureLow <= 200) {
    // Code for setPressure between 0 and 9
    analogWrite(pwmPin, 230);  //  speed
    delay(500);
    analogWrite(pwmPin, 0);  //  speed
    delay(10);
  }

  else if (setPressureLow <= 120) {

    analogWrite(pwmPin, 210);  //  speed
    delay(250);
    analogWrite(pwmPin, 0);  //  speed
    delay(10);
  }
}

void initialRun_high() {

  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, HIGH);

  // analogWrite(pwmPin, 200);  //  speed
  if (setPressureHigh < 30) {
    // Code for setPressure between 0 and 29 (No analogWrite actions)
  } else if (setPressureHigh >= 30 && setPressureHigh < 40) {
    analogWrite(pwmPin, 170);  // speed
    delay(10);
    analogWrite(pwmPin, 160);  // speed
    delay(10);
  } else if (setPressureHigh >= 40 && setPressureHigh < 50) {
    analogWrite(pwmPin, 180);  // speed
    delay(10);
    analogWrite(pwmPin, 170);  // speed
    delay(10);
  } else if (setPressureHigh >= 50 && setPressureHigh < 170) {
    analogWrite(pwmPin, 190);  // speed
    delay(10);
    analogWrite(pwmPin, 180);  // speed
    delay(10);
  } else if (setPressureHigh >= 170 && setPressureHigh <= 200) {
    analogWrite(pwmPin, 210);  // speed
    delay(10);
    analogWrite(pwmPin, 200);  // speed
    delay(10);
  }
}


void finalRun_high() {

  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, HIGH);




  if (setPressureHigh > 120 && setPressureHigh <= 200) {
    // Code for setPressure between 0 and 9
    analogWrite(pwmPin, 230);  //  speed
    delay(500);
    analogWrite(pwmPin, 0);  //  speed
    delay(10);
  }

  else if (setPressureHigh <= 120) {

    analogWrite(pwmPin, 210);  //  speed
    delay(250);
    analogWrite(pwmPin, 0);  //  speed
    delay(10);
  }
}

void MaintainPressure() {

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



void startLeakageTimer() {
  if (isPausedL) {
    // Resume from pause
    isPausedL = false;
    isRunningL = true;
    startTimeL = rtc.now() - TimeSpan(pauseMillisL / 1000);  // Adjust the start time
    previousMillisL = millis();                              // Reset the timer
    // Serial.println("Leakage Timer Resumed");
  } else {
    // Start the timer from scratch
    currentMillisL = millis();
    startTimeL = rtc.now();
    isRunningL = true;
    isPausedL = false;
    elapsedMillisL = 0;
    previousMillisL = currentMillisL;  // Reset the timer
    // Serial.println("Leakage Timer Started");
  }
}

void pauseLeakageTimer() {
  if (isRunningL && !isPausedL) {
    isPausedL = true;
    isRunningL = false;
    pauseMillisL = elapsedMillisL;  // Store the elapsed time when paused
    // Serial.println("Leakage Timer Paused");
  }
}

void stopLeakageTimer() {
  // Reset all relevant variables
  isRunningL = false;
  isPausedL = false;
  elapsedMillisL = 0;
  pauseMillisL = 0;
  totalSecondsL = 0;
  previousMillisL = millis();    // Reset to current time
  startTimeL = rtc.now();        // Reset the start time
                                 // Serial.println("Leakage Timer Stopped and Reset");
  setIntegerVP(0x41, 0x00, 25);  // clear leakage alert icon
  totalSecondsL = 0;
  leakagetest = 0;
}

void leakagetimer() {


  if (isRunningL && !isPausedL) {
    currentMillisL = millis();
    if (currentMillisL - previousMillisL >= intervalL) {
      previousMillisL = currentMillisL;
      DateTime now = rtc.now();
      elapsedMillisL = (now.unixtime() - startTimeL.unixtime()) * 1000;  // Convert to milliseconds

      // Convert elapsedMillisL to HH:MM:SS format
      totalSecondsL = elapsedMillisL / 1000;

      Serial.println("totalSecondsL------------------------------------------");
      Serial.println(totalSecondsL);


      if (totalSecondsL > 119 && absPressure < setPressure * 0.9) {
        runState = -1;
        leakagetest = 1;  // leakge feedback


        // delay(100);
        // setPage(15);
        setIntegerVP(0x41, 0x00, 24);  // show leakage alert icon

        // alarm alert
      }



      if (absPressure > setPressure) {
        // runState = -1;
        leakagetest = 0;  // leakge feedback

        stopLeakageTimer();
        // delay(100);
        // setPage(15);
        //  setIntegerVP(0x41, 0x00, 3);  // show leakage alert
        //  setIntegerVP(0x41, 0x00, 4);  // show leakage alert icon


        // alarm alert




      }


      else {
      }
    }
  }
}


void startBlockageTimer() {
  if (isPausedB) {
    // Resume from pause
    isPausedB = false;
    isRunningB = true;
    startTimeB = rtc.now() - TimeSpan(pauseMillisB / 1000);  // Adjust the start time
    previousMillisB = millis();                              // Reset the timer
    // Serial.println("Blockage Timer Resumed");
  } else {
    // Start the timer from scratch
    currentMillisB = millis();
    startTimeB = rtc.now();
    isRunningB = true;
    isPausedB = false;
    elapsedMillisB = 0;
    previousMillisB = currentMillisB;  // Reset the timer
    // Serial.println("Blockage Timer Started");
  }
}

void pauseBlockageTimer() {
  if (isRunningB && !isPausedB) {
    isPausedB = true;
    isRunningB = false;
    pauseMillisB = elapsedMillisB;  // Store the elapsed time when paused
    // Serial.println("Blockage Timer Paused");
  }
}

void stopBlockageTimer() {
  // Reset all relevant variables
  isRunningB = false;
  isPausedB = false;
  elapsedMillisB = 0;
  pauseMillisB = 0;
  totalSecondsB = 0;
  previousMillisB = millis();    // Reset to current time
  startTimeB = rtc.now();        // Reset the start time
                                 // Serial.println("Blockage Timer Stopped and Reset");
  setIntegerVP(0x41, 0x00, 25);  // clear blockage alert icon
  totalSecondsB = 0;
  blockage_state = 0;
}

void blockagetimer() {
  if (isRunningB && !isPausedB) {
    currentMillisB = millis();
    if (currentMillisB - previousMillisB >= intervalB) {
      previousMillisB = currentMillisB;
      DateTime now = rtc.now();
      elapsedMillisB = (now.unixtime() - startTimeB.unixtime()) * 1000;  // Convert to milliseconds

      // Convert elapsedMillisB to HH:MM:SS format
      totalSecondsB = elapsedMillisB / 1000;

      Serial.println("totalSecondsB------------------------------------------");
      Serial.println(totalSecondsB);

      if (totalSecondsB < 8 && absPressure > 230) {
        blockage_state = 1;  // blockage feedback
        // runState = -1;
        setIntegerVP(0x41, 0x00, 23);  // show blockage alert icon
        // alarm alert
      }

      if (absPressure < 230) {
        blockage_state = 0;  // blockage feedback
        stopBlockageTimer();
      }
    }
  }
}




void intermittent_high() {

  if (currentMillis - previousMillisHigh >= interval) {
    previousMillisHigh = currentMillis;

    // Decrement the intermittentHigh value if it's greater than the minimum
    if (intermittentHigh > intermittentHighMin) {
      intermittentHigh--;
      Serial.print("intermittentHigh: ");
      Serial.println(intermittentHigh);
    }
  }
}
void intermittent_low() {


  // Check if the time interval has passed for intermittentLow countdown


  if (currentMillis - previousMillisLow >= interval) {
    previousMillisLow = currentMillis;

    // Decrement the intermittentLow value if it's greater than the minimum
    if (intermittentLow > intermittentLowMin) {
      intermittentLow--;
      Serial.print("intermittentLow: ");
      Serial.println(intermittentLow);
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
void release_pressure()
{
        if(!valve_state)
      {
      // valveON();
      if(absPressure>10){
        digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);
      }
      else{
        valve_state = 1;
      }
      }

      else{
      valveOFF();
      }
}