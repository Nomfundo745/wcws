//Import all required libraries
#include <Wire.h>                  //library for I2C 
#include <Adafruit_MLX90614.h>     
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "MAX30100_PulseOximeter.h"
#include <WiFi.h>
#include <WiFiMulti.h>
#include <HTTPClient.h>
#include <SIM800L.h>
#include "Adafruit_GFX.h"
#include "OakOLED.h"
 
OakOLED oled; //Define variable for OLED Display


WiFiMulti WiFiMulti;
HTTPClient ask;
// TODO: user config
const char* ssid     = "Nomfundo's Galaxy A12"; //Wifi SSID
const char* password = "Nomfundo"; //Wifi Password
const char* apiKeyIn_temp = "rLPh7YXHUlcHtwf7iViFHDbWtKtaXWI4";      // API KEY IN for temp sensor
const char* apiKeyIn_mpu6050 = "yFRTSK4qiKGnDEtzG0O1djbTsWiI5MsC";  // API KEY IN for accelerometer and gyrometer
const char* apiKeyIn_max30100 = "9SdIiua9AbHZX3aDkZqvdo4WTMWcuGft";  //API KEY IN for pulse oximeter sensor
const unsigned int writeInterval = 1000;   // write interval (in ms)

// ASKSENSORS API host config
const char* host = "api.asksensors.com";  // API host name
const int httpPort = 80;      // port
int httpCode;
int BodyTemp, RoomTemp, HeartRate, oxygen;
String url, url1, url2, payload; //storing message
String gsm_message;

//GSM Module RX pin to ESP32 Pin 16
//GSM Module TX pin to ESP32 Pin 17
#define rxPin 16
#define txPin 17
#define BAUD_RATE 115200
HardwareSerial GSMserial(1);
SIM800L gsm;

char number[]="26878047123"; //mobile number
//end gsm SIM800L definitions

// I2C pins for mlx90614 and mpu6050
#define SDA_2 33
#define SCL_2 32

//Definitions for MLX90614 and MPU6050
Adafruit_MLX90614 mlx;

const int MPU6050_addr = 0x68; //MPU6050 sensor configured on address 0x68 for I2C
int16_t AccX,  AccY, AccZ, Temp, GyroX, GyroY, GyroZ;
float ax=0, ay=0, az=0, gx=0, gy=0, gz=0;
int angleChange=0, amplitude=0;

uint32_t tsLastReport = 0;

#define REPORTING_PERIOD_MS     30500

PulseOximeter pox;

boolean fall = false; //stores if a fall has occurred
boolean trigger1=false; //stores if first trigger (lower threshold) has occurred
boolean trigger2=false; //stores if second trigger (upper threshold) has occurred
boolean trigger3=false; //stores if third trigger (orientation change) has occurred
byte trigger1count=0; //stores the counts past since trigger 1 was set true
byte trigger2count=0; //stores the counts past since trigger 2 was set true
byte trigger3count=0; //stores the counts past since trigger 3 was set true

//tasks definitions
TaskHandle_t Task1;
TaskHandle_t Task2;

bool CALL_CONDITION = 1; //Set a flag for the call

// Callback (registered below) fired when a pulse is detected
void onBeatDetected()
{
    Serial.println("Beat!");
}

/*------------------------------------------------------------------------*/

void setup() {
  // put your setup code here, to run once:
  Wire.begin(); //Default pins for max30100
  Wire1.begin(SDA_2, SCL_2, 100000); // I2C pins for mpu6050 and mlx90614
  
  Serial.begin(115200); //Opening Serial communication at baud rate 115200 for all the sensors
  while (!Serial);

  Serial.print("Initializing pulse oximeter..");
  // Initialize the PulseOximeter instance
  // Failures are generally due to an improper I2C wiring, missing power supply or wrong target chip
  Serial.print("pox.begin result: ");
  Serial.println(pox.begin());
  if (!pox.begin()) {
      Serial.println("FAILED to Initialize pulse oximeter");
      for(;;);
  } else {
      Serial.println("SUCCESSFULLY INITIALIZED MAX30100");
          }
   pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA); //set current for MAX30100 
   pox.setOnBeatDetectedCallback(onBeatDetected); // Register a callback for the beat detection
    
  bool status;
  //Check if MLX90614 can be found on its I2C address
  status = mlx.begin(0x5A, &Wire1); //MLX90614 sensor configured on address 0x5A for I2C
  if (!status) {
    Serial.println("Error connecting to MLX sensor. Check wiring.");
    while (1);
  };
  Serial.print("Emissivity = "); Serial.println(mlx.readEmissivity());
  Serial.println("================================================");


  // Initiate Wire library to start data transmission between ESP32 and MPU6050
  Wire1.beginTransmission(MPU6050_addr); 
  Wire1.write(0x6B); //Initiate power management register
  Wire1.write(0);    //Set to zero (wakes up the MPU6050)
  Wire1.endTransmission(true); 

 // for askSensor - begin setup 
  Serial.println("*****************************************************");
  Serial.println("       Program Start : Connect ESP32 to AskSensors.");
  Serial.println("Wait for WiFi... ");
  // connecting to the WiFi network
  WiFiMulti.addAP(ssid, password);
  while (WiFiMulti.run() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  // connected
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  //end setup for asksensors
      
  //GSM SIM800L setup  
  GSMserial.begin(BAUD_RATE, SERIAL_8N1, rxPin, txPin);
  if(gsm.begin(GSMserial)) // Will return 1 if GSM responds and SIM card inserted then
  {
    Serial.println("GSM Initialized");
  }
  else
  {
    Serial.println("GSM not responding");
    while(1); // Infinite loop, code will not execute other stuff
    //setup for OLED
    oled.begin();
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setTextColor(1);
    oled.setCursor(0, 0);
    oled.print("Connection failed:");
    oled.display(); 

  }
  //end GSM SIM800L setup

  //setup for OLED
    oled.begin();
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setTextColor(1);
    oled.setCursor(0, 0);
    oled.print("Connected to network:");
    oled.display(); 

  pinMode(15, OUTPUT); //vibrator motor set up
 
  //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
                    Task1code,   /* Task function. */
                    "Task1",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task1,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */                  
  delay(500);

  //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
                    Task2code,   /* Task function. */
                    "Task2",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */ 
                    1,           /* priority of the task */
                    &Task2,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */
    delay(1000);
}
/*---------------------END SETUP-------------------------------------*/

//task1 function for heartrate and oxygen reading running on 1 core
void Task1code( void * pvParameters ){
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());

  for(;;){
    //code for heartrate and oxygen here - infinite loop
    // Make sure to call update for MAX30100 as fast as possible
    // HeartRate = 0;
     //oxygen = 0;
     pox.update();
    HeartRate = pox.getHeartRate();
    oxygen = pox.getSpO2();
      
    // Asynchronously dump heart rate and oxidation levels to the serial
    // For both, a value of 0 means "invalid"
    if (millis() - tsLastReport > REPORTING_PERIOD_MS) {
      if (HeartRate < 50 || HeartRate > 300){
        //concatenate to gsm_message
       gsm_message = "Abnormal Heart Rate ["+String(int(HeartRate))+" bpm]\n";//casting (no decimals)
      }
      if (oxygen < 80 || oxygen > 100){
        //concatenate to gsm_message
        gsm_message += "Abnormal oxygen level ["+String(oxygen)+" %]\n";
      }
      Serial.print("Heart rate: ");
      Serial.println(HeartRate);
      Serial.print("bpm / SpO2: ");
      Serial.print(oxygen);
      Serial.println("%");
      oled.clearDisplay();
      oled.setTextSize(1);
      oled.setTextColor(1);
      oled.setCursor(0, 16);
      oled.print("Heart BPM:");
      oled.println(HeartRate);
      oled.print("Oxygen Percent:");
      oled.println(oxygen);
      oled.display(); 
      tsLastReport = millis();
    }
  } 
}

//task2 function for the other modules
void Task2code( void * pvParameters ){
  Serial.print("Task2 running on core ");
  Serial.println(xPortGetCoreID());

  for(;;){
    //code previously main in loop - infinite loop too
    //Read the temperature values and print to serial monitor
    BodyTemp = mlx.readObjectTempC();
    RoomTemp = mlx.readAmbientTempC();
    
    if (BodyTemp < 30 || BodyTemp > 40){
      //concatenate to gsm_message
      gsm_message = gsm_message + "Abnormal Temperature ["+String(BodyTemp)+" Celsius]\n ",BodyTemp;
      }
    Serial.print("RoomTemp = "); Serial.print(RoomTemp);
    Serial.print("*C\tBodyTemp = "); Serial.print(BodyTemp); Serial.println("*C");
    Serial.println();
   
    //Call mpu_read function to read acceleration and angular velocity
    mpu_read();
    //detect fall
    if (amplitude<=2){
      //if amplitude breaks lower threshold (0.4g)
      trigger1=true;
      Serial.println("TRIGGER 1 ACTIVATED");
    }
    if (trigger1==true){
      trigger1count++;
      if (amplitude>=12){  //if AM breaks upper threshold (3g)
        trigger2=true;
        Serial.println("TRIGGER 2 ACTIVATED");
        trigger1=false; trigger1count=0;
      }
    }
    if (trigger2==true){
      trigger2count++;
    
      if (angleChange>=3 && angleChange<=4){   //if orientation changes by between 80-100 degrees
        trigger3=true; trigger2=false; trigger2count=0;
        Serial.println(angleChange);
        Serial.println("TRIGGER 3 ACTIVATED");
      }
    }
    if (trigger3==true){
      trigger3count++;
      if (trigger3count>=10){ 
        if ((angleChange>=0) && (angleChange<=10)){    //if orientation changes remains between 0-10 degrees
          fall=true; trigger3=false; trigger3count=0;
          Serial.println(angleChange);
        }
        else{         //user regained normal orientation
          trigger3=false; trigger3count=0;
          Serial.println("TRIGGER 3 DEACTIVATED");
        }
      }
    }
    if (fall==true){  //in event of a fall detection
      Serial.println("FALL DETECTED");
      //Send GSM sms
      gsm_message += "FALL DETECTED!!!\n";
      SendMessage(gsm_message);
      fall=false;
    }
   
  
    //for askSensors - begin for inside loop
    // Use WiFiClient class to create TCP connections
    WiFiClient client;
    
    if (!client.connect(host, httpPort)) {
      Serial.println("connection failed");
      return;
    }
    
    else {
      // Create a URL for updating module 1 for temp sensor
      url = "http://api.asksensors.com/write/";
      url += apiKeyIn_temp;
      url += "?module1=";
      url += RoomTemp;
      url += "&module2=";
      url += BodyTemp;
      
      // Create a URL for updating module 1 and module 2 for MPU6050
      url1 = "http://api.asksensors.com/write/";
      url1 += apiKeyIn_mpu6050;
      url1 += "?module1="; 
      url1 += angleChange; //gyro
      url1 += "&module2="; 
      url1 += amplitude; //accelerometer
    
      // Create a URL for updating module 1 and module 2 for MAX30100
      url2 = "https://api.asksensors.com/write/";
      url2 += apiKeyIn_max30100;
      url2 += "?module1="; 
      url2 += HeartRate; //heart rate
      url2 += "&module2="; 
      url2 += oxygen; //Oxygen levels
      
        
      Serial.print("********** requesting URL: ");
      Serial.println(url);
       // send data 
      ask.begin(url); //Specify the URL
      
      //Check for the returning code
      httpCode = ask.GET();          
     
      if (httpCode > 0) { 
        payload = ask.getString();
        Serial.println(httpCode);
        Serial.println(payload);
      } 
      else {
        Serial.println("Error on HTTP request");
      }
     
      ask.end(); //End
    
      Serial.println(url1);
      //begin for second url
      ask.begin(url1); //Specify the URL
      
      //Check for the returning code
      httpCode = ask.GET();          
     
      if (httpCode > 0) {    
        payload = ask.getString();
        Serial.println(httpCode);
        Serial.println(payload);
      } else {
          Serial.println("Error on HTTP request");
        }
     
      ask.end(); //End
    
      //For url2 of max30100
      Serial.println(url2);
      //begin for second url
      ask.begin(url2); //Specify the URL
      
      //Check for the returning code
      httpCode = ask.GET();          
     
      if (httpCode > 0) { 
        payload = ask.getString();
        Serial.println(httpCode);
        Serial.println(payload);
      } else {
          Serial.println("Error on HTTP request");
        }
     
      ask.end(); //End
         
    }
  
    client.stop();  // stop client
    
    delay(writeInterval);    // delay
    
    //end asksensors inside loop
    
    //Report to emergency services
    //If a message was stored at one sensor reading or more
    if (gsm_message != ""){
      digitalWrite(15, HIGH); //vibrate motor on system 
      //check pb
      if(CALL_CONDITION){//check if call has been made previous loop
        gsm.dialNumber(number); //auto-dial number
        Serial.println("dialing");
         //start GSM SMS sending conditions
        SendMessage(gsm_message); //send sms
        CALL_CONDITION = 0;
      }
      delay(2000);      
      digitalWrite(15, LOW);
      delay(10000);
    }

       Serial.println("********** End ");
  }
}

// read the MPU6050 sensor for acceleration and gyroscope readings in x,y and z directions.
void mpu_read(){
  Wire1.beginTransmission(MPU6050_addr);
  Wire1.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire1.endTransmission(false);
  Wire1.requestFrom(MPU6050_addr,14,true);  // request a total of 14 registers
  AccX = Wire1.read()<<8|Wire1.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AccY = Wire1.read()<<8|Wire1.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AccZ = Wire1.read()<<8|Wire1.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Temp = Wire1.read()<<8|Wire1.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyroX = Wire1.read()<<8|Wire1.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyroY = Wire1.read()<<8|Wire1.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyroZ = Wire1.read()<<8|Wire1.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  ax = (AccX-2050)/16384.00;
  ay = (AccY-77)/16384.00;
  az = (AccZ-1947)/16384.00;
  gx = (GyroX+270)/131.07;
  gy = (GyroY-351)/131.07;
  gz = (GyroZ+136)/131.07;

  Serial.print("AccX = ");
  Serial.print(ax);
  Serial.print("m/s^2, ");
  delay(2000);
  Serial.print("AccY = ");
  Serial.print(ay);
  Serial.print("m/s^2, ");
  delay(2000);
  Serial.print("AccZ = ");
  Serial.print(az);
  Serial.print("m/s^2, ");
  delay(2000);
  Serial.print("Temp = ");
  Serial.print(Temp);
  Serial.print("*C, ");
  delay(2000);
  Serial.print("GyroX = ");
  Serial.print(gx);
  Serial.print("rps, ");
  delay(2000);
  Serial.print("GyroY = ");
  Serial.print(gy);
  Serial.print("rps, ");
  delay(2000);
  Serial.print("GyroZ = ");
  Serial.print(gz);
  Serial.print("rps");
  delay(2000);
  Serial.println();

  // calculating Amplitute vector for 3 axis
  float raw_amplitude = pow(pow(ax,2)+pow(ay,2)+pow(az,2),0.5);
  amplitude = raw_amplitude * 10;  // Multiplied by 10 bcz values are between 0 to 1
  Serial.print("Vector Amplitude = ");
  Serial.println(amplitude);
  delay(2000);

  // calculating Angle change for 3 axis
  angleChange = pow(pow(gx,2)+pow(gy,2)+pow(gz,2),0.5); 
  Serial.print("Angle Change = ");
  Serial.println(angleChange);
  delay(2000);  

}

//SMS function and location tracking
void SendMessage(String message){  
  Serial.println("Setting the GSM in text mode");
  GSMserial.print("AT+CMGF=1\r");
  delay(2000);
  Serial.println("Sending SMS to the desired phone number!");
  GSMserial.print("AT+CMGS=\"26878047123\"\r");
  
  message += “AT+CIPGSMLOC”;
  GSMserial.print(message); // SMS Text
  delay(200);
  Serial.println("SMS sent");
  delay(2000);
 
  GSMserial.print((char)26); // ASCII code of CTRL+Z
  delay(2000);
}


void loop() {
  
  }//empty loop function 
