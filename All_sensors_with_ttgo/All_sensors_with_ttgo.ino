// Your phone number to send SMS: + (plus sign) and country code, for Portugal +351, followed by phone number

#define SMS_TARGET  "+8801558428977"
float tempThreshold1 = 105.0;
float tempThreshold2 = 55.0;
int alcoholthreshold = 125;
// Flag variable to keep track if alert SMS was sent or not
bool smsSent = false;


// Configure TinyGSM library
#define TINY_GSM_MODEM_SIM800      // Modem is SIM800
#define TINY_GSM_RX_BUFFER   1024  // Set RX buffer to 1Kb

//mq3
#define Sober 30   // Define max value that we consider sober
#define Drunk 125   // Define min value that we consider drunk

//ultrasonic
#define LED1 13
#define LED2 32
#define distThresh1 90
const int trigPin1 = 14;
const int echoPin1 = 12;
const int trigPin2 = 0;
const int echoPin2 = 15;

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <TinyGsmClient.h>
#include <OneWire.h>
#include <ThingsBoard.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include "ESPAsyncWebServer.h"

// Your GPRS credentials
// Leave empty, if missing user or pass
const char apn[]  = "blweb";
//const char user[] = "";
//const char pass[] = "";

//const char* ssid = "Mr Siddique";
//const char* password = "anamabed@022$";

// to understand how to obtain an access token
#define TOKEN "D4sDMpcFsjwaYhKhjauV" //Access token of device Display
char ThingsboardHost[] = "demo.thingsboard.io";




#include <Adafruit_ADS1X15.h> //Library of external adc
//#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 ads; // define the types of external adc
//Adafruit_ADS1015 ads;     /* Use this for the 12-bit version */
Adafruit_MPU6050 mpu;

const int  buttonPin = 18;    // the pin that the pushbutton is attached to
#define LED 19
// Variables will change:
int buttonPushCounter = 0;   // counter for the number of button presses
int buttonState = 0;         // current state of the button
int lastButtonState = 0;     // previous state of the button


// TTGO T-Call pins
#define MODEM_RST            5
#define MODEM_PWKEY          4
#define MODEM_POWER_ON       23
#define MODEM_TX             27
#define MODEM_RX             26
#define I2C_SDA              21
#define I2C_SCL              22

// Set serial for debug console (to Serial Monitor, default speed 115200)
#define SerialMon Serial
// Set serial for AT commands (to SIM800 module)
#define SerialAT  Serial1

// Define the serial console for debug prints, if needed
//#define DUMP_AT_COMMANDS

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

#define IP5306_ADDR          0x75
#define IP5306_REG_SYS_CTL0  0x00

bool setPowerBoostKeepOn(int en) {
  Wire.beginTransmission(IP5306_ADDR);
  Wire.write(IP5306_REG_SYS_CTL0);
  if (en) {
    Wire.write(0x37); // Set bit1: 1 enable 0 disable boost keep on
  } else {
    Wire.write(0x35); // 0x37 is default reg value
  }
  return Wire.endTransmission() == 0;
}

TinyGsmClient espClient(modem);
PubSubClient client(espClient);

//WiFiClient wifiClient;
//PubSubClient client(wifiClient);
//int status = WL_IDLE_STATUS;

// Set to true, if modem is connected
bool modemConnected = false;

float temp, temp1, temp2; // variable to store temperature value
int alch;
// defines variables
long duration1;
int distance1;
long duration2;
int distance2;

// Initialize GSM client
///TinyGsmClient espClient(modem);
// Initialize ThingsBoard instance
///ThingsBoard tb(espClient);

int channel = 0;


void setup() {
  // Set console baud rate
  SerialMon.begin(115200);

  // Keep power when running from battery
  Wire.begin(I2C_SDA, I2C_SCL);
  bool isOk = setPowerBoostKeepOn(1);
  SerialMon.println(String("IP5306 KeepOn ") + (isOk ? "OK" : "FAIL"));

  // Set modem reset, enable, power pins
  pinMode(MODEM_PWKEY, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);
  digitalWrite(MODEM_PWKEY, LOW);
  digitalWrite(MODEM_RST, HIGH);
  digitalWrite(MODEM_POWER_ON, HIGH);
  pinMode(LED, OUTPUT);
  // initialize the button pin as a input:
  pinMode(buttonPin, INPUT);
  pinMode(trigPin1, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin1, INPUT); // Sets the echoPin as an Input
  pinMode(trigPin2, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin2, INPUT); // Sets the echoPin as an Input
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);

  // Set GSM module baud rate and UART pins
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(3000);

  // Restart SIM800 module, it takes quite some time
  // To skip it, call init() instead of restart()
  SerialMon.println("Initializing modem...");
  // modem.restart();
  modem.init();
  // use modem.init() if you don't need the complete restart

  String modemInfo = modem.getModemInfo();
  Serial.print(F("Modem: "));
  Serial.println(modemInfo);

  //mobile data connection
  SerialMon.print("Connecting to APN: ");
  SerialMon.print(apn);
  if (!modem.gprsConnect(apn)) {
    SerialMon.println(" fail");
  }
  else
    SerialMon.println(" OK");
  client.setServer( ThingsboardHost, 1883 );

  /*WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
    delay(200);
    Serial.print(".");
    }
    Serial.println("");
    Serial.print("connected to ");
    Serial.println(ssid);
    client.setServer( ThingsboardHost, 1883 );*/


  delay(1000);


  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
    case MPU6050_RANGE_2_G:
      Serial.println("+-2G");
      break;
    case MPU6050_RANGE_4_G:
      Serial.println("+-4G");
      break;
    case MPU6050_RANGE_8_G:
      Serial.println("+-8G");
      break;
    case MPU6050_RANGE_16_G:
      Serial.println("+-16G");
      break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
    case MPU6050_RANGE_250_DEG:
      Serial.println("+- 250 deg/s");
      break;
    case MPU6050_RANGE_500_DEG:
      Serial.println("+- 500 deg/s");
      break;
    case MPU6050_RANGE_1000_DEG:
      Serial.println("+- 1000 deg/s");
      break;
    case MPU6050_RANGE_2000_DEG:
      Serial.println("+- 2000 deg/s");
      break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
    case MPU6050_BAND_260_HZ:
      Serial.println("260 Hz");
      break;
    case MPU6050_BAND_184_HZ:
      Serial.println("184 Hz");
      break;
    case MPU6050_BAND_94_HZ:
      Serial.println("94 Hz");
      break;
    case MPU6050_BAND_44_HZ:
      Serial.println("44 Hz");
      break;
    case MPU6050_BAND_21_HZ:
      Serial.println("21 Hz");
      break;
    case MPU6050_BAND_10_HZ:
      Serial.println("10 Hz");
      break;
    case MPU6050_BAND_5_HZ:
      Serial.println("5 Hz");
      break;
  }

  Serial.println("");
  delay(100);


  digitalWrite(LED, HIGH);// turn the LED off.(Note that LOW is the voltage level but actually
  //the LED is on; this is because it is acive low on the ESP8266.
  //delay(100);
  Serial.println("MQ3 warming up!");
  delay(3000);
  digitalWrite(LED, LOW);
  //ultrasonic
  digitalWrite(LED1, HIGH);
  digitalWrite(LED2, HIGH);
  delay(5000);
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);


  ads.begin();
  ledcSetup(channel, 2000, 8);
  ledcAttachPin(2, channel);//buzzer
}

void loop()
{
  if ( !client.connected() )
  {
    reconnect();
  }
  SensorData();

  delay(1000);
}

void SensorData() {
  //delay(1);
  int16_t adc0, adc1; // 16 bit interger to store output of analog channel zero

  // read the pushbutton input pin:
  buttonState = digitalRead(buttonPin);
  //delay(5000);
  if (buttonState != lastButtonState) {
    // if the state has changed, increment the counter
    if (buttonState == LOW) {
      // if the current state is HIGH then the button went from off to on:
      buttonPushCounter++;
      Serial.println("on");
      Serial.print("number of button pushes: ");
      Serial.println(buttonPushCounter);
    } else {
      // if the current state is LOW then the button went from on to off:
      Serial.println("off");
    }
    // Delay a little bit to avoid bouncing
    delay(100);
  }
  // save the current state as the last state, for next time through the loop
  lastButtonState = buttonState;
  //Serial.println(lastButtonState);

  // this section displays value of mq3 on serial monitor

  delay(100);
  if (buttonPushCounter > 0) {
    Serial.print("Button has been pushed ");
    Serial.print(buttonPushCounter);
    Serial.println(" Time");

    delay(10);
    //int16_t adc1; // 16 bit interger to store output of analog channel zero
    adc1 = ads.readADC_SingleEnded(0); // read AN0 values
    alch = (adc1 * 0.1875) / 10; // convert ADC value into voltage
    Serial.print("MQ3 Analog Value: ");
    Serial.print(alch);
    // delay(1000);
    if (alch < Sober) {
      digitalWrite ( LED , LOW );       //  turns the LED off
      ledcWriteTone(channel, 0);        //buzzer off
      Serial.println("  |  Status: Stone Cold Sober");
    } else if (alch >= Sober && alch < Drunk) {
      digitalWrite ( LED , LOW );      //  turns the LED off
      //delay(100);
      ledcWriteTone(channel, 0);        //buzzer off
      Serial.println("  |  Status: Drinking but within legal limits");
    } else {
      digitalWrite ( LED , HIGH );    // turns the LED on
      ledcWriteTone(channel, 1000);
      Serial.println("  |  Status: DRUNK");
    }

    //alert message
    if ((alch > alcoholthreshold) && !smsSent) {
      String smsMessage = String("Alcohol level above threshold: ") +
                          String(alch) + String(". You shouldn't drive right now!!");
      if (modem.sendSMS(SMS_TARGET, smsMessage)) {
        SerialMon.println(smsMessage);
        smsSent = true;
      } else {
        SerialMon.println("SMS failed to send");
      }

    }
    else if ((alch < alcoholthreshold) && !smsSent) {
      String smsMessage = String("You are ready to drive ");
      if (modem.sendSMS(SMS_TARGET, smsMessage)) {
        SerialMon.println(smsMessage);
        smsSent = true;
      } else {
        SerialMon.println("SMS failed to send");
      }
      //modem.sendSMS(SMS_TARGET, smsMessage);
      //SerialMon.println(smsMessage);
    }
  }
  else {

    String smsMessage = String("Alert!! Wear your helmet.");
    if (!smsSent && (modem.sendSMS(SMS_TARGET, smsMessage))) {
      SerialMon.println(smsMessage);
      smsSent = true;
    } else {
      SerialMon.println("SMS failed to send");
    }
    //modem.sendSMS(SMS_TARGET, smsMessage);
    //SerialMon.println(smsMessage);
  }


  delay(5000);

  //temperature
  adc0 = ads.readADC_SingleEnded(1); // read ANO values
  temp = (adc0 * 0.1875) / 1000; // convert ADC value into voltage
  temp1 = temp * 100; // converts voltage into temperature 10mv=1C
  temp2 = (1.8 * temp1 + 32);

  // this section displays value of adc and temperature on serial monitor
  Serial.print("AIN0: ");
  Serial.print(adc0);
  Serial.print("\tTemperature in Celsius: ");
  Serial.print(temp1);
  Serial.println("\t°C ");

  Serial.print("\t\tTemperature in Farenheit: ");
  Serial.print(temp2);
  Serial.println("\t°F ");
  Serial.println();

  //delay(100);
  //yield();

  // Check if temperature is above threshold and if it needs to send the SMS alert
  /* if ((temp1 > tempThreshold1) && !smsSent) {
     String smsMessage = String("Temperature above threshold: ") +
                         String(temp1) + String("C");
     if (modem.sendSMS(SMS_TARGET, smsMessage)) {
       SerialMon.println(smsMessage);
       smsSent = true;
     }
     else {
       SerialMon.println("SMS failed to send");
     }
    }
    // Check if temperature is below threshold and if it needs to send the SMS alert
    else if ((temp1 < tempThreshold2) && !smsSent) {
     String smsMessage = String("Temperature below threshold: ") +
                         String(temp1) + String("C");
     if (modem.sendSMS(SMS_TARGET, smsMessage)) {
       SerialMon.println(smsMessage);
       smsSent = true;
     }
     else {
       SerialMon.println("SMS failed to send");
     }
    }
    delay(2000);*/


  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /*//Gyroscope sensor deviation
    float accXerror = 0.11;
    float accYerror = -0.40;
    float accZerror = -10.18;
    //Gyroscope sensor deviation
    float rotXerror = 0.01;
    float rotYerror = 0.03;
    float rotZerror = 0.00;
  */
  /* Print out the values */
  Serial.print("Acceleration X: ");
  //Serial.println(a.acceleration.x);
  //a.acceleration.x-=accXerror;
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s²");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("");

  delay(5000);

  //DISTANCE
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(2);

  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration1 = pulseIn(echoPin1, HIGH);

  // Calculating the distance
  distance1 = (duration1 * 0.034) / 2;
  // Prints the distance on the Serial Monitor
  Serial.print("Distance (Right): ");
  Serial.print(distance1);
  Serial.println("cm");

  delayMicroseconds(2);

  //Left Ultrasonic Data
  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);

  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration2 = pulseIn(echoPin2, HIGH);

  // Calculating the distance
  distance2 = (duration2 * 0.034) / 2;
  // Prints the distance on the Serial Monitor
  Serial.print("Distance (Left): ");
  Serial.print(distance2);
  Serial.println("cm");

  if (distance1 < distThresh1 ) {
    digitalWrite ( LED1 , HIGH );       //  turns the LED on
    Serial.println("There is a vehicle near your RIGHT side. Maintain a proper distance.");
  }
  else if ( distance2 < distThresh1) {
    digitalWrite ( LED2 , HIGH );       //  turns the LED on
    Serial.println("There is a vehicle near your LEFT side. Maintain a proper distance.");
  }
  else {
    digitalWrite ( LED1 , LOW );    // turns the LED off
    digitalWrite ( LED2 , LOW );    // turns the LED off

  }

  delay(5000);





  // Prepare a JSON payload string
  String payload = "{";
  payload += "\"Temp. (°C) \":"; payload += temp1;
  payload += ",";
  payload += "\"Alcohol Level \":"; payload += alch;
  payload += ",";
  payload += "\"Distance Right(cm)\":"; payload += distance1;
  payload += ",";
  payload += "\"Distance Left(cm)\":"; payload += distance2;
  payload += ",";
  payload += "\"AccX (m/s²) \":"; payload += (a.acceleration.x);
  payload += ",";
  payload += "\"AccY (m/s²) \":"; payload += (a.acceleration.y);
  payload += ",";
  payload += "\"AccZ (m/s²) \":"; payload += (a.acceleration.z);
  payload += ",";
  payload += "\"RotX (rad/s) \":"; payload += (g.gyro.x);
  payload += ",";
  payload += "\"RotY (rad/s) \":"; payload += (g.gyro.y);
  payload += ",";
  payload += "\"RotZ (rad/s) \":"; payload += (g.gyro.z);
  payload += "}";

  char attributes[2000];
  payload.toCharArray( attributes, 2000 );
  if ( client.publish( "v1/devices/me/telemetry", attributes)) {
    Serial.println( attributes );
  }

  delay(3000);



}



void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    if (!modemConnected) {
      Serial.print(F("Waiting for network..."));
      if (!modem.waitForNetwork()) {
        Serial.println(" fail");
        delay(1000);
        return;
      }
      Serial.println(" OK");
      Serial.print(F("Connecting to "));
      Serial.print(apn);
      if (!modem.gprsConnect(apn)) {
        Serial.println(" fail");
        delay(1000);
        return;
      }

      modemConnected = true;
      Serial.println(" OK");
    }

    Serial.print("Connecting to ThingsBoard node ...");
    // Attempt to connect (clientId, username, password)
    if ( client.connect("ESP32SIM800L", TOKEN, NULL) ) {
      Serial.println( "[DONE]" );
    } else {
      Serial.print( "[FAILED] [ rc = " );
      Serial.println( " : retrying in 5 seconds]" );
      delay( 500 );
    }
  }

  /*while (!client.connected()) {
    status = WiFi.status();
    if ( status != WL_CONNECTED) {
      WiFi.begin(ssid, password);
      while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
      }
      Serial.println("Connected to AP");
    }
    Serial.print("Connecting to ThingsBoard node ...");
    // Attempt to connect (clientId, username, password)
    if ( client.connect("ESP32Client-", TOKEN, NULL) ) {
      Serial.println( "[DONE]" );
    } else {
      Serial.print( "[FAILED] [ rc = " );
      Serial.println( " : retrying in 5 seconds]" );
      delay( 500 );
    }
    }*/
}
