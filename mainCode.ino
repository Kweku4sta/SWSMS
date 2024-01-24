

#include <Arduino.h>
#if defined(ESP32)
#include <WiFi.h>
#include <FirebaseESP32.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#include <FirebaseESP8266.h>
#elif defined(ARDUINO_RASPBERRY_PI_PICO_W)
#include <WiFi.h>
#include <FirebaseESP8266.h>
#include <FirebaseJson.h>
#endif

// Provide the token generation process info.
#include <addons/TokenHelper.h>

// Provide the RTDB payload printing info and other helper functions.
#include <addons/RTDBHelper.h>

/* 1. Define the WiFi credentials */
#define WIFI_SSID "Kweku'sVirus"
#define WIFI_PASSWORD "ansah12345"

// #define WIFI_SSID "KD25"
// #define WIFI_PASSWORD "Ducati1234"

// the distance declare
#define echoPin 26   
#define trigPin 27

#define flowPin 13      // Flow meter pulse input pin
#define FLOW_SENSOR_CALIBRATION_FACTOR 2.663


#define RELAY_PIN 21
#define RELAY_PIN_OFF 19


#define BUTTON_PIN 25
// the turbidity
#define TURBIDITY_SENSOR_PIN 32
#define PH_SENSOR_PIN 33
#define TDS_SENSOR_PIN 35

volatile float totalFlow = 0;
volatile int pulseCount = 0; // Declare pulseCount as volatile
unsigned long previousMeasurementTime = 0;
const unsigned long measurementInterval = 200;



void IRAM_ATTR increase() {
  pulseCount++;
}




// For the following credentials, see examples/Authentications/SignInAsUser/EmailPassword/EmailPassword.ino

/* 2. Define the API Key */
#define API_KEY "AIzaSyASOIRHivBMo5exf1yVRztCFXBM61h8SAI"

/* 3. Define the RTDB URL */
#define DATABASE_URL "https://watersmart-b66aa-default-rtdb.firebaseio.com/" //<databaseName>.firebaseio.com or <databaseName>.<region>.firebasedatabase.app

/* 4. Define the user Email and password that alreadey registerd or added in your project */
// #define USER_EMAIL "kweku4sta@gmail.com"
// #define USER_PASSWORD "#Fourstar1"

// Define Firebase Data object
FirebaseData fbdo;
int a, b, x, y;
FirebaseAuth auth;
FirebaseConfig config;

int userID = -1;
float userVolume = 0;
bool tapFlowing = false;
bool stopButtonPressed = false;

unsigned long sendDataPrevMillis = 0;

unsigned long count = 0;
int turbidityValue = 0;
int distanceValue = 0;
float tdsValue = 0.00;
float phValue = 0.00;


const float tdsCalibrationFactor = 0.5;
float calibrationValue = 26 - 0.05;
int pHValue = 0;
unsigned long int avgval;
int buffer_arr[10], temp;
float ph_act;



  int measureDistance() {
  long duration, distance;

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = duration / 58.2;

  return distance;
}

// float measureVolume() {
//   unsigned long currentMillis = millis();
//   while (millis() - currentMillis < measurementInterval) {
//     // Wait for the measurement interval to pass
//   }

//   // Disable the interrupt during calculations
//   detachInterrupt(digitalPinToInterrupt(flowPin));

//   float volume = FLOW_SENSOR_CALIBRATION_FACTOR * pulseCount;
//    Serial.print("pulse count: ");
//   Serial.print(pulseCount);
//   // pulseCount = 0; // Reset pulse count after calculation
//   totalFlow += volume;

//   // Print the total accumulated flow
//   Serial.print("Total Flow: ");
//   Serial.print(totalFlow);
//   Serial.println(" ml");
  

//   // Re-enable the interrupt after calculations
//   attachInterrupt(digitalPinToInterrupt(flowPin), increase, RISING);

//   // Return the calculated volume
//   return volume;
// }


void setup()
{

  Serial.begin(115200);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  Serial.printf("Firebase Client v%s\n\n", FIREBASE_CLIENT_VERSION);

  /* Assign the api key (required) */
  config.api_key = API_KEY;

  /* Assign the user sign in credentials */
  // auth.user.email = USER_EMAIL;
  // auth.user.password = USER_PASSWORD;

  /* Assign the RTDB URL (required) */
  config.database_url = DATABASE_URL;

  /* Assign the callback function for the long running token generation task */
  config.token_status_callback = tokenStatusCallback; // see addons/TokenHelper.h

 
  config.api_key = API_KEY;

  config.database_url = DATABASE_URL;
  
  Firebase.begin(DATABASE_URL, API_KEY);
  Firebase.reconnectWiFi(true);

  Firebase.setDoubleDigits(5);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
   pinMode(flowPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(flowPin), increase, RISING);
   pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH);
   pinMode(RELAY_PIN_OFF, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  digitalWrite(BUTTON_PIN, LOW);
  Serial.println("Enter your User ID (integer): ");
  
}
 void fetchUserVolume() {
  // String userVolumePath = String("/test/user10");
  // String userVolumePath = "/test/user" + String(userID);
  // String userVolumePath = String("/test/uservolume" + String(userID) + "/volume/volume");
  String userVolumePath = String("/test/uservolume/" + String(userID) + "/volume/volume");


  if (Firebase.getFloat(fbdo, userVolumePath)) {
     userVolume = fbdo.to<float>();
    Serial.print("User Volume: ");
    Serial.println(userVolume);
    if(userVolume >10){
      digitalWrite(RELAY_PIN,LOW);
      tapFlowing = true;
      delay(10000);
      digitalWrite(RELAY_PIN,HIGH);
      delay(800);
      digitalWrite(RELAY_PIN_OFF,HIGH);
      
    }
    delay(5000);
  } else {
    Serial.println("User not found in our database");
    userID = -1;
  }
}


void measureAndSendFlow() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMeasurementTime >= measurementInterval) {
    // Flow measurement and volume calculation
    float volume = FLOW_SENSOR_CALIBRATION_FACTOR * pulseCount;
    totalFlow += volume;
    pulseCount = 0;
    Serial.println("checking the global scope");
    Serial.println(userVolume);
    Serial.print("Total Flow: ");
    Serial.print(totalFlow);
    Serial.println(" ml");


  String userVolumePath = "/test/uservolume/" + String(userID) + "/volume/volume";

  


    float newVolume = userVolume - volume;
    Firebase.setFloat(fbdo, userVolumePath, newVolume);
    userVolume = newVolume;
   

    
    Serial.print("Remaining User Volume: ");
    Serial.print(userVolume);
    Serial.println(" ml");


//  update the total volume in the cloud
  String userTotalVolumePath = "/test/usertotalvolume/" + String(userID);
  
  
    if (Firebase.getFloat(fbdo, userTotalVolumePath)) {
   float   userVolumeInCloud = fbdo.to<float>();
      Serial.println("Total volume in the cloud");
  Serial.println(userVolumeInCloud);
  float updatedTotalVolume = volume + userVolumeInCloud;
  Firebase.setFloat(fbdo, userTotalVolumePath, updatedTotalVolume);
   
    }
  



    if (userVolume <= 0 || stopButtonPressed) {
      digitalWrite(RELAY_PIN, HIGH);
      delay(800);
      // TRIGER THE OFF PIN 
      digitalWrite(RELAY_PIN_OFF,LOW);
      delay(10000);
       digitalWrite(RELAY_PIN_OFF,HIGH);
      tapFlowing = false;
      totalFlow = 0;
      Serial.println("Tap stopped.");
      userID= -1;
      
    }

    previousMeasurementTime = currentMillis;
  }
}


int measureTurbidity() {
  int sensorValue = analogRead(TURBIDITY_SENSOR_PIN);
  int turbidity = map(sensorValue, 0, 4095, 0, 100);
  return turbidity;
}




void loop()
{   
  // int userID = 10;
    x=random(0,9);
    y=random(10,19);

      FirebaseJson json;

  // Add data to the JSON object
  json.add("waterLevel", 500);
  json.add("volume", 2000);
  json.add("waterConsumed", x);


  


  // Firebase.ready() should be called repeatedly to handle authentication tasks.

  if (Firebase.ready() && (millis() - sendDataPrevMillis > 15000 || sendDataPrevMillis == 0))
  {
    sendDataPrevMillis = millis();
    char path[50];
    snprintf(path, sizeof(path), "/test/volumeUsed/user%d", userID);


    Firebase.setInt(fbdo, path, x);
    Firebase.setInt(fbdo, "/test/turbidity", turbidityValue);
    Firebase.setInt(fbdo, "/test/waterlevel", distanceValue);
    Firebase.setFloat(fbdo, "/test/ph", 8.6);
    Firebase.setInt(fbdo, "/test/tds", tdsValue);


    
    Firebase.updateNode(fbdo,F("/test/json"), json);
    
     count++;
     
  }
  turbidityValue = measureTurbidity();
     distanceValue = 50 - measureDistance( );
     float ph  =   analogRead(PH_SENSOR_PIN);
     phValue = constrain(ph*0.5,0.0,14.0);
    
  //     int ph = analogRead(pHpin); // Read the raw value from the pH sensor
  // float voltage = pHvalue * (3.3 / 4095.0); // Convert the raw value to voltage
  // float pH = 3.5 * voltage + Offset; // Convert the voltage to pH value
  // phValue = constrain(pH, 0, 14);

   for (int i = 0; i < 10; i++) {
    buffer_arr[i] = analogRead(PH_SENSOR_PIN);
    delay(30);
  }
for (int i = 0; i < 9; i++) {
    for (int j = i + 1; j < 10; j++) {
      if (buffer_arr[i] > buffer_arr[j]) {
        temp = buffer_arr[i];
        buffer_arr[i] = buffer_arr[j];
      }
    }
  }

 avgval = 0;
  for (int i = 2; i < 8; i++)
    avgval += buffer_arr[i];

  float volt = (float)avgval * 5.0 / 1024 / 6;
  ph_act = -5.79 * volt + calibrationValue;
  float pHValue = constrain(ph_act, 0, 14);
//  Serial.print("pH Value: ");
//  Serial.print(phValue);






  int tds = analogRead(TDS_SENSOR_PIN);
  tdsValue = tds * 0.5;

  if (digitalRead(BUTTON_PIN) == HIGH) {
    stopButtonPressed = true;
    Serial.println("Button pressed");
    digitalWrite(RELAY_PIN_OFF,LOW);
  } else {
    stopButtonPressed = false;
    Serial.println("Button not pressed");
    
  }



if (userID == -1) {
    Serial.println("Enter your user id for me");
    delay(5000);
    // Ask for user ID once
    if (Serial.available() > 0) {
      String userInput = Serial.readStringUntil('\n');
      userInput.trim();
      userID = userInput.toInt();

      // Validate that the user ID is a positive integer
      if (userID <= 0) {
        Serial.println("Invalid User ID. Please enter a positive integer.");
        userID = -1; // Reset the user ID for re-entering
      } else {
        Serial.print("User ID set to: ");
        Serial.println(userID);
        userID = userInput.toInt();
        fetchUserVolume();
      }
    }
  } else {
    // Perform flow measurement and Firebase update
    measureAndSendFlow();
     
  }




}
