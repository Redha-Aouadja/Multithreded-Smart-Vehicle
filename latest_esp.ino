 #include <WiFi.h>



#include "BluetoothSerial.h"  //Motors
char DATA = ' ';              //Motors
int front=26;                 //Motors
int back=25;                  //Motors
int front2=33;                //Motors
int back2=32;                 //Motors

  #include "BluetoothSerial.h"  //Controls
  BluetoothSerial SerialBT;     //Controls
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#include <Adafruit_LSM6DS33.h>  //DOF
#define LSM_CS 5                //DOF
#define LSM_SCK 18              //DOF
#define LSM_MISO 19             //DOF
#define LSM_MOSI 23             //DOF
Adafruit_LSM6DS33 lsm6ds33;     //DOF


#include <ESP32Servo.h>     //Servos
Servo myservo;              //Servos
Servo myservo2;             //Servos
int servoPin = 16;          //Servos
int servoPin2 = 17;         //Servos

#define echoPin 14        //Sonic
#define trigPin 27        //Sonic
#define echoPin2 13       //Sonic
#define trigPin2 12       //Sonic
long duration;            //Sonic
int distance;             //Sonic
long duration2;           //Sonic
int distance2;            //Sonic


 /// #include <WiFi.h>   //firebase löogging
  //#include <Firebase_ESP_Client.h>
  //#include <Firebase.h>
//#include <Firebase.h>
//#include <FirebaseFS.h>

//Provide the token generation process info.
#include "addons/TokenHelper.h"
//Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"

// Insert your network credentials
#define WIFI_SSID "hidden"
#define WIFI_PASSWORD "hidden"

// Insert Firebase project API Key
#define API_KEY "hidden"

// Insert RTDB URLefine the RTDB URL */
#define DATABASE_URL "hidden" 

//Define Firebase Data object
FirebaseData fbdo;

FirebaseAuth auth;
FirebaseConfig config;

unsigned long sendDataPrevMillis = 0;
int count = 0;
bool signupOK = false;



//////////////////////////////Multicore Tasks
TaskHandle_t ultrasonic_t1;
TaskHandle_t dof_t2;
//TaskHandle_t logging_t3;
/////////////////////////////


void setup() {

////////////////////////////                /* MOTOR DRIVE SETUP*/
  pinMode(front, OUTPUT);
  pinMode(back, OUTPUT);
  pinMode(front2, OUTPUT);
  pinMode(back2, OUTPUT);
  SerialBT.begin("HULK32");
  //////////////////////////

  Serial.begin(115200);
  ///////////////////////////////////////wifi/////////////
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD); 
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED){
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();
  
////////////////////////////////////////////////  /* SERVO SETUP*/
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myservo.setPeriodHertz(50);
  // standard 50 hz servo
  myservo.attach(servoPin, 1000, 2000);
  myservo2.attach(servoPin2, 1000, 2000);
  //////////////////////////

//  //////////////////////////////////////////////////////  Sonic setup
//  Serial.begin(115200);
  pinMode(trigPin, OUTPUT); 
  pinMode(echoPin, INPUT);
  pinMode(trigPin2, OUTPUT); 
  pinMode(echoPin2, INPUT);


  //////////////////////////////////////////////            Initiate multocore tasks
   xTaskCreatePinnedToCore(
                    sonic1code,   /* Task function. */
                    "ultasonic reading",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &ultrasonic_t1,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */                  
  delay(500); 
    //////////////////////////////////////////////            Initiate second multocore tasks
   xTaskCreatePinnedToCore(
                    dofcode,   /* Task function. */
                    "dof reading",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &dof_t2,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */                  
  delay(500); 
  
  
  Serial.println("Adafruit LSM6DS33 test!");

  //if (!lsm6ds33.begin_I2C()) {
    // if (!lsm6ds33.begin_SPI(LSM_CS)) {
     if (!lsm6ds33.begin_SPI(LSM_CS, LSM_SCK, LSM_MISO, LSM_MOSI)) {
    Serial.println("Failed to find LSM6DS33 chip");
    while (1) {
      delay(10);
    }
  }

  Serial.println("LSM6DS33 Found!");

  // lsm6ds33.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  Serial.print("Accelerometer range set to: ");
  switch (lsm6ds33.getAccelRange()) {
  case LSM6DS_ACCEL_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case LSM6DS_ACCEL_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case LSM6DS_ACCEL_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case LSM6DS_ACCEL_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }

  // lsm6ds33.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
  Serial.print("Gyro range set to: ");
  switch (lsm6ds33.getGyroRange()) {
  case LSM6DS_GYRO_RANGE_125_DPS:
    Serial.println("125 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_250_DPS:
    Serial.println("250 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_500_DPS:
    Serial.println("500 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_1000_DPS:
    Serial.println("1000 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_2000_DPS:
    Serial.println("2000 degrees/s");
    break;
  case ISM330DHCX_GYRO_RANGE_4000_DPS:
    break; // unsupported range for the DS33
  }

  // lsm6ds33.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
  Serial.print("Accelerometer data rate set to: ");
  switch (lsm6ds33.getAccelDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial.println("6.66 KHz");
    break;
  }

  // lsm6ds33.setGyroDataRate(LSM6DS_RATE_12_5_HZ);
  Serial.print("Gyro data rate set to: ");
  switch (lsm6ds33.getGyroDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial.println("6.66 KHz");
    break;
  }
  lsm6ds33.configInt1(false, false, true); // accelerometer DRDY on INT1
  lsm6ds33.configInt2(false, true, false); // gyro DRDY on INT2
///////////////////////////////////////////////////////////////////////// END OF 6DOF

////////////////////////////////FIREBASE//////////////////////

  /* Assign the api key (required) */
  config.api_key = API_KEY;

  /* Assign the RTDB URL (required) */
  config.database_url = DATABASE_URL;

  /* Sign up */

}

void loop() {
    motorDrive();
    
  }

void sonic1code( void * pvParameters ){             //////// ULTRASONIC FUNCTION

      for(;;){
        digitalWrite(trigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);
        duration = pulseIn(echoPin, HIGH);
        distance = duration * 0.034 / 2; 
        Serial.print("Distance: ");
        Serial.print(distance);
        Serial.println(" cm");
        
        digitalWrite(trigPin2, LOW);
        delayMicroseconds(2);
        digitalWrite(trigPin2, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin2, LOW);
        duration2 = pulseIn(echoPin2, HIGH);
        distance2 = duration2 * 0.034 / 2; 
        Serial.print("Distance2: ");
        Serial.print(distance2);
        Serial.println(" cm");
        
        Serial.print("Ultrasonics running in Core: ");
        Serial.println(xPortGetCoreID());

        delay(2000);

        } 
      
  
  }
void dofcode( void * pvParameters ){             //////// ULTRASONIC FUNCTION

      for(;;){
                ///////////////////////////////////////////  6DOF ROUTINE

        sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  lsm6ds33.getEvent(&accel, &gyro, &temp);

  Serial.print("\t\tTemperature ");
  Serial.print(temp.temperature);
  Serial.println(" deg C");

  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("\t\tAccel X: ");
  Serial.print(accel.acceleration.x);
  Serial.print(" \tY: ");
  Serial.print(accel.acceleration.y);
  Serial.print(" \tZ: ");
  Serial.print(accel.acceleration.z);
  Serial.println(" m/s^2 ");

  /* Display the results (rotation is measured in rad/s) */
  Serial.print("\t\tGyro X: ");
  Serial.print(gyro.gyro.x);
  Serial.print(" \tY: ");
  Serial.print(gyro.gyro.y);
  Serial.print(" \tZ: ");
  Serial.print(gyro.gyro.z);
  Serial.println(" radians/s ");
  Serial.println();

  delay(2000);
    if (Firebase.signUp(&config, &auth, "", "")){
    Serial.println("ok");
    signupOK = true;
  }
  else{
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }

  /* Assign the callback function for the long running token generation task */
  config.token_status_callback = tokenStatusCallback; //see addons/TokenHelper.h
  
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
  if (Firebase.ready() && signupOK && (millis() - sendDataPrevMillis > 15000 || sendDataPrevMillis == 0)){
    sendDataPrevMillis = millis();
    // Write an Int number on the database path test/int
    if (Firebase.RTDB.setInt(&fbdo, "Distance 1", distance)){
      Serial.println("PASSED");
      Serial.println("PATH: " + fbdo.dataPath());
      Serial.println("TYPE: " + fbdo.dataType());
    }
    else {
      Serial.println("FAILED");
      Serial.println("REASON: " + fbdo.errorReason());
    }
    
    
    if (Firebase.RTDB.setFloat(&fbdo, "Distance 2", distance2)){
      Serial.println("PASSED");
      Serial.println("PATH: " + fbdo.dataPath());
      Serial.println("TYPE: " + fbdo.dataType());
    }
    else {
      Serial.println("FAILED");
      Serial.println("REASON: " + fbdo.errorReason());
    }
    delay(2000);
        } 
      else{Serial.println("not working");}
  
  }}

void motorDrive(){                    ///////////////////// MOTORS FUNCTION
      if (SerialBT.available()) {
      DATA = (SerialBT.read());
      //Serial.print(DATA); 
     if (DATA == 'G') {
        digitalWrite(back,HIGH);
        digitalWrite(back2,HIGH);
      }
      if (DATA == 'S'){
       digitalWrite(back2,LOW); 
       digitalWrite(back,LOW);
       digitalWrite(front,LOW);
       digitalWrite(front2,LOW);
      }
       if (DATA == 'R') {
        digitalWrite(front,HIGH);
        //Serial.print("it is R");
      }
       if (DATA == 'F') {
        digitalWrite(front,HIGH);
        digitalWrite(front2,HIGH);
      }
    
       if (DATA == 'L') {
        digitalWrite(front2,HIGH);
      }

 if (DATA == 'E') {
        digitalWrite(front2,HIGH);
        //Serial.print("it is E");

      }
       if (DATA == 'Q') {
        digitalWrite(front,HIGH);
        //Serial.print("it is Q");

      }
      if (DATA == 'Z') {
        digitalWrite(back,HIGH);
        //Serial.print("it is Z");

      }
      if (DATA == 'C') {
        digitalWrite(back2,HIGH);
        //Serial.print("it is Q");
      }
      if (DATA == 'M'){
        myservo.write(10);
        myservo2.write(10);
      }
      if (DATA == 'N'){
        myservo.write(70);
        myservo2.write(70);
      }
    }
}
