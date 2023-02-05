/***********************************************************************************************/
/*
/// Project: 
/// Description: 
/// Author: Phan Tuan Nhat
*/
/***********************************************************************************************/

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Adafruit_NeoPixel.h>
#include <MFRC522.h>
#include <SPI.h>
#include <HX711_ADC.h>
#include <TinyGPS++.h>
#include <EEPROM.h>


/*=========================================== GPIOs ===========================================*/ 
// Store ID of the user tag
#define EEPROM_SIZE                   12      
  
// MFRC522 RFID Reader    
#define SS_PIN                        5  
#define RST_PIN                       0
  
// Button
#define BUTTON_PIN                    4

// Buzzer   
#define BUZZER_PIN                    21
  
// WS2812 RGB LED   
#define PIXEL_PIN                     22      
#define NUM_OF_PIXEL                  9
  
// Ultrasonic Sensor Inside   
#define INSIDE_US015_TRIG_PIN         12     
#define INSIDE_US015_ECHO_PIN         13
  
// Ultrasonic Sensor Outside    
#define OUTSIDE_US015_TRIG_PIN        27     
#define OUTSIDE_US015_ECHO_PIN        14
  
// HX711 ADC Converter for load cell
#define HX711_CLK_PIN                 25
#define HX711_DOUT_PIN                26
  
// State of Lid   
#define SW_TOP_PIN                    35
//#define SW_BOTTOM_PIN                 34

// Ultraviolet Light
#define UV_LIGHT_PIN                  33

// Lock of the under door
#define LOCK_PIN                      32

// Control Motor
#define IN1_MOTOR_PIN                 2
#define IN2_MOTOR_PIN                 15


/*========================================== OBJECTS ==========================================*/ 
MFRC522 mfrc522(SS_PIN, RST_PIN);
MFRC522::MIFARE_Key userKey;
MFRC522::MIFARE_Key staffKey;
MFRC522::MIFARE_Key defaultKey;

Adafruit_NeoPixel pixels(NUM_OF_PIXEL, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

HX711_ADC LoadCell(HX711_DOUT_PIN, HX711_CLK_PIN);

TinyGPSPlus gps;


/*========================================= VARIABLES =========================================*/
// Information network and server
const char* ssid = "LED";
const char* password = "tuannhat";
const char* serverName = "http://envican.app/connect/data.php"; 


// Token code used to authenticate with server
const String token = "tuannhat";
// GPS data
double GPS_Latitude = 16.078248;
double GPS_Longitude = 108.242405;
String GPS_Date = "04/08/2022";
String GPS_Time = "1:44";    
unsigned long cycleToGetData = 0;    
// Amount garbage inside the bin (unit : %)
int garbageCapacity = 0;
// Garbage weight inside the bin (unit : kg)
float weight = 0;
// Enable to update data on server
bool updateData = false;
bool refreshData = false;

// RFID
enum Tag{ USER, STAFF, UNKNOWN};
Tag rfidTag;

byte userUID[4];
byte staffUID[4];
byte newUID[4];     
byte userAuthenticationKey[6] = {0x44, 0x61, 0x6e, 0x61, 0x6e, 0x67};    //String: "Danang"
byte staffAuthenticationKey[6] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06};   //String: "123456"

volatile unsigned long timeLidHasClosed;
volatile unsigned long timeLidHasOpened;
volatile unsigned long timeOutClosed;
volatile bool isOpening;

volatile bool updateUserTag = false;
volatile unsigned long myTime;

unsigned long timeNetworkStatus = 0;

bool unlockBottomDoor = false;
unsigned long timeUnlock = 0;

enum State{OPEN, CLOSE, UNDEFINED};
State lidState;


/*===================================== Function Prototypes ===================================*/
//Interrupt Service Routine
void IRAM_ATTR updateUserTag_ISR();
void IRAM_ATTR lidIsClosed_ISR();

boolean getTag(byte *);
boolean isUser();
boolean isStaff();
boolean encodeUserTag();
void printHexArray(byte *, byte);

void closedEffect();
void openEffect();
void networkStatusEffect(wl_status_t status);
void isReadyEffect(uint8_t wait);

void turnOnBuzzerSignal(int, int);

int getDistanceOutside();
int getDistanceInside();

void postDataToServer(String token, double latitude, double longitude, float weight, int garbageCapacity);
void postDataToServer(String token, double latitude, double longitude, float weight, int garbageCapacity, byte *staffUID);
void postRequest(String httpRequestData);

void closedProcedure();
void openProcedure();

/*========================================== SET UP ===========================================*/
void setup()
{
  /* Initialize Serial Ports*/
  Serial.begin(115200);
  Serial2.begin(9600);

  /* Set GPIOs */
  pinMode(INSIDE_US015_TRIG_PIN, OUTPUT);   
  pinMode(INSIDE_US015_ECHO_PIN, INPUT);

  pinMode(OUTSIDE_US015_TRIG_PIN, OUTPUT);   
  pinMode(OUTSIDE_US015_ECHO_PIN, INPUT);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), updateUserTag_ISR, FALLING);

  pinMode(SW_TOP_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SW_TOP_PIN), lidIsClosed_ISR, FALLING);

  //pinMode(SW_BOTTOM_PIN, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(SW_BOTTOM_PIN), updateUserTag_ISR, FALLING);

  pinMode(IN1_MOTOR_PIN, OUTPUT);
  pinMode(IN2_MOTOR_PIN, OUTPUT);
  pinMode(LOCK_PIN, OUTPUT);
  pinMode(UV_LIGHT_PIN, OUTPUT);
  
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);    // Turn off buzzer


  /* Initialize Neopixel strip object */
  pixels.begin();           
  pixels.clear();
  pixels.show();


  /* Initialize MFRC522 Module*/
  SPI.begin();              // Init SPI bus
  mfrc522.PCD_Init();       // Init MFRC522 card

  for(byte i = 0; i < 6; i++){
    userKey.keyByte[i] = userAuthenticationKey[i];
  }

  for(byte i = 0; i < 6; i++){
    staffKey.keyByte[i] = staffAuthenticationKey[i];
  }

  for(byte i = 0; i < 6; i++){
    defaultKey.keyByte[i] = 0xFF;
  }


  /* Get user UID from EEPROM */
  EEPROM.begin(EEPROM_SIZE);
  int eepromAddress = 0;
  for(int i = 0; i < 4; i++){
    userUID[i] = EEPROM.read(eepromAddress);
    eepromAddress += 1;
  }
  Serial.print(" User ID: ");
  printHexArray(userUID, 4);
  Serial.print('\n');


  /* Initialize for HX711 module */
  LoadCell.begin();
  float calibrationValue = 109.75;    // Set the calibration value in the sketch
  unsigned long stabilizingtime = 2000; // preciscion right after power-up can be improved by adding a few seconds of stabilizing time
  bool _tare = true; //set this to false if you don't want tare to be performed in the next step
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU > HX711 wiring and pin designations");
    while (1);
  }
  else {
    LoadCell.setCalFactor(calibrationValue); // set calibration value (float)
    Serial.println("Startup is complete");
  }


  /* Conect to Network */
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    networkStatusEffect(WiFi.status());
    Serial.println("Connecting to WiFi...");
    //delay(500);
  }
  Serial.println("Connected to the WiFi network");
  //Blinking LED to indicate ESP32 is connected to Network
  for(uint8_t x = 0; x < 4; x++){
    pixels.setPixelColor(4, pixels.Color(0,0,255));
    pixels.show();
    delay(200);
    pixels.setPixelColor(4, pixels.Color(0,0,0));
    pixels.show();
    delay(200);
  } 
  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP()); //show ip address when connected on serial monitor.
  delay(500);


  /* Get lid state */
  if(digitalRead(SW_TOP_PIN) == LOW){
    lidState = State::CLOSE;
    Serial.println("The lid of trash is closed");
  }else{
    lidState = State::OPEN;
    Serial.println("The lid of trash is open");
    closedProcedure();
  }

  isReadyEffect(100);
  Serial.println("------------------------------------------------------------");
  Serial.println(">> ESP32 is ready.");
  Serial.println("------------------------------------------------------------");

}


/*======================================= MAIN PROGRAM ========================================*/
void loop() 
{
  /* Connect to Internet, update data on server */
  if(WiFi.status()== WL_CONNECTED) {

    if(updateData == true) {
      postDataToServer(token, GPS_Latitude, GPS_Longitude, weight, garbageCapacity);
      updateData = false;
    }
  }

  /* Turn on LED to indicate the operation status*/
  if((millis() - timeNetworkStatus) > 20000)
  {
    networkStatusEffect(WiFi.status());

    if(WiFi.status() == WL_CONNECTED)
    {
      Serial.println("Wi-Fi connected!");
    }
    else
    {
      Serial.println("Wi-Fi disconnected!");
      Serial.println("Reconnecting to WiFi...");
      WiFi.disconnect();
      WiFi.reconnect();
    }
    timeNetworkStatus = millis();
  }


  /* If user's tag is lost, the Smart trash need to add a new user tag */
  if(updateUserTag == true ){
    Serial.println("\n -------Update User Tag-------");
    turnOnBuzzerSignal(1, 200);
    Serial.println("Step : Please put staff's tag onto RFID reader!");
    while((millis() - myTime) < 5000)
    { 
      if(getTag(newUID))
      {
        if(isStaff())
        {
          Serial.println(">> Staff tag is authenticated!");
          delay(500);
          Serial.println("Step 2: Please put white tag on RFID reader to create user's tag!");
          turnOnBuzzerSignal(1, 100);
          myTime = millis();
          while((millis() - myTime) < 5000)
          {    
            if(getTag(newUID))
            {
              if(encodeUserTag() == true)
              {
                // Store new UID into EEPROM 
                int eepromAddr = 0;
                for(int i = 0; i < 4; i++){
                  userUID[i] = newUID[i];
                  EEPROM.write(eepromAddr, userUID[i]);   
                  eepromAddr += 1;
                }
                EEPROM.commit();
                Serial.println(">> User's UID is saved in EEPROM");

                turnOnBuzzerSignal(2, 100);
                break; 
              }
            }
          }
          break; 
        }   
      }
    }
    Serial.println("--------End of Update--------");
    updateUserTag = false;
  }

  /* Read RFID tags */
  if(getTag(newUID)) 
  {
    if(isUser()) 
    {
      Serial.println(" [USER] Please! Give me the garbage");
      openProcedure();
      rfidTag = Tag::USER;
    }else if (isStaff())
    {
      Serial.println(" [STAFF] Please! Take garbage out");
      
      turnOnBuzzerSignal(2, 150);
      unlockBottomDoor = true;
      digitalWrite(LOCK_PIN, HIGH);
      timeUnlock = millis();

      rfidTag = Tag::STAFF;
      for(int i = 0; i < 4; i++){
        staffUID[i] = newUID[i];
      }
    }
    else
    {
      rfidTag = Tag::UNKNOWN;
      Serial.println(" [UNKNOW] Tag is invalid!");
    }
  }
  
  if(unlockBottomDoor)
  {
    if((millis() - timeUnlock) >= 5000)
    {
      unlockBottomDoor = false;
      digitalWrite(LOCK_PIN, LOW);
    }
  }


  if(digitalRead(SW_TOP_PIN) == HIGH)
  {
    lidState = State :: OPEN;
  }else
  {
    lidState = State :: CLOSE;
  }

  if(State :: OPEN == lidState && isOpening == true)
  {
      //Check, if no one is standing in front of the trash can during 10 seconds
    int distanceOutside;
    int timeRemainingToClose;

    distanceOutside = getDistanceOutside();
    Serial.print("Distance: ");
    Serial.println(distanceOutside);

    if(distanceOutside <= 70){
      timeOutClosed = millis();
    }

    timeRemainingToClose = 7 - ((millis() - timeOutClosed) / 1000);

    Serial.print("Lid of trash can is going to close after [ ");
    Serial.print(timeRemainingToClose);
    Serial.println(" ] seconds");

    if(timeRemainingToClose <= 0){
      closedProcedure();
      Serial.println("==> Close Procedure");     
    }
    delay(1000);
  }
  else if(State :: CLOSE == lidState)
  {
    
  }


  if(refreshData)
  {
    /*Get Amount of trash inside*/ 
    int amountOfTrash;
    for(int i = 0; i < 5; i++){
      amountOfTrash = getDistanceInside();
      delay(500);
    }
    Serial.print("Garbage in bin: ");
    Serial.println(amountOfTrash);   // Distance is usually 47 (cm)
    amountOfTrash -= 10;  // We need to subtract 4cm to keep the distance from the sensor to the garbage
    garbageCapacity = 100 - map(amountOfTrash, 0, 67, 0, 100);

    refreshData = false;
    updateData = true;
  }

  /* Get GPS (Global Position System) */
  if( (millis() - cycleToGetData) > 5000)
  {
    while (Serial2.available() > 0)
    {
      if (gps.encode(Serial2.read()))
      {
        // GPS location
        if(gps.location.isValid()){
          GPS_Latitude = gps.location.lat();
          GPS_Longitude = gps.location.lng();
        }else{
        }
        /*
        // GPS date
        if (gps.date.isValid())
        {
          GPS_Date = String(gps.date.month()) + "/" + String(gps.date.day()) + "/" + String(gps.date.year());
        }
        else{
        }
        // GPS time
        if (gps.time.isValid())
        {
          GPS_Time = String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second());
        }
        else{
        }
        */
      }
    }
    cycleToGetData = millis();
  }

  /* Get weight of trash */
  if(LoadCell.update()){
    weight = LoadCell.getData() / 1000.0 ;
  }

}


/*========================================= FUNCTIONS ========================================*/
/*
/// To enable the user tag updated function for trash can 
*/
void IRAM_ATTR updateUserTag_ISR() {
  updateUserTag = true;
  myTime = millis();
}


/*
///
*/
void IRAM_ATTR lidIsClosed_ISR()
{
  for(int i = 0 ; i < 3000; i++);
  if(! isOpening)
  {
    lidState = State :: CLOSE;
    refreshData = true;
    Serial.println(">> Lid is closed!!!");
  }

}


/*
/// 
*/
boolean getTag(byte *buffer) {
  // Reset the loop if no new card present on the sensor/reader.
  if ( ! mfrc522.PICC_IsNewCardPresent())
    return false;
  
  // Select one of the cards
  if ( ! mfrc522.PICC_ReadCardSerial())
    return false;

  for(byte i = 0; i < mfrc522.uid.size; i++){
    buffer[i] = mfrc522.uid.uidByte[i];
  }  
  return true;
}

/*
///  Check if tag has been read is user
*/
boolean isUser() {
  for(int i = 0; i < 4; i++){
    if(mfrc522.uid.uidByte[i] != userUID[i]){
      return false;
    }
  }

  int trailerBlock = 7;
  MFRC522::StatusCode status;
  status = (MFRC522::StatusCode) mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A,
                                                          trailerBlock,
                                                          &userKey,
                                                          &(mfrc522.uid));
  if(status != MFRC522::STATUS_OK){
    mfrc522.PICC_HaltA();         //Halt PICC
    mfrc522.PCD_StopCrypto1();    //Stop encryption on PCD
    return false;
  }

  mfrc522.PICC_HaltA();         //Halt PICC
  mfrc522.PCD_StopCrypto1();    //Stop encryption on PCD
  return true;
}


/*
/// Check if tag has been read is staff
*/
boolean isStaff() {
  int trailerBlock = 7;
  MFRC522::StatusCode status;
  
  //Serial.println(F("Authenticating staff using key A..."));
  status = (MFRC522::StatusCode) mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A,
                                                          trailerBlock,
                                                          &staffKey,
                                                          &(mfrc522.uid));
  if (status != MFRC522::STATUS_OK) {
    mfrc522.PICC_HaltA();         //Halt PICC
    mfrc522.PCD_StopCrypto1();    //Stop encryption on PCD
    return false;
  }
  
  mfrc522.PICC_HaltA();         //Halt PICC
  mfrc522.PCD_StopCrypto1();    //Stop encryption on PCD
  return true;
}


/*
///
*/
boolean encodeUserTag() {

  MFRC522::StatusCode status;
  byte trailerBlock = 7;
  byte blockAddr = 7;
  byte accessBit[4] = {0xFF, 0x07, 0x80, 0x69};
  
  byte dataBlock[16] = {};        // This array use to write data onto sector trailer 
  
  for(int i = 0; i < 16; i++){
    if(i >= 0 && i < 6){          // 6 bytes from 0 to 5 for "Key A"
      dataBlock[i] = userKey.keyByte[i];
    }else if(i >= 6 && i < 10){    // 4 bytes from 6 to 9 for "Access Bits"
      dataBlock[i] = accessBit[i - 6];
    }else{                        // 6 bytes from 10 to 15 for "Key B"
      dataBlock[i] = defaultKey.keyByte[i - 10];
    }
  }

  // Authenticate using key B
  Serial.println(F("Authenticating again using key B..."));
  status = (MFRC522::StatusCode) mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_B,
                                                          trailerBlock,
                                                          &defaultKey,
                                                          &(mfrc522.uid));
  if (status != MFRC522::STATUS_OK) {
    mfrc522.PICC_HaltA();         //Halt PICC
    mfrc522.PCD_StopCrypto1();    //Stop encryption on PCD
    return false;
  }

  // Write data to the block
  Serial.print(F("Writing data into block ")); Serial.print(blockAddr);
  Serial.println(F(" ..."));
  printHexArray(dataBlock, 16); Serial.println();
  status = (MFRC522::StatusCode) mfrc522.MIFARE_Write(blockAddr, dataBlock, 16);
  
  Serial.println();

  Serial.println(F("Authenticating using key A..."));
  status = (MFRC522::StatusCode) mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A,
                                                          trailerBlock,
                                                          &userKey,
                                                          &(mfrc522.uid));
  if (status != MFRC522::STATUS_OK) {
    mfrc522.PICC_HaltA();         //Halt PICC
    mfrc522.PCD_StopCrypto1();    //Stop encryption on PCD
    return false;
  }
  
  mfrc522.PICC_HaltA();         //Halt PICC
  mfrc522.PCD_StopCrypto1();    //Stop encryption on PCD
  return true;
}


/*
///
*/
void printHexArray(byte *buffer, byte bufferSize) {
    for (byte i = 0; i < bufferSize; i++){
        Serial.print(buffer[i] < 0x10 ? " 0" : " ");
        Serial.print(buffer[i], HEX);
    }
}


/*
/// 
*/
void closedEffect(){
  // NeoPixels effect for closed procedure
  int halfOfStrip = pixels.numPixels() / 2;
  if((pixels.numPixels() % 2) == 1)
  {
    for(int j = 0; j <= halfOfStrip; j++)
    {
      pixels.setPixelColor(j, pixels.Color(250,0,0));
      pixels.setPixelColor((halfOfStrip * 2) - j, pixels.Color(250,0,0));
      pixels.show();
      delay(150);
      pixels.setPixelColor(j, pixels.Color(0,0,0));
      pixels.setPixelColor((halfOfStrip * 2) - j, pixels.Color(0,0,0));
      pixels.show();
    }
  }
}


/*
///
*/
void openEffect(){
  //destroyBacteria = false;
  // NeoPixels effect for closed procedure
  int halfOfStrip = pixels.numPixels() / 2;
  if((pixels.numPixels() % 2) == 1)
  {
    for(int j = 0; j <= halfOfStrip; j++)
    {
      pixels.setPixelColor(halfOfStrip - j, pixels.Color(250,0,0));
      pixels.setPixelColor(halfOfStrip + j, pixels.Color(250,0,0));
      pixels.show();
      delay(150);
      pixels.setPixelColor(halfOfStrip - j, pixels.Color(0,0,0));
      pixels.setPixelColor(halfOfStrip + j, pixels.Color(0,0,0));
      pixels.show();

    }
  }
}


/*
///
*/
void networkStatusEffect(wl_status_t status)
{
  if(status == WL_CONNECTED)
  {
    for(uint16_t i = 0; i < 255; i++){
      pixels.setPixelColor(4, pixels.Color(0,0, i));
      pixels.show();
      delay(10);    
    }
    for(uint16_t i = 0; i <= 255; i++){
      pixels.setPixelColor(4, pixels.Color(0, 0, 255 - i));
      pixels.show();
      delay(5);    
    }
  }else
  {
    for(uint16_t i = 0; i < 255; i++){
      pixels.setPixelColor(4, pixels.Color(i, i, 0));
      pixels.show();
      delay(10);    
    }
    for(uint16_t i = 0; i <= 255; i++){
      pixels.setPixelColor(4, pixels.Color(255 - i, 255 - i, 0));
      pixels.show();
      delay(5);    
    }
  }
}

/*
///
*/
void isReadyEffect(uint8_t wait){
  uint8_t x;
  int numPixels = pixels.numPixels();
  for(x = 0; x < numPixels; x++){
    pixels.setPixelColor(x, pixels.Color(0, 0, 255));
    pixels.show();
    delay(wait);
    pixels.setPixelColor(x, pixels.Color(0, 0, 0));
    pixels.show();
  }
  for(x = 1; x <= numPixels; x++){
    pixels.setPixelColor(numPixels - x, pixels.Color(0, 0, 255));
    pixels.show();
    delay(wait);
    pixels.setPixelColor(numPixels - x, pixels.Color(0, 0, 0));
    pixels.show();
  }
}

/*
/// Create sound of the Buzzer
*/
void turnOnBuzzerSignal(int numberOfPlay, int delayTime){
  for(int i = 0; i < numberOfPlay; i++){
    digitalWrite(BUZZER_PIN, HIGH);
    delay(delayTime);
    digitalWrite(BUZZER_PIN, LOW);
    delay(delayTime);
  }
}


/*
///
*/
int getDistanceOutside()
{
  unsigned long duration;
  int distance;
  
  digitalWrite(OUTSIDE_US015_TRIG_PIN,0);   
  delayMicroseconds(2);
  digitalWrite(OUTSIDE_US015_TRIG_PIN,1);   
  delayMicroseconds(5);   
  digitalWrite(OUTSIDE_US015_TRIG_PIN,0); 

  duration = pulseIn(OUTSIDE_US015_ECHO_PIN, HIGH);
  distance = int(duration/2/29.412);

  return distance;  //Distance unit: (cm)
}


/*
///
*/
int getDistanceInside()
{
  unsigned long duration;
  int distance;
  
  digitalWrite(INSIDE_US015_TRIG_PIN,0);   
  delayMicroseconds(2);
  digitalWrite(INSIDE_US015_TRIG_PIN,1);   
  delayMicroseconds(5);   
  digitalWrite(INSIDE_US015_TRIG_PIN,0); 

  duration = pulseIn(INSIDE_US015_ECHO_PIN, HIGH);
  distance = int(duration/2/29.412);

  return distance;  //Distance unit: (cm)
}



/*
///
*/
void postDataToServer(String token, double latitude, double longitude, float weight, int garbageCapacity)
{
  String _location = String(latitude, 6) + "," + String(longitude, 6);
  String _weight = String(weight);
  String _capacity = String(garbageCapacity);

  // Prepare your HTTP POST request data
  String httpRequestData=  ""; 
  httpRequestData += "token="       + token;
  httpRequestData += "&location="   + _location;
  httpRequestData += "&weight="     + _weight;
  httpRequestData += "&garbagepercent=" + _capacity  + "";

  postRequest(httpRequestData);
}


/*
/// 
*/
void postDataToServer(String token, double latitude, double longitude, float weight, int garbageCapacity, byte *staffUID){
  
  String _location = String(latitude, 6) + "," + String(longitude, 6);
  String _weight = String(weight);
  String _capacity = String(garbageCapacity);
  String _uid;

  // Convert byte array {0xXX, 0xXX, 0xXX, 0xXX} to string "XX:XX:XX:XX"
  int leng = sizeof(staffUID) / sizeof(byte);
  for(int i = 0; i < leng; i++){
    _uid += staffUID[i] < 0x10 ? "0" : "";
    _uid += String(staffUID[i], HEX);
    _uid += i < (leng - 1) ? ":" : "";
  }
  
  // Prepare your HTTP POST request data
  String httpRequestData=  ""; 
  httpRequestData += "token="       + token;
  httpRequestData += "&location="   + _location;
  httpRequestData += "&weight="     + _weight;
  httpRequestData += "&garbagepercent=" + _capacity;
  httpRequestData += "&uid="        + _uid + "";

  postRequest(httpRequestData);
}


/*
///
*/
void postRequest(String httpRequestData){
  WiFiClient client;
  HTTPClient http;

  http.begin(client, serverName);
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  
  Serial.print("httpRequestData: ");
  Serial.println(httpRequestData);
  
  // Send HTTP POST request

   int httpResponseCode = http.POST(httpRequestData);
 
   String payload = http.getString(); 
      
  if (httpResponseCode > 0) {
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
    Serial.println(payload);  
  }
  else {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
  }
  
  http.end();
}


/*
///
*/
void closedProcedure(){
  isOpening = false;
  digitalWrite(IN1_MOTOR_PIN, HIGH);
  digitalWrite(IN2_MOTOR_PIN, LOW);

  turnOnBuzzerSignal(3, 150);
  closedEffect();
}


/*
///
*/
void openProcedure(){
  Serial.print(">> Open Procedure\n");
  timeOutClosed = millis();
  isOpening = true;

  if(digitalRead(UV_LIGHT_PIN) == HIGH){
    digitalWrite(UV_LIGHT_PIN, LOW);
    delay(1000);
  }
  digitalWrite(IN1_MOTOR_PIN, LOW);
  digitalWrite(IN2_MOTOR_PIN, HIGH);

  turnOnBuzzerSignal(2, 150);
  openEffect();
}