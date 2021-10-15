
#define TFT_BLACK       0x0000      /*   0,   0,   0 */
#define TFT_NAVY        0x000F      /*   0,   0, 128 */
#define TFT_DARKGREEN   0x03E0      /*   0, 128,   0 */
#define TFT_DARKCYAN    0x03EF      /*   0, 128, 128 */
#define TFT_MAROON      0x7800      /* 128,   0,   0 */
#define TFT_PURPLE      0x780F      /* 128,   0, 128 */
#define TFT_OLIVE       0x7BE0      /* 128, 128,   0 */
#define TFT_LIGHTGREY   0xD69A      /* 211, 211, 211 */
#define TFT_DARKGREY    0x7BEF      /* 128, 128, 128 */
#define TFT_BLUE        0x001F      /*   0,   0, 255 */
#define TFT_GREEN       0x07E0      /*   0, 255,   0 */
#define TFT_CYAN        0x07FF      /*   0, 255, 255 */
#define TFT_RED         0xF800      /* 255,   0,   0 */
#define TFT_MAGENTA     0xF81F      /* 255,   0, 255 */
#define TFT_YELLOW      0xFFE0      /* 255, 255,   0 */
#define TFT_WHITE       0xFFFF      /* 255, 255, 255 */
#define TFT_ORANGE      0xFDA0      /* 255, 180,   0 */
#define TFT_GREENYELLOW 0xB7E0      /* 180, 255,   0 */
#define TFT_PINK        0xFE19      /* 255, 192, 203 */
#define TFT_BROWN       0x9A60      /* 150,  75,   0 */
#define TFT_GOLD        0xFEA0      /* 255, 215,   0 */
#define TFT_SILVER      0xC618      /* 192, 192, 192 */
#define TFT_SKYBLUE     0x867D      /* 135, 206, 235 */
#define TFT_VIOLET      0x915C      /* 180,  46, 226 */

//#include "BluetoothSerial.h"
#include <HardwareSerial.h>
#include "HardwareSerial_NB_BC95.h"
#include <SDI12.h>
#include "FS.h"
#include "SD.h"
#include <TaskScheduler.h>
#include <TimeLib.h>
#include <ArduinoJson.h>
#include "SPI.h"
#include <TFT_eSPI.h>
#include "Sensor.h"
#include "Free_Fonts.h"
#include <EEPROM.h>
#include <ModbusMaster.h>
#include <ArduinoOTA.h>
#include <WiFi.h>
#include <Wire.h>
#include <WiFiManager.h>


#include "rainImg.h"
#include "levelImg.h"
#include "batteryImg.h"
#include "wifi4.h"
#include "wifi3.h"
#include "wifi2.h"
#include "wifi1.h"

#define WIFI_AP ""
#define WIFI_PASSWORD ""
WiFiManager wifiManager;

#define _TASK_TIMECRITICAL

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif




#define CS  4



TFT_eSPI tft = TFT_eSPI();
TFT_eSprite spriteTime = TFT_eSprite(&tft);
TFT_eSprite spriteSave = TFT_eSprite(&tft);
TFT_eSprite spriteUpdate = TFT_eSprite(&tft);
TFT_eSprite spriteRain = TFT_eSprite(&tft);
TFT_eSprite spriteLevel = TFT_eSprite(&tft);


#define SERIAL_BAUD 115200 /*!< The baud rate for the output serial port */
#define DATA_PIN 13         /*!< The pin of the SDI-12 data bus */
#define POWER_PIN -1       /*!< The sensor power pin (or -1 if not switching power) */
#define SENSOR_ADDRESS 0
#define DRY_CONTACT_PIN  25

String deviceToken = "wupnG4krB1CdhCIUk6W2";
String serverIP = "147.50.151.130"; // Your Server IP;
unsigned long nowTime;

String serverPort = "19956"; // Your Server Port;
String json = "";
String udpData = "";
boolean stateGetWaterLevel = 0;
boolean readModbusStatus = false;
int prevStage = 0;
int rainGate = 0;

ModbusMaster node;
String waterLevel = "";
String voltLevel = "";

int reading;                // ค่าที่อ่านได้จากปุ่มกด (รวม bounce)
int counter = 0;            // จำนวน iteration ที่เราเห็นการกด หรือปล่อย
int current_state = LOW;    // ค่าที่ได้หลังการทำ debounce (คือการกดหรือปล่อยจริงๆ)
long timeLoop = 0;              // เวลาล่าสุดที่มีการเก็บค่า
int debounce_count = 2;    // จำนวนมิลลิวินาที/รอบการวนลูป ที่ต่ำสุดที่เชื่อได้ว่ามีการกด หรือปล่อยจริงๆ
int rainCount = 0;       // ไว้แสดงจำนวนการกด

int period = 2;
unsigned long time_now = 0;

HardwareSerial modbus(2);
HardwareSerial_NB_BC95 AISnb;
signal meta ;

//BluetoothSerial SerialBT;
/** Define the SDI-12 bus */
SDI12 mySDI12(DATA_PIN);
WebServer server(80);

String sdiResponse = "";
String myCommand   = "";
String model = "";
String result = "";
String _minusSign = "-";
unsigned long epoch;
unsigned long epoch_mill;
bool sendFinish = false;

StaticJsonDocument<300> doc;

const long interval = 300000;  //millisecond
unsigned long previousMillis = 0;

unsigned long lastSend;

const long intervalDrycontact = 1000;  //millisecond
unsigned long previousMillisDrycontact = 0;
void t1CallModbus();
void t2CallgetVoltLevel();
void t3CallgetRain();
void t4CallsendViaNBIOT();
void t5CallTime();

//TASK
Task t1(60000, TASK_FOREVER, &t1CallModbus);
Task t2(60000, TASK_FOREVER, &t2CallgetVoltLevel);
Task t3(100, TASK_FOREVER, &t3CallgetRain);
Task t4(60000, TASK_FOREVER, &t4CallsendViaNBIOT);
Task t5(1000, TASK_FOREVER, &t5CallTime);
Scheduler runner;
String _config = F("{\"_type\":\"retrattr\",\"Tn\":\"8966031840041733110\",\"keys\":[\"epoch\",\"ip\"]}");
unsigned long _epoch = 0;
String _IP = "";
String dataJson = "";
boolean validEpoc = false;




struct Meter
{
  String waterLevel;
  String flow;
  String velocity;
  String area;
  String volt;

};
Meter meter;

unsigned long ms;




/**********************************************  WIFI Client 注意编译时要设置此值 *********************************
   wifi client
*/
//WiFi&OTA 参数
String HOSTNAME = "Decode-";
#define PASSWORD "7650" //the password for OTA upgrade, can set it in any char you want

void configModeCallback (WiFiManager *myWiFiManager) {
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  //if you used auto generated SSID, print it
  Serial.println(myWiFiManager->getConfigPortalSSID());
}
void drawSpriteRain()
{

  spriteRain.fillSprite(TFT_BLACK);
  spriteRain.setTextSize(1);
  spriteRain.setFreeFont(FMB9);
  spriteRain.setTextColor(TFT_GREEN);
  spriteRain.drawNumber(rainCount, 0, 0);
  spriteRain.pushSprite(105, 100);
}

void drawSpriteLevel()
{

  spriteLevel.fillSprite(TFT_BLACK);
  spriteLevel.setTextSize(1);
  spriteLevel.setFreeFont(FMB9);
  spriteLevel.setTextColor(TFT_GREEN);
  spriteLevel.drawString(meter.waterLevel, 0, 0);
  spriteLevel.pushSprite(115, 140);
}

void drawSpriteTime()
{
  //  lastSend = epoch + ((millis() - epoch_mill) / 1000) + (7 * 3600);

  spriteTime.fillSprite(TFT_BLACK);
  spriteTime.setTextSize(1);
  spriteTime.setFreeFont(FMB9);
  spriteTime.setTextColor(TFT_MAGENTA);
  spriteTime.drawString(mkTime(nowTime), 0, 5);
  spriteTime.pushSprite(0, 20);
}
void drawSpriteSave()
{


  spriteSave.setTextSize(1);
  spriteSave.setFreeFont(FMB9);

  tft.fillRect(0, 215, 320, 25, TFT_PURPLE);
  spriteSave.drawString("SAVE :" + mkTime(lastSend), 10, 0);
  spriteSave.pushSprite(0, 220);


}

void drawUpdate(int num, int x, int y)
{
  spriteUpdate.createSprite(40, 20);
  spriteUpdate.fillScreen(TFT_BLACK);
  spriteUpdate.setFreeFont(FSB9);
  spriteUpdate.setTextColor(TFT_YELLOW);
  spriteUpdate.setTextSize(1);
  spriteUpdate.drawNumber(num, 0, 4);
  spriteUpdate.drawString("%", 20, 4);
  spriteUpdate.pushSprite(x, y);
  spriteUpdate.deleteSprite();
}

void setup() {

  Serial.begin(SERIAL_BAUD);
  modbus.begin(9600, SERIAL_8N1, 16, 17);
  initTFT();
  //  SerialBT.begin("Decode"); //Bluetooth device name
  //  SerialBT.println("Hello:Decode");

  tft.drawString("Waiting for WiFi", 10, (tft.height() / 2) - 20, GFXFF);
  tft.drawString(" (Timeout 120 Sec)", 20, (tft.height() / 2) + 20, GFXFF);

  //  Serial.println("Opening SDI-12 bus...");
  //  mySDI12.begin();
  //  delay(1000);  // allow things to settle
  //  // Power the sensors;
  //  if (POWER_PIN > 0) {
  //    Serial.println("Powering up sensors...");
  //    pinMode(POWER_PIN, OUTPUT);
  //    digitalWrite(POWER_PIN, HIGH);
  //    delay(200);
  //  }
  //  Serial.print("devie:");
  //  Serial.println(SENSOR_ADDRESS);
  //  SerialBT.print("devie:");
  //  SerialBT.println(SENSOR_ADDRESS);
  //  getModel();
  wifiManager.setTimeout(120);
  wifiManager.setAPCallback(configModeCallback);
  String wifiName = "@Decode-";
  wifiName.concat(String((uint32_t)ESP.getEfuseMac(), HEX));
  if (!wifiManager.autoConnect(wifiName.c_str())) {
    //Serial.println("failed to connect and hit timeout");
    //reset and try again, or maybe put it to deep sleep
    //    ESP.reset();
    //delay(1000);
    delay(1);
  }

  setupOTA();
  Serial.print("Heap:");
  Serial.println(ESP.getFreeHeap());

  delay(1000);

  pingRESP pingR = AISnb.pingIP(serverIP);

  Serial.println(model);
  pinMode(DRY_CONTACT_PIN, INPUT_PULLUP);
  attachInterrupt(DRY_CONTACT_PIN, checkRainGate, CHANGE);
  String nccid = AISnb.getNCCID();
  runner.init();
  Serial.println("Initialized scheduler");

  runner.addTask(t1);
  Serial.println("added t1");
  runner.addTask(t2);
  Serial.println("added t2");
  runner.addTask(t3);
  Serial.println("added t4");
  runner.addTask(t4);
  Serial.println("added t4");
  runner.addTask(t5);
  Serial.println("added t5");
  delay(2000);


  Serial.print("nccid:");
  Serial.println(nccid);

  Serial.println("Initialized scheduler");
  //  SerialBT.println("Start..");

  HOSTNAME.concat(getMacAddress());
  //  SerialBT.begin(HOSTNAME); //Bluetooth

  //  SerialBT.begin(HOSTNAME); //Bluetooth device name
  //  SerialBT.println(HOSTNAME);
  AISnb.debug = true;
  AISnb.setupDevice(serverPort);

  _init();
  initSD();

  /* Put your SSID & Password */
  const char* ssid = "SDI12";  // Enter SSID here
  const char* password = "12345678";  //Enter Password here
  
  
  if (WiFi.status() != WL_CONNECTED) {
    /* Put IP Address details */
    IPAddress local_ip(192,168,1,1);
    IPAddress gateway(192,168,1,1);
    IPAddress subnet(255,255,255,0);
    
    WiFi.softAP(ssid, password);
    WiFi.softAPConfig(local_ip, gateway, subnet);
  }
  server.on("/", handle_file);
  server.on("/get", handle_get);
  server.onNotFound(handle_NotFound);
  server.begin();
  Serial.println("HTTP server started");
  Serial.println(WiFi.localIP());

  getepoch();
  tft.fillScreen(TFT_BLACK);
  lastSend = epoch + ((millis() - epoch_mill) / 1000) + (7 * 3600);
  TFTshow(epoch + ((millis() - epoch_mill) / 1000) + (7 * 3600));





  _loadConfig();

  Serial.println();
  Serial.println(F("***********************************"));

  Serial.println("Initialize...");


  tft.setFreeFont(FMB9);
  tft.drawString("ID:" + deviceToken, 0, 60);
  tft.pushImage(10, 90, RainWidth, RainHeight, rain);
  //  tft.fillRect(130, 100, 200, 24, TFT_BLACK);
  nowTime = epoch + ((millis() - epoch_mill) / 1000) + (7 * 3600);
  t1.enable();
  t2.enable();
  t3.enable();
  t4.enable();
  t5.enable();
  meter.waterLevel = "0";
  meter.flow = "0";
  meter.velocity = "0";
  meter.area = "0";
  meter.volt = "0";

  TFTshow(nowTime);
  drawSpriteTime();
  drawSpriteSave();


}

void handle_file() {
  Serial.println("Hello");
  File dir = SD.open("/");
  String htmlStr = "<!DOCTYPE html> <html>\n";
  htmlStr += "<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n";
  htmlStr += "<title>File System</title>\n";
  htmlStr += "</head>\n";
  htmlStr += "<body>\n";
  htmlStr += "<h1>File System</h1>\n";
  while (true) {
    File entry = dir.openNextFile();
    if (! entry) {
      break;
    }
    if (entry.isDirectory()) {
    } else {
      htmlStr += "<a href=\"/get?file=";
      //htmlStr += "<a href=\"/file";
      htmlStr += entry.name();
      htmlStr += "\">";
      htmlStr += entry.name();
      htmlStr += "</a><br/>\n";
    }
    Serial.println(entry.name());
    entry.close();
  }
  htmlStr +="</body>\n";
  htmlStr +="</html>\n";
  server.send(200, "text/html", htmlStr);
}
void handle_get() {
  String fileBytes = "";
  String fileType = "";
  File dataFile = SD.open(server.arg("file"));
  if (dataFile) {
    while (dataFile.available()) {
      fileBytes += (char)dataFile.read();
    }
    dataFile.close();
  }
  if (server.arg("file").endsWith(".csv")) fileType = "text/csv";
  else if (server.arg("file").endsWith(".jpeg") || server.arg("file").endsWith(".jpg")) fileType = "image/jpeg";
  else fileType = "application/octet-stream";
  server.sendHeader("Content-Disposition", "attachment");
  server.arg("file").remove(0);
  server.sendHeader("filename", server.arg("file"));
  server.send(200, fileType, fileBytes);
}
void handle_NotFound(){
  server.send(404, "text/plain", "Not found");
}

void loop() {
  runner.execute();
  ArduinoOTA.handle();
  server.handleClient();



  ms = millis();
  if (ms % 60000 == 0)
  {

    Serial.println("Attach WiFi for，OTA "); Serial.println(WiFi.RSSI() );

    setupWIFI();
    setupOTA();

  }

}



void t2CallgetVoltLevel() {
  checkVoltLevel();
}

void t3CallgetRain() {
  //  checkRainGate();
  drawSpriteRain();
}

void getModel() {
  Serial.println("start query..");
  myCommand = String(SENSOR_ADDRESS) + "I!";

  Serial.print("cmd:");
  Serial.println(myCommand);  // echo command to terminal

  mySDI12.sendCommand(myCommand);
  delay(1000);  // wait a while for a response

  while (mySDI12.available()) {  // build response string
    char c = mySDI12.read();
    if ((c != '\n') && (c != '\r')) {
      sdiResponse += c;
      delay(5);
    }
  }

  if (sdiResponse.length() > 1) {
    Serial.println(sdiResponse);  // write the response to the screen
    model = sdiResponse;
  }
  mySDI12.clearBuffer();

  delay(200);       // delay between taking reading and requesting data

  // next command to request data from last measurement
  myCommand = String(SENSOR_ADDRESS) + "D0!";
  Serial.println(myCommand);  // echo command to terminal

  mySDI12.sendCommand(myCommand);
  delay(300);  // wait a while for a response

  while (mySDI12.available()) {  // build string from response
    char c = mySDI12.read();
    if ((c != '\n') && (c != '\r')) {
      sdiResponse += c;
      delay(5);
    }
  }
  if (sdiResponse.length() > 1) {
    Serial.println(sdiResponse);  // write the response to the screen
    model = sdiResponse;
  }
  mySDI12.clearBuffer();
}

float voltMeasure(int Pin)
{
  unsigned int vRAW = 0;
  float Vout = 0.0;
  float Vin = 0.0;
  float R1 = 15000.0;
  float R2 = 2000.0;

  vRAW = analogRead(Pin);
  Vout = (vRAW * 3.3) / 4096;
  Vin = Vout / (R2 / (R1 + R2));
  if (Vin < 0.05)
  {
    Vin = 0.0;
  }
  return Vin + 1;
}

boolean getResponse() {
}

void getWaterLevel() {
  sdiResponse = "";  // clear the response string

  Serial.println("start query..");
  myCommand = String(SENSOR_ADDRESS) + "M!";
  Serial.print("cmd:");
  Serial.println(myCommand);  // echo command to terminal

  mySDI12.sendCommand(myCommand);
  delay(12000);  // wait a while for a response

  while (mySDI12.available()) {  // build response string
    char c = mySDI12.read();
    if ((c != '\n') && (c != '\r')) {
      sdiResponse += c;
      delay(5);
    }
  }
  Serial.print("  sdiResponse:");
  Serial.println(sdiResponse);  // write the response to the screen

  if (! sdiResponse.indexOf("121") > 0 ) {

    int whereis_ = sdiResponse.indexOf("+");
    Serial.println(whereis_);

    if (whereis_ > 0) {
      Serial.print("  +:");
      waterLevel = sdiResponse.substring(whereis_, sdiResponse.length());
      waterLevel = string2float(waterLevel);
      Serial.println(waterLevel);
    } else {
      whereis_ = sdiResponse.indexOf("-");
      if (whereis_ > 0 ) { // check for - value
        Serial.print("  -:");
        waterLevel = sdiResponse.substring(whereis_, sdiResponse.length());
        waterLevel = string2float(waterLevel);
        Serial.println(waterLevel);
      }
    }
    mySDI12.clearBuffer();
  } else {
    delay(200);       // delay between taking reading and requesting data
    sdiResponse = "";  // clear the response string


    // next command to request data from last measurement
    myCommand = String(SENSOR_ADDRESS) + "D0!";
    Serial.print("cmd:");
    Serial.println(myCommand);  // echo command to terminal

    mySDI12.sendCommand(myCommand);
    delay(1200);  // wait a while for a response
    //    for (int i = 0; i < 1200000; i++);
    while (mySDI12.available()) {  // build string from response
      char c = mySDI12.read();
      if ((c != '\n') && (c != '\r')) {
        sdiResponse += c;
        delay(5);
      }
    }
    Serial.print("  sdiResponse:");
    Serial.println(sdiResponse);  // write the response to the screen
    if (sdiResponse.length() > 1) {
      int whereis_ = sdiResponse.indexOf("+");
      Serial.println(whereis_);
      if (whereis_ > 0) {
        Serial.print("  +:");
        waterLevel = sdiResponse.substring(whereis_, sdiResponse.length());
        waterLevel = string2float(waterLevel);
        Serial.println(waterLevel);
      } else {
        whereis_ = sdiResponse.indexOf("-");
        if (whereis_ > 0 ) { // check for - value
          Serial.print("  -:");
          waterLevel = sdiResponse.substring(whereis_, sdiResponse.length());
          waterLevel =  string2float(waterLevel);
          Serial.println(waterLevel);
        }
      }
    }
    mySDI12.clearBuffer();
  }
}

void checkVoltLevel()
{
  float voltlev;
  voltlev = voltMeasure(35);
  Serial.print("Battery");
  Serial.print(":");
  Serial.println(voltlev);
  voltLevel = String(voltlev, 2);
}

void checkRainGate()
{
  int current1_state, current2_state;
  //อ่านค่าปุ่มกด
  current1_state = digitalRead(DRY_CONTACT_PIN);

  time_now = millis();
  while (millis() < time_now + period) {
    //wait approx. [period] ms
  }
  current2_state = digitalRead(DRY_CONTACT_PIN);

  if (current1_state == current2_state)
  {
    //ถ้าเป็นการกดจะแสดงผลจำนวนที่กดไปทาง serial monitor
    if (current1_state == LOW) {
      rainCount++;
      //      Serial.print("LOW");
      Serial.print(rainCount);
      Serial.print(":");
      Serial.println(current_state);
      //      SerialBT.print(rainCount);
      //      SerialBT.print(":");/
      //      SerialBT.println(current_state);
    }
    counter = 0;
    current_state = digitalRead(DRY_CONTACT_PIN);
    //    Serial.print("HIGH ");
    Serial.print(rainCount);
    Serial.print(":");
    Serial.println(current_state);
    //    SerialBT.print(rainCount);
    //    SerialBT.print(":");
    //    SerialBT.println(current_state);
    drawSpriteRain();
  }
}

String string2float(String v) {
  String result = "";
  //+0145.715
  //+0105.715
  Serial.print("    :");
  Serial.println(v);

  if (v.indexOf("-") == 0)
    result = _minusSign;

  if (!(  v.startsWith("+0000", 0) == 0  && v.startsWith("-0000", 0)  == 0) ) {
    Serial.println("  5");
    result += "0" + v.substring(5, v.length());
    return result;
  } else if (! ( v.startsWith("+000", 0) == 0  && v.startsWith("-000", 0) == 0 ) ) {
    Serial.println("  4");
    result +=   v.substring(4, v.length());
    return result;
  } else if  (!( v.startsWith("+00", 0) == 0  && v.startsWith("-00", 0) == 0 ) ) {
    Serial.println("  3");
    result += v.substring(3, v.length());
    Serial.print("result:");
    Serial.println(result);
    return result;
  } else if  (! ( v.startsWith("+0", 0) == 0  && v.startsWith("-0", 0) == 0 ) ) {
    Serial.println("  2");
    result += v.substring(2, v.length());
    return result;
  } else if  (! ( v.startsWith("+", 0) == 0 && v.startsWith("-", 0) == 0 ) ) {
    Serial.println("  1");
    result += v.substring(1, v.length());
    return result;
  } else {
    Serial.println("  0");
    result += v.substring(0, v.length());
    return result;
  }
}

char  char_to_byte(char c)
{
  if ((c >= '0') && (c <= '9'))
  {
    return (c - 0x30);
  }
  if ((c >= 'A') && (c <= 'F'))
  {
    return (c - 55);
  }
}

void getepoch() {
  String retJson = "{\"_type\":\"retrattr\",\"Tn\":\"";
  retJson.concat(deviceToken);
  retJson.concat("\",\"keys\":[\"epoch\",\"ip\"]}");
  do {
    json = "";
    UDPSend udp = AISnb.sendUDPmsgStr(serverIP, serverPort, retJson);
    UDPReceive resp = AISnb.waitResponse();
    epoch_mill = millis();
    Serial.println("Start Send");
    AISnb.receive_UDP(resp);
    Serial.print("waitData:");
    Serial.println(resp.data);
    if (udp.status == true) {
      for (int x = 0; x < resp.data.length(); x += 2) {
        char c = char_to_byte(resp.data[x]) << 4 | char_to_byte(resp.data[x + 1]);

        json += c;
      }
      //Serial.println(json);
      DeserializationError error = deserializeJson(doc, json);

      // Test if parsing succeeds.
      if (error) {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.f_str());
        delay(4000);
      } else {
        epoch = doc["epoch"];
        Serial.print("epoch : ");
        Serial.println(epoch);
        break;
      }
    }
  } while (true);
}

void readFile(fs::FS &fs, const char * path) {
  Serial.printf("Reading file: %s\n", path);

  File file = fs.open(path);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }

  Serial.print("Read from file: ");
  while (file.available()) {
    Serial.write(file.read());
  }
  file.close();
}

void writeFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (file.print(message)) {
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
  Serial.println("appendFile.done");
}

void deleteFile(fs::FS &fs, const char * path) {
  Serial.printf("Deleting file: %s\n", path);
  if (fs.remove(path)) {
    Serial.println("File deleted");
  } else {
    Serial.println("Delete failed");
  }
}

void testFileIO(fs::FS &fs, const char * path) {
  File file = fs.open(path);
  static uint8_t buf[512];
  size_t len = 0;
  uint32_t start = millis();
  uint32_t end = start;
  if (file) {
    len = file.size();
    size_t flen = len;
    start = millis();
    while (len) {
      size_t toRead = len;
      if (toRead > 512) {
        toRead = 512;
      }
      file.read(buf, toRead);
      len -= toRead;
    }
    end = millis() - start;
    Serial.printf("%u bytes read for %u ms\n", flen, end);
    file.close();
  } else {
    Serial.println("Failed to open file for reading");
  }
  file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  size_t i;
  start = millis();
  for (i = 0; i < 2048; i++) {
    file.write(buf, 512);
  }
  end = millis() - start;
  Serial.printf("%u bytes written for %u ms\n", 2048 * 512, end);
  file.close();
}

//void t2CallShowEnv() {
//
//}

void listDir(fs::FS &fs, const char * dirname, uint8_t levels) {
  Serial.printf("Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if (!root) {
    Serial.println("Failed to open directory");
    return;
  }
  if (!root.isDirectory()) {
    Serial.println("Not a directory");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print(" DIR : ");
      Serial.println(file.name());
      if (levels) {
        listDir(fs, file.name(), levels - 1);
      }
    } else {
      Serial.print(" FILE: ");
      Serial.print(file.name());
      Serial.print(" SIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}

void initSD() {

  if (!SD.begin(CS)) {
    Serial.println("-----------------------Card Mount Failed");
    return;
  }
  uint8_t cardType = SD.cardType();

  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return;
  }

  Serial.print("SD Card Type: ");
  if (cardType == CARD_MMC) {
    Serial.println("MMC");
  } else if (cardType == CARD_SD) {
    Serial.println("SDSC");
  } else if (cardType == CARD_SDHC) {
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("-----------------------------SD Card Size: %lluMB\n", cardSize);
  //  writeFile(SD, "/hello.txt", "Hello ");
  //  appendFile(SD, "/hello.txt", "World!\n");
  //  readFile(SD, "/hello.txt");
  //  testFileIO(SD, "/test.txt");
  Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
  Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));
  listDir(SD, "/", 0);
  if (!SD.exists("/data.csv")) {
    writeFile(SD, "/data.csv", "DateTime,waterLevel,rainCount,voltLevel,rssi\n");
  }
}

void initTFT() {
  tft.init();
  tft.setSwapBytes(true);
  tft.setRotation(3);
  tft.setFreeFont(FMB12);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_BLUE);
  tft.drawString("Decode", tft.width() / 2 - 50, tft.height() / 2, GFXFF);
  delay(3000);
  tft.fillScreen(TFT_BLACK);
  spriteTime.createSprite(210, 20);

  spriteSave.createSprite(320, 20);
  spriteRain.createSprite(100, 20);
  spriteLevel.createSprite(100, 20);


}

String a0(int n) {
  return (n < 10) ? "0" + String(n) : String(n);
}
String mkTime(unsigned long epoch_) {
  return String(a0(day(epoch_))) + "/" + a0(month(epoch_)) + "/" + a0(year(epoch_)) + " " + a0(hour(epoch_)) + ":" + a0(minute(epoch_)) + ":" + a0(second(epoch_));
}
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  if ((in_max - in_min) + out_min != 0) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  } else {
    return 0;
  }
}
void TFTshow(unsigned long nowTime) {
  tft.setTextSize(1);

  //  tft.fillScreen(TFT_WHITE);

  tft.setTextSize(1);
  tft.setFreeFont(FMB9);
  tft.setTextColor(TFT_WHITE);

  tft.setFreeFont(FMB9);
  tft.setTextSize(1);
  int rssi = map(meta.rssi.toInt(), -110, -40, 0, 100);
  if (rssi > 100) rssi = 100;
  if (rssi < 0) rssi = 0;
  if (rssi > 74) tft.pushImage(285, 10, WifiWidth, WifiHeight, wifi4);
  else if (rssi > 49) tft.pushImage(285, 10, WifiWidth, WifiHeight, wifi3);
  else if (rssi > 24) tft.pushImage(285, 10, WifiWidth, WifiHeight, wifi2);
  else tft.pushImage(280, 10, WifiWidth, WifiHeight, wifi1);

  tft.pushImage(215, 14, BatteryWidth, BatteryHeight, battery);
  int percentVolt =  map(meter.volt.toInt(), 0.0, 12, 0, 100);
  tft.drawString(String(percentVolt) + "%", 245, 25);
  tft.setFreeFont(FMB9);


  tft.drawString("RAIN : ", 40, 100);
  tft.pushImage(10, 134, LevelWidth, LevelHeight, level);
  //  tft.fillRect(140, 140, 190, 24, TFT_BLACK);
  tft.drawString("LEVEL : ", 40, 140);
  //  Serial.println("TFTShow()");
}

void appendSD(unsigned long nowTime) {
  String Data = a0(day(nowTime)) + "/" + a0(month(nowTime)) + "/" + String(year(nowTime)) + " " + a0(hour(nowTime)) + ":" + a0(minute(nowTime)) + ":" + a0(second(nowTime));
  Data.concat(",");
  Data.concat(meter.waterLevel);
  Data.concat(",");
  Data.concat(meter.waterLevel);
  Data.concat(",");
  Data.concat(meter.flow);
  Data.concat(",");
  Data.concat(meter.velocity);
  Data.concat(",");
  Data.concat(meter.area);
  Data.concat(",");
  Data.concat(rainCount);
  Data.concat(",");
  Data.concat(meter.volt);
  Data.concat(",");
  Data.concat(meta.rssi);
  Data.concat("\n");

  char csvData[300];
  Data.toCharArray(csvData, Data.length() + 1);
  Serial.println(Data);
  appendFile(SD, ("/data_" + a0(month(nowTime)) + "_" + String(year(nowTime)) + ".csv").c_str(), csvData);
  lastSend = epoch + ((millis() - epoch_mill) / 1000) + (7 * 3600);

}




void _writeEEPROM(String data) {
  Serial.print("Writing Data:");
  Serial.println(data);

  writeString(10, data);  //Address 10 and String type data
  delay(10);
}

void _loadConfig() {
  serverIP = read_String(10);
  serverIP.trim();
  Serial.print("IP:");
  Serial.println(serverIP);
}


//char  char_to_byte(char c)
//{
//  if ((c >= '0') && (c <= '9'))
//  {
//    return (c - 0x30);
//  }
//  if ((c >= 'A') && (c <= 'F'))
//  {
//    return (c - 55);
//  }
//}
void _init() {

  Serial.println(_config);

  do {
    UDPSend udp = AISnb.sendUDPmsgStr(serverIP, serverPort, _config);
    dataJson = "";
    deviceToken = AISnb.getNCCID();
    Serial.print("nccid:");
    Serial.println(deviceToken);


    UDPReceive resp = AISnb.waitResponse();
    AISnb.receive_UDP(resp);
    Serial.print("waitData:");
    Serial.println(resp.data);


    for (int x = 0; x < resp.data.length(); x += 2)
    {
      char c =  char_to_byte(resp.data[x]) << 4 | char_to_byte(resp.data[x + 1]);

      dataJson += c;
    }
    Serial.println(dataJson);
    DeserializationError error = deserializeJson(doc, dataJson);

    // Test if parsing succeeds.
    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      validEpoc = true;
      delay(4000);
    } else {
      validEpoc = false;
      unsigned long epoch = doc["epoch"];
      _epoch = epoch;
      String ip = doc["ip"];
      _IP = ip;
      Serial.println(dataJson);
      Serial.print("epoch:");  Serial.println(_epoch);
      _writeEEPROM(_IP);
      Serial.println(_IP);

    }
    delay(5000);

  } while (validEpoc);


}


void writeString(char add, String data)
{
  EEPROM.begin(512);
  int _size = data.length();
  int i;
  for (i = 0; i < _size; i++)
  {
    EEPROM.write(add + i, data[i]);
  }
  EEPROM.write(add + _size, '\0'); //Add termination null character for String Data
  EEPROM.commit();
}


String read_String(char add)
{
  int i;
  char data[100]; //Max 100 Bytes
  int len = 0;
  unsigned char k;
  k = EEPROM.read(add);
  while (k != '\0' && len < 500) //Read until null character
  {
    k = EEPROM.read(add + len);
    data[len] = k;
    len++;
  }
  data[len] = '\0';
  Serial.print("Debug:");
  Serial.println(String(data));
  return String(data);
}



void setupOTA()
{
  //Port defaults to 8266
  //ArduinoOTA.setPort(8266);

  //Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(HOSTNAME.c_str());

  //No authentication by default
  ArduinoOTA.setPassword(PASSWORD);

  //Password can be set with it's md5 value as well
  //MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  //ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]()
  {
    Serial.println("Start Updating....");
    //    SerialBT.println("Start Updating....");

    //    SerialBT.printf("Start Updating....Type:%s\n", (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem");

    Serial.printf("Start Updating....Type:%s\n", (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem");
  });

  ArduinoOTA.onEnd([]()
  {

    //    SerialBT.println("Update Complete!");
    Serial.println("Update Complete!");


    ESP.restart();
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
  {
    String pro = String(progress / (total / 100)) + "%";
    int progressbar = (progress / (total / 100));
    //int progressbar = (progress / 5) % 100;
    //int pro = progress / (total / 100);


    //    SerialBT.printf("Progress: %u%%\n", (progress / (total / 100)));

    Serial.printf("Progress: %u%%\n", (progress / (total / 100)));
    drawUpdate(progressbar, 110, 0);

  });

  ArduinoOTA.onError([](ota_error_t error)
  {
    Serial.printf("Error[%u]: ", error);
    String info = "Error Info:";
    switch (error)
    {
      case OTA_AUTH_ERROR:
        info += "Auth Failed";
        Serial.println("Auth Failed");
        break;

      case OTA_BEGIN_ERROR:
        info += "Begin Failed";
        Serial.println("Begin Failed");
        break;

      case OTA_CONNECT_ERROR:
        info += "Connect Failed";
        Serial.println("Connect Failed");
        break;

      case OTA_RECEIVE_ERROR:
        info += "Receive Failed";
        Serial.println("Receive Failed");
        break;

      case OTA_END_ERROR:
        info += "End Failed";
        Serial.println("End Failed");
        break;
    }


    Serial.println(info);
    ESP.restart();
  });

  ArduinoOTA.begin();
}

void setupWIFI()
{
  WiFi.setHostname(HOSTNAME.c_str());


  //等待5000ms，如果没有连接上，就继续往下
  //不然基本功能不可用
  byte count = 0;
  while (WiFi.status() != WL_CONNECTED && count < 10)
  {
    count ++;
    delay(500);
    Serial.print(".");
  }


  if (WiFi.status() == WL_CONNECTED)
    Serial.println("Connecting...OK.");
  else
    Serial.println("Connecting...Failed");

}

String getMacAddress() {
  uint8_t baseMac[6];
  // Get MAC address for WiFi station
  esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
  char baseMacChr[18] = {0};
  sprintf(baseMacChr, "%02X%02X%02X%02X%02X%02X", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
  return String(baseMacChr);
}


//********************************************************************//
//*********************** HeartBeat Function **************************//
//********************************************************************//


void t5CallTime()
{
  nowTime = epoch + ((millis() - epoch_mill) / 1000) + (7 * 3600);
  drawSpriteTime();
  TFTshow(nowTime);
}
void t4CallsendViaNBIOT()
{
  //  readMeter();
  meta = AISnb.getSignal();

  Serial.print("RSSI:"); Serial.println(meta.rssi);

  json = "";
  json.concat("{\"Tn\":\"");
  json.concat(deviceToken);
  /////
  json.concat("\",\"waterLevel\":");
  json.concat(meter.waterLevel);
  json.concat(",\"volt\":");
  json.concat(meter.volt);
  json.concat(",\"flow\":");
  json.concat(meter.flow);
  json.concat(",\"velocity\":");
  json.concat(meter.velocity);
  json.concat(",\"area\":");
  json.concat(meter.area);
  json.concat(",\"rain\":");
  json.concat(rainCount);
  json.concat(",\"rssi\":");
  json.concat(meta.rssi);
  json.concat(",\"csq\":");
  json.concat(meta.csq);
  json.concat("}");
  Serial.println(json);
  Serial.print("MIN:");
  Serial.println(minute(nowTime));
  //  SerialBT.println(json);
  if (readModbusStatus) {
    if ((minute(nowTime)) % 15 == 0 ) {
      UDPSend udp = AISnb.sendUDPmsgStr(serverIP, serverPort, json);
      UDPReceive resp = AISnb.waitResponse();
      Serial.print("rssi:");
      Serial.println(meta.rssi);
      rainCount = 0;
      appendSD(nowTime);
    }
  } else {
    Serial.println("cannot read Modbus");
  }
}

void readMeter()
{
  float modbusValue = 0.0;
  modbusValue = read_Modbus(_waterLevel);
  if (readModbusStatus) {
    meter.waterLevel = modbusValue;
  }

  modbusValue = read_Modbus(_flow);

  if (readModbusStatus) {
    meter.flow = modbusValue;
  }


  modbusValue = read_Modbus(_velocity);
  if (readModbusStatus) {
    meter.velocity = modbusValue;
  }

  modbusValue = read_Modbus(_area);
  if (readModbusStatus) {
    meter.area = modbusValue;
  }

  modbusValue = read_Modbus(_volt);
  if (readModbusStatus) {
    meter.volt = modbusValue;
  }





  Serial.println("");
  if (meter.volt.toInt() > 0) {
    Serial.print("meter.waterLevel:"); Serial.println( meter.waterLevel);
    Serial.print("meter.flow:"); Serial.println( meter.flow);  //Voltage Unbalance L-N Worst
    Serial.print("meter.velocity:"); Serial.println( meter.velocity);
    Serial.print("meter.area:"); Serial.println( meter.area);
    Serial.print("meter.volt:"); Serial.println( meter.volt);  //Voltage Unbalance L-N Worst

    
  }
  Serial.println("readMeter.done");
}

void t1CallModbus() {     // Update read all data
  readMeter();
  drawSpriteSave();

  drawSpriteLevel();

}

float HexTofloat(uint32_t x)
{

  return (*(float*)&x);
}


float read_Modbus(uint16_t  REG)
{
  static uint32_t i;
  uint32_t j, result;
  uint16_t data[2];
  uint32_t value = 0;
  float val = 0.0;

  // communicate with Modbus slave ID 1 over Serial (port 2)
  node.begin(ID_Meter, modbus);

  // slave: read (6) 16-bit registers starting at register 2 to RX buffer
  result = node.readHoldingRegisters(REG, 2);
  Serial.print("result:"); Serial.print(result); Serial.print(" node.ku8MBSuccess:"); Serial.println(node.ku8MBSuccess);
  // do something with data if read is successful
  if (result == node.ku8MBSuccess)
  {
    for (j = 0; j < 2; j++)
    {
      data[j] = node.getResponseBuffer(j);
      //      SerialBT.print(REG); SerialBT.print(":"); SerialBT.print(j); SerialBT.print(":");  SerialBT.println(data[j]);
      //      Serial.print(REG); Serial.print(":"); Serial.print(j); Serial.print(":");  Serial.println(data[j]);

    }
    value = data[0];
    value = value << 16;
    value = value + data[1];

    val = HexTofloat(value);

    readModbusStatus = true;
    return val;
  } else {
    Serial.print("Connec modbus fail. REG >>> "); Serial.println(REG, HEX); // Debug
    readModbusStatus = false;
    return 0;
  }
}
String decToHex(int decValue) {

  String hexString = String(decValue, HEX);
  return hexString;
}

unsigned int hexToDec(String hexString) {

  unsigned int decValue = 0;
  int nextInt;

  for (int i = 0; i < hexString.length(); i++) {

    nextInt = int(hexString.charAt(i));
    if (nextInt >= 48 && nextInt <= 57) nextInt = map(nextInt, 48, 57, 0, 9);
    if (nextInt >= 65 && nextInt <= 70) nextInt = map(nextInt, 65, 70, 10, 15);
    if (nextInt >= 97 && nextInt <= 102) nextInt = map(nextInt, 97, 102, 10, 15);
    nextInt = constrain(nextInt, 0, 15);

    decValue = (decValue * 16) + nextInt;
  }

  return decValue;
}
int getResult( unsigned int x_high, unsigned int x_low)
{
  String hex2 = "";
  hex2.concat(decToHex(x_low));
  hex2.concat(decToHex(x_high));
  Serial.print("hex:");  Serial.println(hex2);
  Serial.print("dec:");  Serial.println(hexToDec(hex2));                                                               //rightmost 8 bits
  return hexToDec(hex2);
}

/****************************************************
   [通用函数]ESP32 WiFi Kit 32事件处理
*/
void WiFiEvent(WiFiEvent_t event)
{
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch (event)
  {
    case SYSTEM_EVENT_WIFI_READY:               /**< ESP32 WiFi ready */
      break;
    case SYSTEM_EVENT_SCAN_DONE:                /**< ESP32 finish scanning AP */
      break;

    case SYSTEM_EVENT_STA_START:                /**< ESP32 station start */
      break;
    case SYSTEM_EVENT_STA_STOP:                 /**< ESP32 station stop */
      break;

    case SYSTEM_EVENT_STA_CONNECTED:            /**< ESP32 station connected to AP */
      break;

    case SYSTEM_EVENT_STA_DISCONNECTED:         /**< ESP32 station disconnected from AP */
      break;

    case SYSTEM_EVENT_STA_AUTHMODE_CHANGE:      /**< the auth mode of AP connected by ESP32 station changed */
      break;

    case SYSTEM_EVENT_STA_GOT_IP:               /**< ESP32 station got IP from connected AP */
    case SYSTEM_EVENT_STA_LOST_IP:              /**< ESP32 station lost IP and the IP is reset to 0 */
      break;

    case SYSTEM_EVENT_STA_WPS_ER_SUCCESS:       /**< ESP32 station wps succeeds in enrollee mode */
    case SYSTEM_EVENT_STA_WPS_ER_FAILED:        /**< ESP32 station wps fails in enrollee mode */
    case SYSTEM_EVENT_STA_WPS_ER_TIMEOUT:       /**< ESP32 station wps timeout in enrollee mode */
    case SYSTEM_EVENT_STA_WPS_ER_PIN:           /**< ESP32 station wps pin code in enrollee mode */
      break;

    case SYSTEM_EVENT_AP_START:                 /**< ESP32 soft-AP start */
    case SYSTEM_EVENT_AP_STOP:                  /**< ESP32 soft-AP stop */
    case SYSTEM_EVENT_AP_STACONNECTED:          /**< a station connected to ESP32 soft-AP */
    case SYSTEM_EVENT_AP_STADISCONNECTED:       /**< a station disconnected from ESP32 soft-AP */
    case SYSTEM_EVENT_AP_PROBEREQRECVED:        /**< Receive probe request packet in soft-AP interface */
    case SYSTEM_EVENT_AP_STA_GOT_IP6:           /**< ESP32 station or ap interface v6IP addr is preferred */
      break;

    case SYSTEM_EVENT_ETH_START:                /**< ESP32 ethernet start */
    case SYSTEM_EVENT_ETH_STOP:                 /**< ESP32 ethernet stop */
    case SYSTEM_EVENT_ETH_CONNECTED:            /**< ESP32 ethernet phy link up */
    case SYSTEM_EVENT_ETH_DISCONNECTED:         /**< ESP32 ethernet phy link down */
    case SYSTEM_EVENT_ETH_GOT_IP:               /**< ESP32 ethernet got IP from connected AP */
    case SYSTEM_EVENT_MAX:
      break;
  }
}
