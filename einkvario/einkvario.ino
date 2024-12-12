/*
    LilyGo 墨水屏+MS5611模块
    组成高度表
    可蓝牙与XCTrack连接使用
        - dangxj 2022-11
    alphabeta滤波计算升速
    单独任务声音提示
    更换蜂鸣放大器，按键开关声音 2024-3
    按键切换音量，第二次超过阈值才响 2024-6
    
*/

#define LILYGO_T5_V213
#include <Wire.h>
#include <TimeLib.h>
#include <OneButton.h>

#define ENABLE_GxEPD2_GFX 0
#include <GxEPD2_BW.h>
#include <U8g2_for_Adafruit_GFX.h>
#include <Fonts/FreeSerifBold18pt7b.h>
#include <Fonts/FreeSerifBold12pt7b.h>

#include <MS5611.h>
//#include <ESP32_tone.h>
//#include <TinyGPSPlus.h>
#include <AlphaBeta.h>
#define SERVICE_UUID           "E079C6A0-AA8B-11E3-A903-0002A5D5C51B"
//#define CHARACTERISTIC_UUID_RX "0000ffe1-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID_TX "0000ffe1-0000-1000-8000-00805f9b34fb"
//#define CHARACTERISTIC_UUID_TX "B38312C0-AA89-11E3-9CEF-0002A5D5C51B"

#include <NimBLEDevice.h>
NimBLEServer* pServer = NULL;
NimBLEService* pService = NULL;
NimBLECharacteristic* pTxCharacteristic = NULL;
//NimBLECharacteristic* pRxCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

//#include GxEPD_BitmapExamples
GxEPD2_BW<GxEPD2_213_B73, GxEPD2_213_B73::HEIGHT> display(GxEPD2_213_B73(/*CS=5*/ SS, /*DC=*/ 17, /*RST=*/ 16, /*BUSY=*/ 4)); // GDEH0213B74
//GxEPD2_BW<GxEPD2_213_WS1, GxEPD2_213_WS1::HEIGHT> display(GxEPD2_213_WS1(/*CS=5*/ SS, /*DC=*/ 17, /*RST=*/ 16, /*BUSY=*/ 4)); // GDEH0213B74
U8G2_FOR_ADAFRUIT_GFX u8g2Fonts;
const int beepPin = 12;
const int batPin = 35;
const int rxPin=14;
const int btnPin=39;
const int ledPin=19;
int isound=2;

OneButton btn = OneButton(btnPin,true,true);

MS5611 ms5611(0x77);
//ESP32_tone buzzer(beepPin);
AlphaBeta filter;
const float GAIN = 0.8;

const float sea_press = 1025.25;
//全局数据变量
int beepTotal[10]={16,14,12,10,9,8,7,6,5,4};
int beepOn[10]={10,9,8,7,6,5,4,3,2,2};
const float downLimit = -2.6, upLimit = 0.6;
const int downFreq=1000,upFreq=1600;

int rate;
SemaphoreHandle_t mutex_v;
TaskHandle_t dataHandle,displayHandle,beepHandle;
float pressure, vspeed,vspeed0, temperature,atmAlti;
unsigned long milli0 = 0, milli1, millibp = 0;
portTickType xPreWkTmData,xPreWkTmDisplay,xPreWkTmBeep;
int disOff = 5; //displayOffset
int ifullrefresh=0;

void soundOnOff() {
  isound++;
  if(isound>3) isound=0;
  if (isound>0)
    vTaskResume(beepHandle);
  else
    vTaskSuspend(beepHandle);
}

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      soundOnOff();
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      soundOnOff();
    }
};

float getBattery(){
  float vbat= analogRead(batPin)*6.6*1.1/4095;
  float p=(vbat-3.7)*2;// (4.2-3.7)线性
  if(p<0) p=0;
  if(p>1) p=1;
  return p;
}

float  getAltitude(float p, float temp) {
  if (p < 1) p = 1;
  float dat=((pow((sea_press / p), 1 / 5.257) - 1.0) * (temp + 273.15)) / 0.0065;
  if(dat<-1000) dat=-1000;
  if(dat>10000) dat/=10000;
  return dat;
}

static void displayData(float val, int16_t x, int16_t y, int16_t d = 0)
{
  //display.getTextBounds(str, x, y, &x1, &y1, &w, &h);
  display.setCursor(x, y);
  display.print(val, d);
}

// since BLE packets can only be 20 bytes, we split the string to be sent in chunks of 20 bytes and send them
void send_ble_uart(const char* str, int size) {
  int offset = 0;
  int remainingLength = size;
  while (remainingLength > 0) {
    int bytesToSend = min(20, remainingLength);
    pTxCharacteristic->setValue((const uint8_t*)(str + offset), bytesToSend);
    pTxCharacteristic->notify();
    remainingLength -= bytesToSend;
    offset += bytesToSend;
  }
  //Serial.print(str);
}

void sendXCTrack()
{
  //$LK8EX1,pressure,altitude,vario,temperature,battery,*checksum
  //$LK8EX1,98684,99999,-4,28,1100,*02<CR><LF>
  //https://github.com/LK8000/LK8000/blob/master/Docs/LK8EX1.txt

  String str_out = String(
                     "LK8EX1,"
                     + String(int(pressure * 100), DEC) + ","
                     + String(int(atmAlti), DEC)
                     + ",9999,"
                     + String(int(temperature), DEC)
                     + String(",999,")
                   );

  unsigned int checksum_end, ai, bi;   // Calculating checksum for data string
  for (checksum_end = 0, ai = 0; ai < str_out.length(); ai++)
  {
    bi = (unsigned char)str_out[ai];
    checksum_end ^= bi;
  }
  //creating now NMEA serial output for LK8000. LK8EX1 protocol format:
  //$LK8EX1,pressure,altitude,vario,temperature,battery,*checksum

  String strble = String("$" + str_out + "*" + String(checksum_end, HEX) + "\r\n");
  send_ble_uart(strble.c_str(), strble.length());
}

void  readData()
{
    if(ms5611.read()!= MS5611_READ_OK)
      ms5611.begin();
    pressure = ms5611.getPressure();
    temperature = ms5611.getTemperature();
    atmAlti=getAltitude(pressure, temperature);
    AlphaBeta::updateTimer();
    if(filter.timeStep()<0.05)
      return;//错乱数据
    filter.update( atmAlti, GAIN );
    vspeed0=vspeed;
    vspeed = filter.deriv();
    //Serial.println(vspeed);

}

void updateVario(void * pvParameters)
{
  while (1) {
    xSemaphoreTake(mutex_v, portMAX_DELAY);
    readData();
    if (deviceConnected)
      sendXCTrack();
    //Serial.print("    ");
    //Serial.println(vspeed);
    xSemaphoreGive(mutex_v); //
    vTaskDelayUntil(&xPreWkTmData,pdMS_TO_TICKS(100));
    //vTaskDelay(pdMS_TO_TICKS(85));
  }  
}

void  updateDisplay(void * pvParameters)
{
  while (1) {
    //更新显示 280X122
    display.fillRect(0, 0, 122, 30, GxEPD_BLACK);
    display.fillRect(disOff + 30, 30, 92, 220, GxEPD_WHITE);

    xSemaphoreTake(mutex_v, portMAX_DELAY);
    display.setTextColor(GxEPD_BLACK);
    display.setFont(&FreeSerifBold18pt7b);
    if (fabs(vspeed) < 10)
      displayData(vspeed, disOff + 40, 65, 1);
    displayData(atmAlti, disOff + 35, 115);
    displayData(temperature, disOff + 40, 165);
    display.setFont(&FreeSerifBold12pt7b);
    displayData(pressure, disOff + 40, 210,2);
    display.setCursor(disOff + 50,245);
    display.print(String(hour())+':'+String(minute()));
    display.setTextColor(GxEPD_WHITE);
    u8g2Fonts.setCursor(disOff, 20);
    if(deviceConnected)
      u8g2Fonts.print("XC");
    else
      u8g2Fonts.print("蓝牙");
    u8g2Fonts.setCursor(disOff + 40,20);
    u8g2Fonts.print("音量"+String(isound));
    display.setCursor(disOff + 80,20);
    display.print(getBattery()*100,0);
    //display.print("%");
    xSemaphoreGive(mutex_v); //

    //display.displayWindow(0, 0, 122, 220);
    if(ifullrefresh++>60)
    {
      ifullrefresh=0;
      display.display(false);
      //display.refresh(true);
      //vTaskDelay(pdMS_TO_TICKS(1000));
    }
    else
      display.display(true);
    display.hibernate();
    //display.powerOff();
    vTaskDelayUntil(&xPreWkTmDisplay,pdMS_TO_TICKS(2000));
  }
}

void beep(void *param)
{
  while (1)
  {
    int ntms=70;
    int nrate=abs(vspeed);
    if(nrate>9) nrate=9;
    //Serial.println(vspeed);
    if(vspeed>upLimit &&vspeed0>upLimit )
    {
      tone(beepPin,upFreq+isound*100+30*vspeed,beepOn[nrate]*ntms);
      //noTone(beepPin);
      vTaskDelay(beepOn[nrate]*ntms / portTICK_PERIOD_MS ); // 等待
    }
    else if(vspeed<downLimit&&vspeed0<downLimit)
    {
      int tdown=beepOn[nrate]*ntms/3;
      tone(beepPin,downFreq+isound*100+10*vspeed,tdown);
      vTaskDelay(tdown / portTICK_PERIOD_MS ); 
      tone(beepPin,downFreq+isound*100+10*vspeed-100,tdown);
      vTaskDelay(tdown / portTICK_PERIOD_MS ); 
      tone(beepPin,downFreq+isound*100+10*vspeed-200,tdown);
      vTaskDelay(tdown / portTICK_PERIOD_MS ); 
    }
    vTaskDelay((beepTotal[nrate]-beepOn[nrate])*ntms / portTICK_PERIOD_MS ); // 等待
  }
}

void setup()
{
  setCpuFrequencyMhz(80);
  //Serial.begin(9600);
  pinMode(batPin, OUTPUT);
  pinMode(beepPin, OUTPUT);
  pinMode(ledPin,OUTPUT);
  setTime(0,0,0,1,1,2024);

  display.init();
  display.setRotation(2);
  display.setTextColor(GxEPD_BLACK);
  display.setFont(&FreeSerifBold18pt7b);
  //display.clearScreen(0x00);
  display.fillScreen(GxEPD_WHITE);

  u8g2Fonts.begin(display);
  u8g2Fonts.setFontMode(1);                           // use u8g2 transparent mode (this is default)
  u8g2Fonts.setFontDirection(0);
  u8g2Fonts.setFont(u8g2_font_wqy16_t_gb2312a);            // select u8g2 font from here: https://github.com/olikraus/u8g2/wiki/

  display.fillRect(0, 0, 125, 30, GxEPD_BLACK);
  u8g2Fonts.setForegroundColor(GxEPD_WHITE);
  u8g2Fonts.setCursor(disOff, 20);
  u8g2Fonts.print("蓝牙");
  u8g2Fonts.setForegroundColor(GxEPD_BLACK);
  u8g2Fonts.setBackgroundColor(GxEPD_WHITE);
  u8g2Fonts.setCursor(disOff, 60);
  u8g2Fonts.print("升速");
  u8g2Fonts.setCursor(disOff, 110);
  u8g2Fonts.print("高度");
  u8g2Fonts.setCursor(disOff, 160);
  u8g2Fonts.print("温度");
  u8g2Fonts.setCursor(disOff, 210);
  u8g2Fonts.print("气压");
  u8g2Fonts.setCursor(disOff, 245);
  u8g2Fonts.print("时间");
  u8g2Fonts.setForegroundColor(GxEPD_WHITE);
  u8g2Fonts.setBackgroundColor(GxEPD_BLACK);

  display.display(false);

  Wire.begin();
  ms5611.begin();
  ms5611.setOversampling(OSR_ULTRA_HIGH);
  ms5611.setTemperatureOffset(-10);//温度偏差，不知道原因，新传感器也是高10度
  delay(300);
  readData();
  delay(50);
  filter.setState( atmAlti);
  AlphaBeta::updateTimer();

  //buzzer = ESP32_tone(beepPin);
  //buzzer.setCompatibleMode(true);
  mutex_v = xSemaphoreCreateMutex();
  BaseType_t xReturned;
  xReturned = xTaskCreate(updateVario, "dataRead", 2560, NULL, 3, &dataHandle);
  xReturned = xTaskCreate(updateDisplay, "updateDisplay", 5120, NULL, 1, &displayHandle);
  xReturned = xTaskCreate(beep, "beep", 1280, NULL, 2, &beepHandle);
  xPreWkTmDisplay=xTaskGetTickCount();

  btn.attachClick(soundOnOff);

  NimBLEDevice::init("eink_vario");
  //NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  pServer = NimBLEDevice::createServer();
  //pServer->clear()
  pServer->setCallbacks(new MyServerCallbacks());
  pService = pServer->createService(SERVICE_UUID);
  
  pTxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, NIMBLE_PROPERTY::NOTIFY);
  //  pRxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX,
  //    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_ENC | NIMBLE_PROPERTY::WRITE_AUTHEN);
  //  pRxCharacteristic->setCallbacks(this->receiveCallback);
  pTxCharacteristic->setValue("$LK8EX1");
  pService->start();
  pServer->getAdvertising()->start();
  
  xPreWkTmData=xTaskGetTickCount();

}
void loop()
{
  btn.tick();
}
