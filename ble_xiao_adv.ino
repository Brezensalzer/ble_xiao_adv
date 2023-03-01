//-------------------------------------------------------------------------------
// ble_xiao_dv.ino
// seeed studio XIAO BLE MAC Address F6:73:3E:2A:1C:FA
//-------------------------------------------------------------------------------
#include <bluefruit.h>
#include <ble_gap.h>

// shutdown flash memory
#include "Adafruit_SPIFlash.h"
#include "SdFat.h"
#include <SPI.h>
// for flashTransport definition
#include "flash_config.h"

Adafruit_SPIFlash flash(&flashTransport);

// save power without serial interface...
//#define DEBUG

struct ATTR_PACKED
{
  uint16_t manufacturer;
  uint16_t temperature;
  uint16_t humidity;
  uint16_t pressure;
  uint16_t battery;
} sensor_data;

//--- HTU21D temperature and humidity sensor ---
#include <Wire.h>
#include "SparkFunHTU21D.h"
HTU21D htu;

#define VCC_I2C 10    // define the GPIO pin used as i2c vcc

//---- ADC Battery Monitoring ----------------------------
const int adcPin = PIN_VBAT;

//------------------------------------------------------------------------------
int batteryVoltage()
//------------------------------------------------------------------------------
{
  analogReference(AR_INTERNAL); // reference voltage 0..3.6V
  analogReadResolution(12);     // analog resolution 12bit
  int adcValue = analogRead(adcPin);
  adcValue = analogRead(adcPin);
  #ifdef DEBUG
    Serial.print("Battery: ");
    Serial.println(adcValue);
  #endif
  return adcValue;  // mV
}

//---------------------------------------------------------------------------
void sensorData()
//---------------------------------------------------------------------------
{  
  //--- power on i2c vcc --------------------------------------------
  pinMode(VCC_I2C, OUTPUT);
  digitalWrite(VCC_I2C, HIGH);
  #ifdef DEBUG
    Serial.println("setting pin VCC_I2C to HIGH");
  #endif
  delay(100); // sensor boot time

  // Initialize sensor --------------------------
  Wire.begin();
  htu.begin();
  
  // read sensor data
  float celsius = htu.readTemperature();
  float humidity = htu.readHumidity();
  if (humidity > 100.0)
  { humidity = 100.0; }
  float pressure = 0.0;
  float battery = 0.0;

  #ifdef DEBUG
    Serial.print("Temperatur: ");
    Serial.println(celsius);
    Serial.print("Luftfeuchte: ");
    Serial.println(humidity);
    Serial.print("Luftdruck: ");
    Serial.println(pressure);
  #endif

  sensor_data.manufacturer = 0x0822;
  sensor_data.temperature = int(celsius * 100.0);
  sensor_data.humidity = int(humidity * 100.0);
  sensor_data.pressure = int(pressure * 10.0); 
  sensor_data.battery = batteryVoltage();

  Bluefruit.Advertising.addData(BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA, &sensor_data, sizeof(sensor_data));

  // sensor sleep 
  Wire.end();
  
  //--- power off i2c vcc --------------------------------------------
  digitalWrite(VCC_I2C, LOW);
  pinMode(VCC_I2C, INPUT);
  #ifdef DEBUG
    Serial.println("setting pin VCC_I2C to LOW");
  #endif  
}

//---------------------------------------------------------------------------
void startAdvertising()
//---------------------------------------------------------------------------
{   
  #ifdef DEBUG
    Serial.println("Starting to Advertise");
  #endif

  Bluefruit.Advertising.clearData();
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.setType(BLE_GAP_ADV_TYPE_NONCONNECTABLE_SCANNABLE_UNDIRECTED);
  Bluefruit.setName("Brezensalzer XIAO");
  Bluefruit.ScanResponse.addName();
  Bluefruit.setTxPower(4);
  
  sensorData();

  Bluefruit.Advertising.restartOnDisconnect(false);
  Bluefruit.Advertising.setInterval(250, 250);
  Bluefruit.Advertising.setFastTimeout(1);
  Bluefruit.Advertising.start(1);
}

//---------------------------------------------------------------------------
void stopAdvertising()
//---------------------------------------------------------------------------
{
  #ifdef DEBUG
    Serial.println("Clearing Advertisement Data");
    Serial.println("----------------------------------");
  #endif
  Bluefruit.Advertising.clearData();
  Bluefruit.Advertising.stop();
}

//---------------------------------------------------------------------------
void setup() 
//---------------------------------------------------------------------------
{
  #ifdef DEBUG
    Serial.begin(115200);
    int i = 0;
    while ( !Serial ) 
    {
      i++;
      if (i > 20) break;
      delay(100);   // for nrf52840 with native usb
    }
    Serial.println("Seeed XIAO nRF52840 Environmental Sensor");
    Serial.println("----------------------------------------");
  #endif
  #ifndef DEBUG
    Serial.end();   // power saving
  #endif
  
  if (!Bluefruit.begin())
  {
    #ifdef DEBUG
      Serial.println("Unable to init Bluefruit");
    #endif
    while(1)
    {
      digitalToggle(LED_RED);
      delay(100);
    }
  }

  // before you put the NRF to sleep put the flash to sleep
  // this saves about 16 µA
  flash.begin();
  flashTransport.runCommand(0xB9);
  flashTransport.end();
}

//---------------------------------------------------------------------------
void loop() 
//---------------------------------------------------------------------------
{
  
  // losing 4.6 µA
  delay(59000);
  startAdvertising();
  delay(500);
  stopAdvertising();
  //--- brute force soft reset ----------------
  // otherwise we lose 274 µA
  NVIC_SystemReset();
}
