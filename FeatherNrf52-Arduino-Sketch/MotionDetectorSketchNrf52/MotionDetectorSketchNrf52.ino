#include <bluefruit.h>

// Using Custom Values
/* Home Environment Sensing Service Definitions
   Home Environment Service:  0x28FF
   Motion Sensing Char: 0x4A37
*/


#define STATUS_LED (19)

// Power Reduction: https://github.com/adafruit/Adafruit_nRF52_Arduino/issues/165
// Serial seems to increase consumption by 500uA https://github.com/adafruit/Adafruit_nRF52_Arduino/issues/51#issuecomment-368289198
//#define DEBUG

#ifdef DEBUG
  #define DEBUG_PRINT(x)  Serial.print(x)
  #define DEBUG_PRINTLN(x)  Serial.println(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif
  
// TODO Consider appending BT Mac last 4 digits.
const char* DEVICENAME = "Motion HP%X%X";
const char* DEVICE_MODEL = "HomePi Motion";
const char* DEVICE_MANUFACTURER = "Rounin Labs";

/* Home Environmental Sensing Service.
 * This service exposes measurement data from an home sensor intended for home automation applications. 
 * A wide range of environmental parameters is supported.
 */
const int UUID16_SVC_HOME_ENV_SENSE = 0x28FF;
// Value will be 1 when motion is detected.
const int UUID16_CHR_MOTION_SENSE_MEASUREMENT = 0x4A37;

const int BATTERY_INFO_PIN = A7;
const int LOW_BATT = 25;
int lastBattLevel = 0;
const int BATT_REPORTING_INTERVAL = 60000; // Interval at which the battery is reported in MS

BLEService        hess = BLEService(UUID16_SVC_HOME_ENV_SENSE);
BLECharacteristic msc = BLECharacteristic(UUID16_CHR_MOTION_SENSE_MEASUREMENT);

BLEDis bledis;    // DIS (Device Information Service) helper class instance
BLEBas blebas;    // BAS (Battery Service) helper class instance

bool bMotionDetected = false;

static volatile bool bMotionChangeDetected = false;

const int iMotionInterruptPin = A3;
// Pin where the sensor output is connected to on the Feather
const int MOTION_SENSOR_OUTPUT_PIN = iMotionInterruptPin;

static TaskHandle_t _notifyMotionValuesHandle;
uint32_t _notifyMotionValuesStackSize = 512;
// define notify motion task.
void TaskNotifyMotion( void *pvParameters );

void setup() {

  #ifdef DEBUG
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb
  #endif
  
  DEBUG_PRINTLN("Setting up Motion Sensor");
  DEBUG_PRINTLN("-----------------------\n");
  
  pinMode(STATUS_LED, OUTPUT);
 
  // Initialise the Bluefruit module
  DEBUG_PRINTLN("Initialise the Bluefruit nRF52 module");
  Bluefruit.begin();

  Bluefruit.autoConnLed(false);
  // Set the advertised device name (keep it short!)
  DEBUG_PRINT("Setting Device Name to ");
  uint8_t address [6];
  Bluefruit.Gap.getAddr(address);
  char nameBuff[50] = "";
  sprintf(nameBuff, DEVICENAME, address[1],address[0]);
  DEBUG_PRINTLN(nameBuff);
  Bluefruit.setName(nameBuff);

  // Set the connect/disconnect callback handlers
  Bluefruit.setConnectCallback(connect_callback);
  Bluefruit.setDisconnectCallback(disconnect_callback);

  // Configure and Start the Device Information Service
  DEBUG_PRINTLN("Configuring the Device Information Service");
  bledis.setManufacturer(DEVICE_MANUFACTURER);
  bledis.setModel(DEVICE_MODEL);
  bledis.begin();

  // Start the BLE Battery Service and set it to 100%
  DEBUG_PRINTLN("Configuring the Battery Service");
  blebas.begin();
  blebas.write(100);

  // Setup the Home Environment Sensing service using
  // BLEService and BLECharacteristic classes
  DEBUG_PRINTLN("Configuring the Motion Sensor Service");
  setupMS();

  // Setup the advertising packet(s)
  DEBUG_PRINTLN("Setting up the advertising payload(s)");
  startAdv();
  DEBUG_PRINTLN("\nAdvertising");

  //Setup Motion Sensor.
  setupMotionSensor();
  setupMotionDectectionInterrupts();
  
  // Setup FreeRTOS notification tasks
  DEBUG_PRINTLN("Setting up FreeRTOS notification task(s)");
  setupNotificationTasks();

  // Configure Low Power Mode
  // Note: Seems to put the device to sleep and disconnects
  //sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
  //sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
  
  // TODO Consider suspendLoop() to save power, since we probably won't have any code there
  //suspendLoop();

  DEBUG_PRINTLN("\nSetup Complete!");
}

void setupNotificationTasks()
{
  // Create the notification task. 
  xTaskCreate(
    TaskNotifyMotion
    ,  (const portCHAR *)"NotifyMotion"   // A name just for humans
    ,  _notifyMotionValuesStackSize  // Stack size
    ,  NULL // Parameters, should be an address to a variable on the heap, not the stack.
    ,  TASK_PRIO_LOW  // priority
    ,  &_notifyMotionValuesHandle ); // Task handle
}

/**
 * FreeRTOS task to notify motion changes
 */
void TaskNotifyMotion( void *pvParameters )
{
  (void) pvParameters;
  for (;;)
  {
    

    if ( Bluefruit.connected() )
    {
      DEBUG_PRINTLN("Notifying Sensor Data");
      int batteryLevel = readBatteryLevel();
  
      // Notify the battery level 
      notifyBatteryLevel(batteryLevel);
      lastBattLevel = batteryLevel;
      
      if(batteryLevel < LOW_BATT)
      {
        digitalToggle(LED_RED);
      }
  
      if(bMotionChangeDetected)
      {
        bMotionDetected = digitalRead(MOTION_SENSOR_OUTPUT_PIN) == HIGH;
        notifyMotionDetectionValue(bMotionDetected);
        bMotionChangeDetected = false;
      }
    }
    else
    {
      digitalToggle(STATUS_LED);
    }
    
    // The task suspends itself after notifying.
    vTaskSuspend( NULL );
  }
}

void connect_callback(uint16_t conn_handle)
{
  char central_name[32] = { 0 };
  Bluefruit.Gap.getPeerName(conn_handle, central_name, sizeof(central_name));

  DEBUG_PRINT("Connected to ");
  DEBUG_PRINTLN(central_name);
  // Disable the BT connection LED to save battery.
  digitalWrite(STATUS_LED, LOW);
}

/**
   Callback invoked when a connection is dropped
   @param conn_handle connection where this event happens
   @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
   https://github.com/adafruit/Adafruit_nRF52_Arduino/blob/master/cores/nRF5/nordic/softdevice/s140_nrf52_6.1.1_API/include/ble_hci.h
*/
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  DEBUG_PRINTLN("Disconnected");

  // Consider ligthing LED when it is disconnected.
}

void setupMS(void)
{
  // Configure the Motion Sensing service
  // Supported Characteristics:
  // Name                         UUID    Requirement Properties
  // ---------------------------- ------  ----------- ----------
  // Motion Sense Measurement     0x4A37  Mandatory   Notify
  hess.begin();

  // Note: You must call .begin() on the BLEService before calling .begin() on
  // any characteristic(s) within that service definition.. Calling .begin() on
  // a BLECharacteristic will cause it to be added to the last BLEService that
  // was 'begin()'ed!

  // Configure the Motion Sense Measurement characteristic
  // See: https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.characteristic.heart_rate_measurement.xml
  // Properties = Notify
  // Min Len    = 1
  // Max Len    = 8
  //    B0      = UINT8  - Flag (MANDATORY)
  //      b5:7  = Reserved
  //      b4    = RR-Internal (0 = Not present, 1 = Present)
  //      b3    = Energy expended status (0 = Not present, 1 = Present)
  //      b1:2  = Sensor contact status (0+1 = Not supported, 2 = Supported but contact not detected, 3 = Supported and detected)
  //      b0    = Value format (0 = UINT8, 1 = UINT16)
  //    B1      = UINT8  - 8-bit heart rate measurement value in BPM
  //    B2:3    = UINT16 - 16-bit heart rate measurement value in BPM
  //    B4:5    = UINT16 - Energy expended in joules
  //    B6:7    = UINT16 - RR Internal (1/1024 second resolution)
  msc.setProperties(CHR_PROPS_NOTIFY);
  msc.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  msc.setFixedLen(2);
  msc.setCccdWriteCallback(cccd_callback);  // Optionally capture CCCD updates
  msc.setUserDescriptor("Motion Sense Measurement. A value of 1 means detected.");
  msc.begin();
  uint8_t hrmdata[1] = { 0 }; // Set the characteristic to use 8-bit values, with the sensor connected and detected
  msc.notify(hrmdata, 1);                   // Use .notify instead of .write!
}

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include Motion Sensor Service UUID
  Bluefruit.Advertising.addService(hess);

  // Include Name
  Bluefruit.Advertising.addName();

  /* Start Advertising
     - Enable auto advertising if disconnected
     - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
     - Timeout for fast mode is 30 seconds
     - Start(timeout) with timeout = 0 will advertise forever (until connected)

     For recommended advertising interval
     https://developer.apple.com/library/content/qa/qa1931/_index.html
  */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds
}

/**
   Set up to read from the Motion Sensor ouput pin.
*/
void setupMotionSensor()
{
  pinMode(MOTION_SENSOR_OUTPUT_PIN, INPUT_PULLUP);
}

void setupMotionDectectionInterrupts()
{
  attachInterrupt(digitalPinToInterrupt(iMotionInterruptPin), motionChanged, CHANGE);
}

void cccd_callback(BLECharacteristic& chr, uint16_t cccd_value)
{
  // Display the raw request packet
  DEBUG_PRINT("CCCD Updated: ");
  //Serial.printBuffer(request->data, request->len);
  DEBUG_PRINT(cccd_value);
  DEBUG_PRINTLN("");

  // Check the characteristic this CCCD update is associated with in case
  // this handler is used for multiple CCCD records.
  if (chr.uuid == msc.uuid) 
  {
    if (chr.notifyEnabled()) 
    {
      DEBUG_PRINTLN("Motion Sensing Measurement 'Notify' enabled");
    } 
    else 
    {
      DEBUG_PRINTLN("Motion Sensing Measurement 'Notify' disabled");
    }
  }
}

void loop() {
  // put your main code here, to run repeatedly:

  if ( Bluefruit.connected() )
  {
    int batteryLevel = readBatteryLevel();

    // Notify the battery level only if it has changed.
    if(batteryLevel != lastBattLevel)
    {
      notifyBatteryLevel(batteryLevel);
      lastBattLevel = batteryLevel;
    }
  }

  // Only send update once per second
  delay(BATT_REPORTING_INTERVAL);

}

/**
   Interrupt function called when
   the sensor goes from Low to High.
   This will happen when it goes from
   not detecting motion to detecting motion.
*/
void motionChanged()
{
  DEBUG_PRINTLN("Motion Detected");
  bMotionChangeDetected = true;

  
  BaseType_t yieldRequired;

  //Resume suspsned task. 
  yieldRequired = xTaskResumeFromISR(_notifyMotionValuesHandle);

  if (yieldRequired == pdTRUE)
  {
    taskYIELD();
  }
  //bMotionDetected = digitalRead(MOTION_SENSOR_OUTPUT_PIN) == HIGH;
  //notifyMotionDetectionValue(bMotionDetected);
}

void notifyMotionDetectionValue(boolean motionDetected)
{
  if (!Bluefruit.connected())
  {
    return;
  }
  uint8_t mdmdata[1] = { motionDetected ? 1 : 0 };   // Set to 1 if motion is detected orwise 0

  // Note: We use .notify instead of .write!
  // If it is connected but CCCD is not enabled
  // The characteristic's value is still updated although notification is not sent

  if (msc.notify(mdmdata, sizeof(mdmdata)) )
  {
    DEBUG_PRINT("Motion Sense Measurement updated to: "); 
    DEBUG_PRINTLN(mdmdata[0]);
  }
  else
  {
    DEBUG_PRINTLN("ERROR: Notify not set in the CCCD or not connected!");
  }
  
}

/**
   Helper function to notify the battery level.
*/
void notifyBatteryLevel(int level)
{
  blebas.notify(level);
}

/**
   Reads the battery level from the feather pin.
   Note: Updated from: https://learn.adafruit.com/adafruit-feather-32u4-basic-proto/power-management
*/

/**
 * Excerpt From https://github.com/adafruit/Adafruit_nRF52_Arduino/blob/master/libraries/Bluefruit52Lib/examples/Hardware/adc_vbat/adc_vbat.ino
 */
#define VBAT_PIN          (A7)
#define VBAT_MV_PER_LSB   (0.73242188F)   // 3.0V ADC range and 12-bit ADC resolution = 3000mV/4096
#define VBAT_DIVIDER      (0.71275837F)   // 2M + 0.806M voltage divider on VBAT = (2M / (0.806M + 2M))
#define VBAT_DIVIDER_COMP (1.403F)        // Compensation factor for the VBAT divider

int readVBAT(void) 
{
  int raw;

  // Set the analog reference to 3.0V (default = 3.6V)
  analogReference(AR_INTERNAL_3_0);

  // Set the resolution to 12-bit (0..4095)
  analogReadResolution(12); // Can be 8, 10, 12 or 14

  // Let the ADC settle
  delay(1);

  // Get the raw 12-bit, 0..3000mV ADC value
  raw = analogRead(VBAT_PIN);

  // Set the ADC back to the default settings
  analogReference(AR_DEFAULT);
  analogReadResolution(10);

  return raw;
}

uint8_t mvToPercent(float mvolts) {
    uint8_t battery_level;

    if (mvolts >= 3000)
    {
        battery_level = 100;
    }
    else if (mvolts > 2900)
    {
        battery_level = 100 - ((3000 - mvolts) * 58) / 100;
    }
    else if (mvolts > 2740)
    {
        battery_level = 42 - ((2900 - mvolts) * 24) / 160;
    }
    else if (mvolts > 2440)
    {
        battery_level = 18 - ((2740 - mvolts) * 12) / 300;
    }
    else if (mvolts > 2100)
    {
        battery_level = 6 - ((2440 - mvolts) * 6) / 340;
    }
    else
    {
        battery_level = 0;
    }

    return battery_level;
}

int readBatteryLevel()
{
  /*
  float measuredvbat = analogRead(BATTERY_INFO_PIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  Serial.print("VBat: " ); Serial.println(measuredvbat);
  Serial.print("CBat: " ); Serial.println(map(measuredvbat, 3.0, 4.2, 0, 100));
  */
   int vbat_raw = readVBAT();

  // Convert from raw mv to percentage (based on LIPO chemistry)
  uint8_t vbat_per = mvToPercent(vbat_raw * VBAT_MV_PER_LSB);

  // Possible fix for exception waking the CPU
  #define FPU_EXCEPTION_MASK 0x0000009F 
  __set_FPSCR(__get_FPSCR() & ~(FPU_EXCEPTION_MASK)); 
  (void) __get_FPSCR();
  NVIC_ClearPendingIRQ(FPU_IRQn);

 
  DEBUG_PRINT("VBat: " ); DEBUG_PRINT(vbat_per); DEBUG_PRINTLN("%");
  return vbat_per;
}
