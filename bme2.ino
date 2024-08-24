/**
   @file Environment_Monitoring.ino
   @author rakwireless.com
   @brief This sketch demonstrate how to get environment data from BME680
      and send the data to lora gateway.
   @version 0.1
   @date 2020-07-28
   @copyright Copyright (c) 2020
**/

#include <Arduino.h>
#include <LoRaWan-RAK4630.h> // Click to install library: http://librarymanager/ALL#SX126x-Arduino
#include <SPI.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h> // Click to install library: http://librarymanager/All#Adafruit_BME680

#include <ArduinoRS485.h>
#include <ArduinoModbus.h>

Adafruit_BME680 bme;

#define PIN_VBAT A0

uint32_t vbat_pin = PIN_VBAT;

#define VBAT_MV_PER_LSB (0.73242188F) // 3.0V ADC range and 12 - bit ADC resolution = 3000mV / 4096
#define VBAT_DIVIDER_COMP (1.73)      // Compensation factor for the VBAT divider, depend on the board

#define REAL_VBAT_MV_PER_LSB (VBAT_DIVIDER_COMP * VBAT_MV_PER_LSB)


// RAK4630 supply two LED
#ifndef LED_BUILTIN
#define LED_BUILTIN 35
#endif

#ifndef LED_BUILTIN2
#define LED_BUILTIN2 36
#endif

#define SEALEVELPRESSURE_HPA (1010.0)

bool doOTAA = true;   // OTAA is used by default.
#define SCHED_MAX_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE 60										  /**< Maximum number of events in the scheduler queue. */
#define LORAWAN_DATERATE DR_3									  /*LoRaMac datarates definition, from DR_0 to DR_5*/
#define LORAWAN_TX_POWER TX_POWER_0							/*LoRaMac tx power definition, from TX_POWER_0 to TX_POWER_15*/
#define JOINREQ_NBTRIALS 5										  /**< Number of trials for the join request. */
DeviceClass_t g_CurrentClass = CLASS_A;					/* class definition*/
LoRaMacRegion_t g_CurrentRegion = LORAMAC_REGION_EU868;    /* Region:EU868*/
lmh_confirm g_CurrentConfirm = LMH_UNCONFIRMED_MSG;				  /* confirm/unconfirm packet definition*/
uint8_t gAppPort = LORAWAN_APP_PORT;							        /* data port*/

/**@brief Structure containing LoRaWan parameters, needed for lmh_init()
*/
static lmh_param_t g_lora_param_init = {LORAWAN_ADR_OFF, LORAWAN_DATERATE, LORAWAN_PUBLIC_NETWORK, JOINREQ_NBTRIALS, LORAWAN_TX_POWER, LORAWAN_DUTYCYCLE_OFF};

// Foward declaration
static void lorawan_has_joined_handler(void);
void lorawan_join_fail(void);
static void lorawan_rx_handler(lmh_app_data_t *app_data);
static void lorawan_confirm_class_handler(DeviceClass_t Class);
static void send_lora_frame(void);

/**@brief Structure containing LoRaWan callback functions, needed for lmh_init()
*/
static lmh_callback_t g_lora_callbacks = {BoardGetBatteryLevel, BoardGetUniqueId, BoardGetRandomSeed,
                                        lorawan_rx_handler, lorawan_has_joined_handler, lorawan_confirm_class_handler, lorawan_join_fail
                                       };

//OTAA keys !!!! KEYS ARE MSB !!!!
uint8_t nodeDeviceEUI[8] = {0xAC, 0x1F, 0x09, 0xFF, 0xFE, 0x05, 0x06, 0x9B};
uint8_t nodeAppEUI[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02};
uint8_t nodeAppKey[16] = {0x16, 0x5F, 0xB9, 0xEB, 0xB1, 0x26, 0xF1, 0x55, 0xD3, 0x1C, 0xB3, 0x35, 0x28, 0xAB, 0x36, 0xE3};

// ABP keys
uint32_t nodeDevAddr = 0x260116F8;
uint8_t nodeNwsKey[16] = {0x7E, 0xAC, 0xE2, 0x55, 0xB8, 0xA5, 0xE2, 0x69, 0x91, 0x51, 0x96, 0x06, 0x47, 0x56, 0x9D, 0x23};
uint8_t nodeAppsKey[16] = {0xFB, 0xAC, 0xB6, 0x47, 0xF3, 0x58, 0x45, 0xC7, 0x50, 0x7D, 0xBF, 0x16, 0x8B, 0xA8, 0xC1, 0x7C};

// Private defination
#define LORAWAN_APP_DATA_BUFF_SIZE 64										  /**< buffer size of the data to be transmitted. */
#define LORAWAN_APP_INTERVAL 60000*15									  /**< Defines for user timer, the application data transmission interval. value in [ms]. */
static uint8_t m_lora_app_data_buffer[LORAWAN_APP_DATA_BUFF_SIZE];			  //< Lora user application data buffer.
static lmh_app_data_t m_lora_app_data = {m_lora_app_data_buffer, 0, 0, 0, 0}; //< Lora user application data structure.

TimerEvent_t appTimer;
static uint32_t timers_init(void);
static uint32_t count = 0;
static uint32_t count_fail = 0;

void setup()
{
  // Initialize the built in LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Initialize Serial for debug output
  Serial.begin(115200);

  time_t serial_timeout = millis();
  // On nRF52840 the USB serial is not available immediately
  while (!Serial)
  {
    if ((millis() - serial_timeout) < 5000)
    {
      delay(100);
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
    else
    {
      break;
    }
  }

  Serial.println("=====================================");
  Serial.println("Welcome to RAK4630 LoRaWan!!!");
  if (doOTAA)
  {
    Serial.println("Type: OTAA");
  }
  else
  {
    Serial.println("Type: ABP");
  }

  switch (g_CurrentRegion)
  {
    case LORAMAC_REGION_AS923:
      Serial.println("Region: AS923");
      break;
    case LORAMAC_REGION_AU915:
      Serial.println("Region: AU915");
      break;
    case LORAMAC_REGION_CN470:
      Serial.println("Region: CN470");
      break;
  case LORAMAC_REGION_CN779:
    Serial.println("Region: CN779");
    break;
    case LORAMAC_REGION_EU433:
      Serial.println("Region: EU433");
      break;
    case LORAMAC_REGION_IN865:
      Serial.println("Region: IN865");
      break;
    case LORAMAC_REGION_EU868:
      Serial.println("Region: EU868");
      break;
    case LORAMAC_REGION_KR920:
      Serial.println("Region: KR920");
      break;
    case LORAMAC_REGION_US915:
      Serial.println("Region: US915");
      break;
  case LORAMAC_REGION_RU864:
    Serial.println("Region: RU864");
    break;
 /* case LORAMAC_REGION_AS923_2:
    Serial.println("Region: AS923-2");
    break;
  case LORAMAC_REGION_AS923_3:
    Serial.println("Region: AS923-3");
    break;
  case LORAMAC_REGION_AS923_4:
    Serial.println("Region: AS923-4");
    break;*/
  }
  Serial.println("=====================================");

// Set the analog reference to 3.0V (default = 3.6V)
    analogReference(AR_INTERNAL_3_0);

    // Set the resolution to 12-bit (0..4095)
    analogReadResolution(12); // Can be 8, 10, 12 or 14

    // Let the ADC settle
    delay(1);

    // Get a single ADC sample and throw it away
    readVBAT();


///////////////////////////////////////////////////////////////////////////////
  if (!ModbusRTUClient.begin(9600))
  {
    Serial.println("Failed to start Modbus RTU Client!");
    while (1)
      ;
  }
  //////////////////////////////////////////////////////////////////////////////
  // Initialize LoRa chip.
  lora_rak4630_init();

  /* bme680 init */
  init_bme680();


  //creat a user timer to send data to server period
  uint32_t err_code;

  err_code = timers_init();
  if (err_code != 0)
  {
    Serial.printf("timers_init failed - %d\n", err_code);
    return;
  }

  // Setup the EUIs and Keys
  if (doOTAA)
  {
    lmh_setDevEui(nodeDeviceEUI);
    lmh_setAppEui(nodeAppEUI);
    lmh_setAppKey(nodeAppKey);
  }
  else
  {
    lmh_setNwkSKey(nodeNwsKey);
    lmh_setAppSKey(nodeAppsKey);
    lmh_setDevAddr(nodeDevAddr);
  }

  // Initialize LoRaWan
  err_code = lmh_init(&g_lora_callbacks, g_lora_param_init, doOTAA, g_CurrentClass, g_CurrentRegion);
  if (err_code != 0)
  {
    Serial.printf("lmh_init failed - %d\n", err_code);
    return;
  }
  
  lmh_join();
}

void loop()
{
  // Put your application tasks here, like reading of sensors,
  // Controlling actuators and/or other functions. 
}


float readVBAT(void)
{
    float raw;

    // Get the raw 12-bit, 0..3000mV ADC value
    raw = analogRead(vbat_pin);

    return raw * REAL_VBAT_MV_PER_LSB;
}

/**
 * @brief Convert from raw mv to percentage
 * @param mvolts
 *    RAW Battery Voltage
 */
uint8_t mvToPercent(float mvolts)
{
    if (mvolts < 3300)
        return 0;

    if (mvolts < 3600)
    {
        mvolts -= 3300;
        return mvolts / 30;
    }

    mvolts -= 3600;
    return 10 + (mvolts * 0.15F); // thats mvolts /6.66666666
}

/**
 * @brief get LoRaWan Battery value
 * @param mvolts
 *    Raw Battery Voltage
 */
uint8_t mvToLoRaWanBattVal(float mvolts)
{
    if (mvolts < 3300)
        return 0;

    if (mvolts < 3600)
    {
        mvolts -= 3300;
        return mvolts / 30 * 2.55;
    }

    mvolts -= 3600;
    return (10 + (mvolts * 0.15F)) * 2.55;
}






/**@brief LoRa function for failed Join event
*/
void lorawan_join_fail(void)
{
  Serial.println("OTAA join failed!");
}

/**@brief LoRa function for handling HasJoined event.
*/
void lorawan_has_joined_handler(void)
{
  if(doOTAA == true)
  {
  Serial.println("OTAA Mode, Network Joined!");
  }
  else
  {
    Serial.println("ABP Mode");
  }

  lmh_error_status ret = lmh_class_request(g_CurrentClass);
  if (ret == LMH_SUCCESS)
  {
    delay(1000);
    TimerSetValue(&appTimer, LORAWAN_APP_INTERVAL);
    TimerStart(&appTimer);
  }
}

/**@brief Function for handling LoRaWan received data from Gateway
   @param[in] app_data  Pointer to rx data
*/
void lorawan_rx_handler(lmh_app_data_t *app_data)
{
  Serial.printf("LoRa Packet received on port %d, size:%d, rssi:%d, snr:%d, data:%s\n",
                app_data->port, app_data->buffsize, app_data->rssi, app_data->snr, app_data->buffer);
}

void lorawan_confirm_class_handler(DeviceClass_t Class)
{
  Serial.printf("switch to class %c done\n", "ABC"[Class]);
  // Informs the server that switch has occurred ASAP
  m_lora_app_data.buffsize = 0;
  m_lora_app_data.port = gAppPort;
  lmh_send(&m_lora_app_data, g_CurrentConfirm);
}

void send_lora_frame(void)
{
  if (lmh_join_status_get() != LMH_SET)
  {
    //Not joined, try again later
    
    return;
  }
  if (!bme.performReading()) {
  return;
  }

  bme680_get();

  lmh_error_status error = lmh_send(&m_lora_app_data, g_CurrentConfirm);
  if (error == LMH_SUCCESS)
  {
    count++;
    Serial.printf("lmh_send ok count %d\n", count);
  }
  else
  {
    count_fail++;
    Serial.printf("lmh_send fail count %d\n", count_fail);
  }
}

/**@brief Function for handling user timerout event.
*/
void tx_lora_periodic_handler(void)
{
  TimerSetValue(&appTimer, LORAWAN_APP_INTERVAL);
  TimerStart(&appTimer);
  Serial.println("Sending frame now...");
  send_lora_frame();
}

/**@brief Function for the Timer initialization.
   @details Initializes the timer module. This creates and starts application timers.
*/
uint32_t timers_init(void)
{
  TimerInit(&appTimer, tx_lora_periodic_handler);
  return 0;
}

void init_bme680(void)
{
  Wire.begin();

  if (!bme.begin(0x77)) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    return;
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_4X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms

  
}
///////////////////////////////////////////////////////////////////////////////////////
/*
static short read_reg(int reg_address)
{
  short reg_value;

  if (!ModbusRTUClient.requestFrom(18, HOLDING_REGISTERS, reg_address, 1))
  {
    Serial.print("failed to read registers! ");
    Serial.println(ModbusRTUClient.lastError());
  }
  else
  {
    reg_value = ModbusRTUClient.read();
  }
  return reg_value;
}

static short get_soil_conductivity(void)
{
  return read_reg(0x0002);
}

static short get_soil_temperature(void)
{
  return read_reg(0x0000);
}

static short get_soil_humidity(void)
{
  return read_reg(0x0001);
}*/

////////////////////////////////////////////////////////////////

//String data = "";

void bme680_get()
{
  uint32_t i = 0;

  memset(m_lora_app_data.buffer, 0, LORAWAN_APP_DATA_BUFF_SIZE);
  m_lora_app_data.port = gAppPort;

  // Get a raw ADC reading
    float vbat_mv = readVBAT();
    uint16_t bat = 0.10*vbat_mv + -330.00;



  //////////////////////////////////////////////////////////////////////
  //temporário short raw_conductivity;
  //temporárioshort raw_temperature;
  //temporárioshort raw_humidity;
  //uint16_t raw_conductivity;
  //uint16_t raw_temperature;
  //uint16_t raw_humidity;

  
  /* RS485 Power On */
  //temporáriopinMode(WB_IO2, OUTPUT);
  //temporáriodigitalWrite(WB_IO2, HIGH);
  //temporáriodelay(100);
  /* RS485 Power On */

  //temporárioraw_conductivity = get_soil_conductivity();
  //temporárioraw_temperature = get_soil_temperature();
  //temporárioraw_humidity = get_soil_humidity();

  /* RS485 Power Off */
  //temporáriopinMode(WB_IO2, OUTPUT);
  //temporáriodigitalWrite(WB_IO2, LOW);
 //temporáriodelay(100);
  /* RS485 Power Off */
  
  //temporárioSerial.printf("-----raw_conductivity = %d-------\n", raw_conductivity);
 //temporárioSerial.printf("-----raw_temperature = %d-------\n", raw_temperature / 100);
  //temporárioSerial.printf("-----raw_humidity = %d-------\n\n", raw_humidity / 100);
//////////////////////////////////////////////////////////////////////

  double temp = bme.temperature;
  double pres = bme.pressure / 100.0;
  double hum = bme.humidity;

  uint16_t t = temp * 100;
  uint16_t h = hum * 100;
  uint32_t pre = pres * 100;
  

delay(15000);


uint32_t gas = bme.gas_resistance;

  if(count>=20 || count_fail>=20){
    NVIC_SystemReset();
  }

 // Serial.print(bat);
 /* Serial.print("Temperature = ");
  Serial.print(bme.temperature);
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(bme.pressure / 100.0);
  Serial.println(" hPa");

  Serial.print("Humidity = ");
  Serial.print(bme.humidity);
  Serial.println(" %");

  Serial.print("Gas = ");
  Serial.print(bme.gas_resistance / 1000.0);
  Serial.println(" KOhms");

  Serial.println();*/



  //result: T=28.25C, RH=50.00%, P=958.57hPa, G=100406 Ohms
  m_lora_app_data.buffer[i++] = 0x01;
  m_lora_app_data.buffer[i++] = (uint8_t)(t >> 8);
  m_lora_app_data.buffer[i++] = (uint8_t)t;
  m_lora_app_data.buffer[i++] = 0x02;
  m_lora_app_data.buffer[i++] = (uint8_t)(h >> 8);
  m_lora_app_data.buffer[i++] = (uint8_t)h;
  m_lora_app_data.buffer[i++] = 0x03;
  m_lora_app_data.buffer[i++] = (uint8_t)((pre & 0xFF000000) >> 24);
  m_lora_app_data.buffer[i++] = (uint8_t)((pre & 0x00FF0000) >> 16);
  m_lora_app_data.buffer[i++] = (uint8_t)((pre & 0x0000FF00) >> 8);
  m_lora_app_data.buffer[i++] = (uint8_t)(pre & 0x000000FF);
  //m_lora_app_data.buffer[i++] = 0x04;
  m_lora_app_data.buffer[i++] = (uint8_t)((gas & 0xFF000000) >> 24);
  m_lora_app_data.buffer[i++] = (uint8_t)((gas & 0x00FF0000) >> 16);
  m_lora_app_data.buffer[i++] = (uint8_t)((gas & 0x0000FF00) >> 8);
  m_lora_app_data.buffer[i++] = (uint8_t)(gas & 0x000000FF);
  m_lora_app_data.buffer[i++] = 0xA0;
  m_lora_app_data.buffer[i++] = (uint8_t)(bat >> 8);
  m_lora_app_data.buffer[i++] = (uint8_t)bat;
///////////////////////////////////////////////////////////////////////////////
  //temporáriom_lora_app_data.buffer[i++] = 0x42;
  //temporáriom_lora_app_data.buffer[i++] = (raw_conductivity >> 8) & 0xFF;
  //temporáriom_lora_app_data.buffer[i++] = raw_conductivity & 0x00FF;
  //temporáriom_lora_app_data.buffer[i++] = 0x43;
 //temporário m_lora_app_data.buffer[i++] = (raw_temperature >> 8) & 0xFF;
  //temporáriom_lora_app_data.buffer[i++] = raw_temperature & 0x00FF;
 //temporário m_lora_app_data.buffer[i++] = 0x40;
  //temporáriom_lora_app_data.buffer[i++] = (raw_humidity >> 8) & 0xFF;
  //temporáriom_lora_app_data.buffer[i++] = raw_humidity & 0x00FF;
  /*m_lora_app_data.buffer[i++] = 0x42;
  m_lora_app_data.buffer[i++] = (uint8_t)(raw_conductivity >> 8);
  m_lora_app_data.buffer[i++] = (uint8_t)raw_conductivity;
  m_lora_app_data.buffer[i++] = 0x43;
  m_lora_app_data.buffer[i++] = (uint8_t)(raw_temperature >> 8);
  m_lora_app_data.buffer[i++] = (uint8_t)raw_temperature;
  m_lora_app_data.buffer[i++] = 0x40;
  m_lora_app_data.buffer[i++] = (uint8_t)(raw_humidity >> 8);
  m_lora_app_data.buffer[i++] = (uint8_t)raw_humidity;*/
/////////////////////////////////////////////////////////////////////////////// 
  m_lora_app_data.buffsize = i;

}
