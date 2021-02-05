#include <ETH.h>
#include <WiFiClientSecure.h>

#include <Wire.h>
#include <Temperature_LM75_Derived.h>

#define ETH_POWER_PIN   -1
#define ETH_ADDR        1

#define I2C_SDA 2
#define I2C_SCL 16

// I2C Expander bit
#define PCF_ADDR 0x3A
//#define PCF_ADDR 0x22
#define PCF_INTPIN 39

// define how many miliseconds should wait to debounce
#define PCF_INTDEBOUNCEMS 25
#define PCF_READTIMEOUT 500

#define DO1 4
#define DO2 5
#define DO3 6
#define DO4 7
#define DO_INVERT true

#define DI1 0
#define DI2 1
#define DI3 2
#define DI4 3
#define DI_INVERT false

#define TEMP_ADDR 0x49
#define TEMPTIME 60000

#define ADC1 35
#define ADC2 34

#define ADCTIME 60000
#define ADCCOUNT 100

bool lastSense1;
bool lastSense2;
bool lastSense3;
bool lastSense4;

Generic_LM75_11Bit temperature (&Wire,  TEMP_ADDR);

#define KEEPALIVE 5000
#define MQTT_KEEPALIVE 60000

// State flags
bool DEBUG = true;

static bool eth_connected = false;

void WiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case SYSTEM_EVENT_ETH_START:
      Serial.println("ETH Started");
      //set eth hostname here
      ETH.setHostname("esp32-ethernet");
      break;
    case SYSTEM_EVENT_ETH_CONNECTED:
      Serial.println("ETH Connected");
      break;
    case SYSTEM_EVENT_ETH_GOT_IP:
      Serial.print("ETH MAC: ");
      Serial.print(ETH.macAddress());
      Serial.print(", IPv4: ");
      Serial.print(ETH.localIP());
      if (ETH.fullDuplex()) {
        Serial.print(", FULL_DUPLEX");
      }
      Serial.print(", ");
      Serial.print(ETH.linkSpeed());
      Serial.println("Mbps");
      eth_connected = true;
      break;
    case SYSTEM_EVENT_ETH_DISCONNECTED:
      Serial.println("ETH Disconnected");
      eth_connected = false;
      break;
    case SYSTEM_EVENT_ETH_STOP:
      Serial.println("ETH Stopped");
      eth_connected = false;
      break;
    default:
      break;
  }
}

void i2c_pcfwrite(uint8_t addr, uint8_t data) {
  Wire.beginTransmission(addr);
  Wire.write(data);
  Wire.endTransmission();
}

uint8_t i2c_pcfread(uint8_t addr) {
  if (DEBUG) {
    Serial.print("Reading PCF... ");
  }

  Wire.requestFrom(addr, 1);

  int retry = 0;
  while (!Wire.available()) {
    retry++;

    if (retry > PCF_READTIMEOUT) {
      if (DEBUG) {
        Serial.print(" Timeout");
      }
      return -1;
    }
    delay(1);
  }

  if (DEBUG) {
    Serial.print(retry);
    Serial.println(" Done");
  }

  return Wire.read();
}

bool pcf_int_state = false;
void IRAM_ATTR pcf_int() {
  pcf_int_state = true;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println(F("Booting..."));

  Serial.print("Setting up I2C... ");
  pinMode(I2C_SDA, INPUT_PULLUP);
  pinMode(I2C_SCL, INPUT_PULLUP);
  pinMode(PCF_INTPIN, INPUT);

  Wire.begin(I2C_SDA, I2C_SCL, 100000);
  attachInterrupt(PCF_INTPIN, pcf_int, FALLING);
  Serial.println("Done");

  Serial.print("Init temp: ");
  Serial.print(temperature.readTemperatureC());
  Serial.println(" C");

  Serial.println("Input pins setup");
  di_setup();

  Serial.print(F("LAN connect... "));

  WiFi.onEvent(WiFiEvent);
  ETH.begin(ETH_ADDR, ETH_POWER_PIN, ETH_PHY_MDC, ETH_PHY_MDIO, ETH_PHY_TYPE, ETH_CLOCK_GPIO17_OUT);

  int retry = 0;
  while (!eth_connected) {
    Serial.println(F("Waiting"));
    delay(1000);
    retry++;
    if (retry > 10) {
      break;
    }
  }

  if (!eth_connected) {
    Serial.println("Connect LAN failed, skipping");
  }

  Serial.print("Init temp: ");
  Serial.print(temperature.readTemperatureC());
  Serial.println(" C");

  Serial.println(F("Ready"));
  Serial.print(F("IP address: "));
  Serial.println(ETH.localIP());
}

uint32_t sampledAdcRead(int8_t pin, uint16_t count) {
  uint32_t ad = 0;

  for (uint16_t i = 0; i < count; i++) {
    ad += analogRead(pin);
    delay(1);
  }

  return ad / count;
}

void di_setup() {
  uint8_t state = i2c_pcfread(PCF_ADDR);

  Serial.print("Sense state: ");
  Serial.println(state);

  lastSense1 = ((state >> DI1) & 0x01) ^ DI_INVERT;
  lastSense2 = ((state >> DI2) & 0x01) ^ DI_INVERT;
  lastSense3 = ((state >> DI3) & 0x01) ^ DI_INVERT;
  lastSense4 = ((state >> DI4) & 0x01) ^ DI_INVERT;

  Serial.println(lastSense1);
  Serial.println(lastSense2);
  Serial.println(lastSense3);
  Serial.println(lastSense4);
}


void do_write(int pin, bool state) {
  uint8_t oldstate = i2c_pcfread(PCF_ADDR);
  uint8_t newstate;

  newstate = oldstate;

  // Force log 1 to input pins

  newstate |= 0x01 << DI1;
  newstate |= 0x01 << DI2;
  newstate |= 0x01 << DI3;
  newstate |= 0x01 << DI4;

  if (DO_INVERT)
    state = !state;

  switch (pin) {
    case 1:
      if (state)
        newstate |=  0x01 << DO1;
      else
        newstate &= ~(0x01 << DO1);
      break;

    case 2:
      if (state)
        newstate |= 0x01 << DO2;
      else
        newstate &= ~(0x01 << DO2);
      break;

    case 3:
      if (state)
        newstate |= 0x01 << DO3;
      else
        newstate &= ~(0x01 << DO3);
      break;

    case 4:
      if (state)
        newstate |= 0x01 << DO4;
      else
        newstate &= ~(0x01 << DO4);
      break;
  }

  i2c_pcfwrite(PCF_ADDR, newstate);
}

bool lastDO1;
long previousKeepAliveMillis = 0;
void handleKeepAlive() {
  if (millis() - previousKeepAliveMillis >= KEEPALIVE) {
    previousKeepAliveMillis = millis();

    lastDO1 = !lastDO1;
    Serial.print("Write DO1 ");
    Serial.println(lastDO1);    
    do_write(1, lastDO1);

    Serial.print("Keep Alive - DEBUG ");
    if (DEBUG) {
      Serial.println("ON");
    }
    else {
      Serial.println("OFF");
    }
  }
}

bool digitalReadDebounce(int pin, int count, int treshold) {
  int c = 0;
  for (int i = 0; i < count; i++)
  {
    c += digitalRead(pin);
  }

  return c > treshold;
}

long pcf_int_laststate;
bool pcf_read_int;
void handleSense() {
  if (!pcf_read_int) {
    return;
  }
  if (millis() < pcf_int_laststate + PCF_INTDEBOUNCEMS) {
    Serial.println("Ignore sense - too early");
    return;
  }

  pcf_read_int = false;

  uint8_t state = i2c_pcfread(PCF_ADDR);

  bool s1 = ((state >> DI1) & 0x01) ^ DI_INVERT;
  bool s2 = ((state >> DI2) & 0x01) ^ DI_INVERT;
  bool s3 = ((state >> DI3) & 0x01) ^ DI_INVERT;
  bool s4 = ((state >> DI4) & 0x01) ^ DI_INVERT;

  if (DEBUG) {
    Serial.print("S1: ");
    Serial.print(s1);

    Serial.print("  S2: ");
    Serial.print(s2);

    Serial.print("  S3: ");
    Serial.print(s3);

    Serial.print("  S4: ");
    Serial.println(s4);
  }

  if ( s1 != lastSense1) {
    lastSense1 = s1;
  }

  if ( s2 != lastSense2) {
    lastSense2 = s2;
  }

  if ( s3 != lastSense3) {
    lastSense3 = s3;
  }

  if ( s4 != lastSense4) {
    lastSense4 = s4;
  }
}

long previousADCHandlerMillis = 0;
void handleADC() {
  if (millis() - previousADCHandlerMillis >= ADCTIME) {
    previousADCHandlerMillis = millis();

    int senseValue = sampledAdcRead(ADC1, ADCCOUNT);
    int senseValue2 = sampledAdcRead(ADC2, ADCCOUNT);

    Serial.print("ADC1: ");
    Serial.println(senseValue);
    Serial.print("ADC2: ");
    Serial.println(senseValue2);
  }
}

long previousTempHandlerMillis = 0;
void handleTemp() {
  if (millis() - previousTempHandlerMillis >= TEMPTIME) {
    if (DEBUG) {
      Serial.println("Reading temp...");
    }

    previousTempHandlerMillis = millis();
    float temp = temperature.readTemperatureC();

    if (DEBUG) {
      Serial.print("Temp: ");
      Serial.print(temp);
      Serial.println(" C");
    }
  }
}

void handle_pcf_interrupt() {
  if (!pcf_int_state) {
    return;
  }

  pcf_int_state = false;

  if (millis() < pcf_int_laststate + PCF_INTDEBOUNCEMS) {
    Serial.println("Ignore interrupt - too early");
    return;
  }

  Serial.println("Processing interrupt");

  pcf_read_int = true;
  pcf_int_laststate = millis();
}

// Main loop
void loop() {
  handleKeepAlive();

  handle_pcf_interrupt();

  handleSense();

  handleTemp();
}
