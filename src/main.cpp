#include <Arduino.h>
#include <ArduinoOTA.h> // https://github.com/esp8266/Arduino/blob/master/libraries/ArduinoOTA/ArduinoOTA.h
#include <AsyncMqttClient.h>
#include <Ticker.h>
#include <ESP8266WiFi.h>
#include "..\..\Credentials.h" // contains definitions of WIFI_SSID and WIFI_PASSWORD

#define HOSTNAME "HotBinMonitor2"
const char *ssid = WIFI_SSID;
const char *password = WIFI_PASSWORD;

#define WIFI_CONNECT_TIMEOUT 15000
#define MQTT_HOST IPAddress(192, 168, 1, 210)
#define MQTT_PORT 1883
#define PUBLISH_FREQ_SECS 60*6

#define SENSOR_1_PIN    12
#define SENSOR_2_PIN    14

AsyncMqttClient mqttClient;
WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;
Ticker mqttReconnectTimer;
Ticker wifiReconnectTimer;
Ticker mqttPublishTimer;

//--------------------------------------------------------------------
class ArithmeticMean
{
private:
    const double ALPHA = 0.35;
    double _average = 0.0;
public:
    void Update(double val)
    {
        _average = ALPHA * val + (1.0 - ALPHA) * _average;
    }
    double Average() { return _average; }
};

ArithmeticMean sensorValue1;
ArithmeticMean sensorValue2;

int rawValue;

//--------------------------------------------------------------------

// Get three measurements like below, then use
// http://www.thinksrs.com/downloads/programs/Therm%20Calc/NTCCalibrator/NTCcalculator.htm
// 100K thermistor
//  100.0C = 6.7kohm
//   34.4C = 60.7kohm
// -13.0C = 624.0kohm
// 25.0C = 75500 ohm
// beta = 3855

// 10K Thermistor
// -14.5C = 63.0kohm
//   19.0C = 12.0kohm
//  100.0C = 0.7kohm
// 25.0C = 9250ohm
// beta = 3740
// const float BETA = 3840;
// const float NOM_RESIST = 8830;
// const float NOM_TEMP = 25;

const float Vcc = 3.3;  // Input voltage
const float SERIES_RESIST = 99800;  // 100K but measured as 99.8K
// const float BETA = 3950;
const float BETA = 3380;
const float NOMINAL_RESIST = 10000;
const float NOMINAL_TEMP = 25;
const float ZERO_OFFSET = 2.25;

//--------------------------------------------------------------------
float resistanceToCelsius(float resistance) {
  // Simplified Steinhart-Hart
  float temp = log(resistance / NOMINAL_RESIST) / BETA;
  temp += 1.0 / (NOMINAL_TEMP + 273.15);
  temp = 1.0 / temp;
  temp -= 273.15; // convert to C
  return temp + ZERO_OFFSET;
}

//--------------------------------------------------------------------
float ReadSensor1(uint8_t pin)
{
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);    // switch on the sensor
 
    // read the sensor
    rawValue = analogRead(A0);
    
    pinMode(pin, INPUT);    // switch off the sensor

    float voltage = rawValue / 1024.0;
    float resistance = (SERIES_RESIST * voltage) / (Vcc - voltage);

    Serial.print(pin);
    Serial.print(": Raw=");
    Serial.print(rawValue);
    Serial.print(" Resistance=");
    Serial.print(resistance);
    Serial.print(" Celsius=");
    Serial.println(resistanceToCelsius(resistance));

    return resistance;
}

//--------------------------------------------------------------------
float ReadSensor(uint8_t pin)
{
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);    // switch on the sensor
 
    // read the sensor
    rawValue = analogRead(A0);
    
    pinMode(pin, INPUT);    // switch off the sensor

    float voltage = (1023.0 / rawValue) - 1;
    float resistance = SERIES_RESIST / voltage;
    
    Serial.print(pin);
    Serial.print(": Raw=");
    Serial.print(rawValue);
    Serial.print(" Resistance=");
    Serial.print(resistance);
    Serial.print(" Celsius=");
    Serial.println(resistanceToCelsius(resistance));

    return resistance;
}

//--------------------------------------------------------------------
void init_OTA()
{
    Serial.println("init OTA");

    // ArduinoOTA callback functions
    ArduinoOTA.onStart([]() {
        Serial.println("OTA starting...");
    });

    ArduinoOTA.onEnd([]() {
        Serial.println("OTA done.Reboot...");
    });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        static unsigned int prevPcnt = 100;
        unsigned int pcnt = (progress / (total / 100));
        unsigned int roundPcnt = 5 * (int)(pcnt / 5);
        if (roundPcnt != prevPcnt)
        {
            prevPcnt = roundPcnt;
            Serial.println("OTA upload " + String(roundPcnt) + "%");
        }
    });

    ArduinoOTA.onError([](ota_error_t error) {
        Serial.print("OTA Error " + String(error) + ":");
        const char *line2 = "";
        switch (error)
        {
        case OTA_AUTH_ERROR:
            line2 = "Auth Failed";
            break;
        case OTA_BEGIN_ERROR:
            line2 = "Begin Failed";
            break;
        case OTA_CONNECT_ERROR:
            line2 = "Connect Failed";
            break;
        case OTA_RECEIVE_ERROR:
            line2 = "Receive Failed";
            break;
        case OTA_END_ERROR:
            line2 = "End Failed";
            break;
        }
        Serial.println(line2);
    });

    ArduinoOTA.setPort(8266);
    ArduinoOTA.setHostname(HOSTNAME);
    ArduinoOTA.setPassword(HOSTNAME);

    ArduinoOTA.begin();
}

//------------------------------------------------------------------------
void connectToWifi()
{
    Serial.println("Connecting to Wi-Fi...");
    WiFi.enableSTA(true);
    WiFi.hostname(HOSTNAME);
    WiFi.begin(ssid, password);
}

//------------------------------------------------------------------------
void connectToMqtt()
{
    Serial.println("Connecting to MQTT...");
    mqttClient.connect();
}

//------------------------------------------------------------------------
void publishSensorValue()
{
    String val = String( resistanceToCelsius(sensorValue1.Average()));
    mqttClient.publish("HotBinTemp/value1", 0, true, val.c_str());
    
    val = String( resistanceToCelsius(sensorValue2.Average()));
    mqttClient.publish("HotBinTemp/value2", 0, true, val.c_str());
}

//------------------------------------------------------------------------
void onMqttConnect(bool sessionPresent)
{
    Serial.println("Connected to MQTT.");

    // attach the publisher
    mqttPublishTimer.attach(PUBLISH_FREQ_SECS, publishSensorValue);
}

//------------------------------------------------------------------------
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason)
{
    Serial.println("Disconnected from MQTT.");

    mqttPublishTimer.detach(); // stop publishing

    if (WiFi.isConnected())
        mqttReconnectTimer.once(2, connectToMqtt);
}

//------------------------------------------------------------------------
void init_mqtt()
{
    mqttClient.onConnect(onMqttConnect);
    mqttClient.onDisconnect(onMqttDisconnect);
    mqttClient.setServer(MQTT_HOST, MQTT_PORT);
}

//--------------------------------------------------------------------
void setup()
{
    pinMode(SENSOR_1_PIN, INPUT);
    pinMode(SENSOR_2_PIN, INPUT);

    Serial.begin(115200);
    delay(10);
    //Serial.setDebugOutput(true);
    Serial.println("\nSetup");

    wifiConnectHandler = WiFi.onStationModeGotIP([](const WiFiEventStationModeGotIP &ev) 
    {
        Serial.println(WiFi.localIP());
        connectToMqtt();
    });

    wifiDisconnectHandler = WiFi.onStationModeDisconnected([](const WiFiEventStationModeDisconnected &ev) 
    {
        mqttReconnectTimer.detach();
        wifiReconnectTimer.once(2, connectToWifi);
    });

    init_mqtt();
    init_OTA();
    connectToWifi();

    Serial.println("Setup Done.");
}

//--------------------------------------------------------------------
void loop()
{
    sensorValue1.Update(ReadSensor1(SENSOR_1_PIN));
    sensorValue2.Update(ReadSensor1(SENSOR_2_PIN));

    ArduinoOTA.handle();
    delay(5000);
}