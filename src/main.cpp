#include <Arduino.h>
#include <ArduinoOTA.h> // https://github.com/esp8266/Arduino/blob/master/libraries/ArduinoOTA/ArduinoOTA.h
#include <AsyncMqttClient.h>
#include <Ticker.h>
#include <ESP8266WiFi.h>
#include "..\..\Credentials.h" // contains definitions of WIFI_SSID and WIFI_PASSWORD

#define HOSTNAME "HotBinMonitor"
const char *ssid = WIFI_SSID;
const char *password = WIFI_PASSWORD;

#define WIFI_CONNECT_TIMEOUT 15000
#define MQTT_HOST IPAddress(192, 168, 1, 210)
#define MQTT_PORT 1883
#define PUBLISH_FREQ_SECS 60

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

ArithmeticMean sensorValue;

int rawValue;

//--------------------------------------------------------------------
// Voltage divider details
const float R1 = 0;  // Fixed resistor next to thermistor. 0 if not required.
const float R2 = 6.8;  // Fixed resistor on other side of divider
const float Vcc = 3.3;  // Input voltage

// Get three measurements like below, then use
// http://www.thinksrs.com/downloads/programs/Therm%20Calc/NTCCalibrator/NTCcalculator.htm
//  100.0C = 6.7kohm
//   34.4C = 60.7kohm
// -13.0C = 624.0kohm
const float BETA = 3855;
const float NOM_RESIST = 75500;    // 90496;
const float NOM_TEMP = 25;

float resistanceToCelsius(float resistance) {
  // Simplified Steinhart-Hart
  float temp = log(resistance / NOM_RESIST) / BETA;
  temp += 1.0 / (NOM_TEMP + 273.15);
  temp = 1 / temp;
  temp -= 273.15; // convert to C
  return temp;
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
    String val = String( sensorValue.Average());
    mqttClient.publish("HotBinTemp/value", 0, true, val.c_str());
    
    val = String( rawValue );
    mqttClient.publish("HotBinTemp/raw", 0, true, val.c_str());
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

    connectToWifi();
}
//------------------------------------------------------------------------
void WiFiEvent(WiFiEvent_t event)
{
    switch (event)
    {
    case WIFI_EVENT_STAMODE_GOT_IP:
        connectToMqtt();
        break;
    case WIFI_EVENT_STAMODE_DISCONNECTED:
        mqttReconnectTimer.detach();
        wifiReconnectTimer.once(2, connectToWifi);
        break;
    default:
        break;
    }
}

void setup()
{
    Serial.begin(115200);
    delay(10);
    //Serial.setDebugOutput(true);
    Serial.println("\nSetup");

    WiFi.onEvent(WiFiEvent);

    init_OTA();
    init_mqtt();

    Serial.println("Setup Done.");
}

void loop()
{
    // read the sensor
    rawValue = analogRead(A0);
    
    Serial.print(rawValue);

    float voltage = rawValue / 1024.0;
    
    float totalResistance = ((Vcc * R2) / voltage) - R2;
    float resistance = (totalResistance - R1) * 1000;
    
    Serial.print(" temp = ");
    sensorValue.Update( resistanceToCelsius(resistance));
    Serial.println(sensorValue.Average());

    ArduinoOTA.handle();

    delay(5000);
}