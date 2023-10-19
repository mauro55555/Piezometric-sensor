#include <FS.h> //this needs to be first, or it all crashes and burns...


#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <WiFiManager.h>          //https://github.com/tzapu/
#ifdef ESP32
#include <SPIFFS.h>
#endif

WiFiClient espClient;
PubSubClient client(espClient);
#include <EasyButton.h>


#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_BMP280.h>
Adafruit_BMP280 bmp; // I2C
WiFiManager wifiManager;

#include <Wire.h>
#include <Adafruit_ADS1X15.h>
Adafruit_ADS1115 ads;
#define DS18B20 D5 //DS18B20 esta conectado al pin GPIO D5 del NodeMCU

ADC_MODE(ADC_VCC);


// Arduino pin where the button is connected to.
#define BUTTON_PIN 0
EasyButton button(BUTTON_PIN);
// Callback function to be called when the button is pressed.
void onPressed() {
  Serial.println("RESETEO DE WIFI MANAGER");
  WiFiManager wifiManager;
  wifiManager.resetSettings();

  ESP.restart();
}


const char* mqtt_server = "piezometriaguasave.ddns.net"; // CAMBIAR PARA CUANDO SEA EXTERNO

#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson

// Update these with values suitable for your network.

char DEVICE[20] ;
char API[34];
char api_token[34];

//flag for saving data
bool shouldSaveConfig = false;

//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;

}


char msg[50];// VARIABLE DE MENSAJE PARA SERVIDOR

float temp_amb;
float presion_atm;
double Irms;



#define PRINT_INTERVAL 900000 // 15 MINUTOS ENTRE MEDICION

#include <SPI.h>

#include <Adafruit_GFX.h>

#include "EmonLib.h"  // Biblioteca para la familia de sensores SCT-013 
EnergyMonitor emon1;  // Creamos una instancia del sensor






float temp;// Variable para la temperatura leida
OneWire ourWire(DS18B20);// Se declara un objeto para la libreria
DallasTemperature sensor(&ourWire);// Se declara un objeto para la otra libreria
boolean first = false;
int16_t adc0;
float volts0;
float dataCurrent, depth, old_depth, head, cota; //unit:mA
unsigned long timepoint_measure;





void setup() {
  /* CODIGO SETUP DE mqtt_8266 de prueba*/
  Serial.begin(115200);
  // setup_wifi();
  client.setServer(mqtt_server, 1883);

  button.begin();
  // Add the callback function to be called when the button is pressed.
  button.onPressed(onPressed);

  while ( !Serial ) delay(100);   // wait for native usb
  Serial.println(F("BMP280 test"));
  ads.begin();
  unsigned status;
  status = bmp.begin(0x76);
  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  // put your setup code here, to run once:

  emon1.current(0, 60.60); // current (pin de entrada, constante de calibración)
  sensor.begin();
 
  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
  ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  //clean FS, for testing
  //SPIFFS.format();
  //wifiManager.resetSettings();
  //read configuration from FS json
  Serial.println("mounting FS...");






  if (SPIFFS.begin()) {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);

#ifdef ARDUINOJSON_VERSION_MAJOR >= 6
        DynamicJsonDocument json(1024);
        auto deserializeError = deserializeJson(json, buf.get());
        serializeJson(json, Serial);
        if ( ! deserializeError ) {
#else
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {
#endif
          Serial.println("\nparsed json");
          strcpy(DEVICE, json["DEVICE"]);
          strcpy(API, json["API"]);
          strcpy(api_token, json["api_token"]);
        } else {
          Serial.println("failed to load json config");
        }
        configFile.close();
      }
    }
  } else {
    Serial.println("failed to mount FS");
  }
  //end read

  WiFiManagerParameter custom_DEVICE("server", "NOMBRE_DISPOSITIVO", DEVICE, 20);
  WiFiManagerParameter custom_API("port", "Cota Topografica", API, 34);
  WiFiManagerParameter custom_api_token("apikey", "Profundidad sonda", api_token, 34);
  WiFiManager wifiManager;
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  //add all your parameters here
  wifiManager.addParameter(&custom_DEVICE);
  wifiManager.addParameter(&custom_API);
  wifiManager.addParameter(&custom_api_token);
//sets timeout until configuration portal gets turned off
  //useful to make it all retry or go to sleep
  //in seconds
  wifiManager.setConfigPortalTimeout(300);


  if (!wifiManager.autoConnect("Sensor IPN CIIDIR-OAX")) {
    Serial.println("failed to connect and hit timeout");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.restart();
    delay(5000);
  }

  strcpy(DEVICE, custom_DEVICE.getValue());
  strcpy(API, custom_API.getValue());
  strcpy(api_token, custom_api_token.getValue());
  Serial.println("The values in the file are: ");
  Serial.println("\tDISPOSITIVO : " + String(DEVICE));
  Serial.println("\tAPITEST : " + String(API));
  Serial.println("\tapi_token : " + String(api_token));

  //save the custom parameters to FS
  if (shouldSaveConfig) {
    Serial.println("saving config");
#ifdef ARDUINOJSON_VERSION_MAJOR >= 6
    DynamicJsonDocument json(1024);
#else
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
#endif
    json["DEVICE"] = DEVICE;
    json["API"] = API;
    json["api_token"] = api_token;

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      Serial.println("failed to open config file for writing");
    }

#ifdef ARDUINOJSON_VERSION_MAJOR >= 6
    serializeJson(json, Serial);
    serializeJson(json, configFile);
#else
    json.printTo(Serial);
    json.printTo(configFile);
#endif
    configFile.close();
    //end save
  }
  Serial.println("connected...yeey :)");

  cota = atof (API);
  head = atof (api_token);

  delay (1000);
  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");

  }

}


void loop() {


  // put your main code here, to run repeatedly:
  button.read();


  adc0 = ads.readADC_SingleEnded(0);
  volts0 = ads.computeVolts(adc0);
  dataCurrent = volts0 * 1000 / 120.0; //Sense Resistor:120ohm

if (millis()<15000){
  double Irms =emon1.calcIrms(1480);  // Calculate Irms only
  Irms=0;
}else {
  double Irms =emon1.calcIrms(1480);
}
  

  depth = ((dataCurrent - 4.0) * (5 / 1 / 16.0)) ; //Calculate depth from current readings
  if (depth < 0)
  {
    depth = 0.0;
  }
  depth = depth - head + cota;



  if (millis() - timepoint_measure > PRINT_INTERVAL || first == false || depth > (old_depth + 0.03) || depth < old_depth - 0.03) {
    timepoint_measure = millis();
    first = true;
    old_depth = depth;
    sensor.requestTemperatures();    // Le pide el valor de temperatura al sensor
    temp = sensor.getTempCByIndex(0);
    temp_amb = bmp.readTemperature();
    presion_atm = bmp.readPressure() / 100;
    if (!client.connected()) {
      reconnect();
    }




    snprintf (msg, 75, "%S/%.2f/%.2f/%.2f/%.2f/%.2f/FECHA", DEVICE, presion_atm, temp_amb, temp, depth, Irms);
    Serial.print("Publish message: "); Serial.println(msg);
    client.publish("mauro", msg);

  }
 
}





void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
    digitalWrite(BUILTIN_LED, LOW);   // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is acive low on the ESP-01)
  } else {
    digitalWrite(BUILTIN_LED, HIGH);  // Turn the LED off by making the voltage HIGH
  }

}
void resetWiFiManagerCredentials() {
  WiFiManager wifiManager;
  wifiManager.resetSettings();
  
  // Opcional: Reiniciar la tarjeta después de resetear las credenciales
  ESP.restart();
}
void reconnect() {
  // Loop until we're reconnected
  int count=0;
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    
    // Attempt to connect
    if (client.connect(DEVICE)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      count=0;

    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
      count=count+1;
      if (count>60){
        ESP.restart();
      }
    }
  }
}
