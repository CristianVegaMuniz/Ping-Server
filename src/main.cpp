#if defined(ARDUINO_ARCH_ESP8266)
#include <ESP8266WiFi.h>
#elif defined(ARDUINO_ARCH_ESP32)
#include <WiFi.h>
#endif
#include <esp8266ndn.h>
#include <DHT.h>
#include <Adafruit_Sensor.h>
#include <DHT_U.h>

#define LED D0        // Led in NodeMCU at pin GPIO16 (D0).
#define DHTTYPE DHT11 // Definimos el modelo DHT11
#define DHTPIN D2     // Se define el pin para conectar el sensor
DHT_Unified dht(DHTPIN, DHTTYPE);
float h, t;
uint32_t delayMS;

const char *WIFI_SSID = "MOVISTAR_100F";
const char *WIFI_PASS = "Z5wAz8MEbHCZ2453kmz2";

// const char *WIFI_SSID = "MiFibra-7430";
// const char *WIFI_PASS = "CUPsTPsg";

// const char *WIFI_SSID = "K30_1840";
// const char *WIFI_PASS = "AtWchC3y";

ndnph::StaticRegion<1024> region;
const char *PREFIX0 = "/temp";

std::array<uint8_t, esp8266ndn::UdpTransport::DefaultMtu> udpBuffer;

esp8266ndn::UdpTransport transport(udpBuffer);
ndnph::transport::ForceEndpointId transportw(transport);
ndnph::Face face(transportw);
ndnph::PingServer server(ndnph::Name::parse(region, PREFIX0), face);

void setup()
{
  Serial.begin(9600);
  Serial.println();
  esp8266ndn::setLogOutput(Serial);
  Serial.println(WiFi.localIP());
  pinMode(LED, OUTPUT); // LED pin as output.
  digitalWrite(LED, HIGH);

  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
  }
  delay(1000);

  Serial.println("My IP Address: ");
  Serial.println(WiFi.localIP());
  esp8266ndn::EthernetTransport::listNetifs(Serial);

  bool ok2 = transport.beginMulticast(WiFi.localIP());

  if (!ok2)
  {
    Serial.println(F("UDP multicast transport initialization failed"));
    ESP.restart();
  }

  /*  SENSOR DHT11 SETUP  */
  dht.begin();
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  delayMS = sensor.min_delay / 1000;
}

/*
void printData(ndnph::PingServer &server)
{
  try
  {
    Serial.println(" Recogiendo interest ...");
    ndnph::Interest data = server.get_Interest();
    Serial.println(" Recogiendo dataname ... ");
    ndnph::Name dataName = data.getName();
    Serial.printf("Interest received with dataname: ");
    Serial.println(dataName);
  }
  catch (const std::exception &e)
  {
    Serial.println("Todavia no hay paquete");
  }

  Serial.flush();
}
*/

// Funcion para recoger temperatura y humedad del sensor
void get_Temp()
{
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature))
  {
    Serial.println(F("Error reading temperature!"));
  }
  else
  {
    Serial.print(F("Temperature: "));
    Serial.print(event.temperature);
    Serial.println(F("°C"));
    server.set_Temperature(event.temperature);
  }

  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity))
  {
    Serial.println(F("Error reading humidity!"));
  }
  else
  {
    Serial.print(F("Humidity: "));
    Serial.print(event.relative_humidity);
    Serial.println(F("%"));
    server.set_Humidity(event.relative_humidity);
  }
}

void loop()
{
  face.loop();

  /*  SENSOR DHT11 LOOP  */
  delay(delayMS);
  get_Temp();

  /* LED */
  if (server.Led)
  {
    digitalWrite(LED, LOW);
    delay(500);
    digitalWrite(LED, HIGH);
    server.Led = 0;
  }
}
