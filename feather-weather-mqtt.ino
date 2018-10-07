
//
//
//

#include "./config.h"

//
//
//

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define DHTPIN   2
#define DHTTYPE  DHT22

DHT_Unified dht(DHTPIN, DHTTYPE);

uint32_t delayMS;

//
// wifi
//

#include <ESP8266WiFi.h>

WiFiClientSecure client ;

//
// mqtt
//

#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>

const char* AIO_SERVER = "io.adafruit.com" ;
const int AIO_SERVERPORT = 8883 ;

Adafruit_MQTT_Client mqtt( &client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY ) ;

Adafruit_MQTT_Publish mqtt_temperature = Adafruit_MQTT_Publish( &mqtt, AIO_USERNAME "/feeds/temperature" ) ;
Adafruit_MQTT_Publish mqtt_humidity = Adafruit_MQTT_Publish( &mqtt, AIO_USERNAME "/feeds/humidity" ) ;


//
// oled
//

#include <Adafruit_SSD1306.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SPITFT.h>
#include <Adafruit_SPITFT_Macros.h>
#include <gfxfont.h>

Adafruit_SSD1306 display = Adafruit_SSD1306();

//
// code
//

void setup() {
  Serial.begin(115200);
  delay(100);

  // oled
  Serial.println("OLED FeatherWing test");
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
  display.display();
  Serial.println("OLED begun");

  // wifi
  Serial.print("Connecting to WiFi " WIFI_SSID " ...");
  WiFi.begin( WIFI_SSID, WIFI_PASS );
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println( F(" Connected") ) ;
  Serial.print( F("IP address: ") ) ; Serial.println( WiFi.localIP() ) ;

  // clear display
  display.clearDisplay();
  display.display();

  //
  // temp sensor
  //

  // Initialize device.
  dht.begin();

  // Print temperature sensor details.
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.println("Temperature");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" *C");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" *C");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" *C");
  Serial.println("------------------------------------");

  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.println("Humidity");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println("%");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println("%");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println("%");
  Serial.println("------------------------------------");

  // Set delay between sensor readings based on sensor details.
  delayMS = sensor.min_delay / 1000;
}

void loop() {
  // Delay between measurements.
  delay(delayMS);

  Serial.println( F("loop() MQTT_connect()") ) ;
  MQTT_connect();

  // update the display
  Serial.println( F("loop() update display with IP") ) ;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.print("IP: "); display.println( WiFi.localIP() );

  //
  sensors_event_t event ;
  float celsius ;
  float humidity ;

  // temperature
  Serial.println( F("loop() calling dht.temperature()") ) ;
  dht.temperature().getEvent(&event);
  celsius = event.temperature ;

  if (isnan(celsius)) {
    Serial.println("Error reading temperature!");
    display.print("Temperature: NaN" );
  }
  else {
    display.print( F("Temperature: ") );
    display.print( celsius );
    display.println( F(" *C") );
    Serial.print( F("Temperature: ") );
    Serial.print(celsius);
    Serial.println( F(" *C") );

    Serial.print( F("loop() calling mqtt_temperature.publish() ... ") ) ;
    ( mqtt_temperature.publish( celsius ) )
      ? Serial.println( F("OK") )
      : Serial.println( F("Failed") )
      ;

  }

  // humidity
  Serial.println( F("loop() calling dht.humidity()") ) ;
  dht.humidity().getEvent( &event ) ;
  humidity = event.relative_humidity ;

  if (isnan(humidity)) {
    Serial.println("Error reading humidity!");
    display.print("Humidity: NaN" );
  }
  else {
    display.print("Humidity: ");
    display.print( humidity );
    display.println( F("%") ) ;
    Serial.print( F("Humidity: ") );
    Serial.print(humidity);
    Serial.println( F("%") );

    Serial.print( F("loop() calling mqtt_humidity.publish() ... ") ) ;
    ( mqtt_humidity.publish( humidity ) )
      ? Serial.println( F("OK") )
      : Serial.println( F("Failed") )
      ;
  }

  display.setCursor(0,0);
  display.display();

  delay( 10000 - delayMS ) ;
}


void MQTT_connect() {
  int8_t ret;

  if (mqtt.connected()) {
    return;
  }

  Serial.print( F("Connecting to MQTT ... ") ) ;

  uint8_t retries = 3;
  while ( (ret = mqtt.connect()) != 0 ) { // connect will return 0 for connected
       Serial.println( mqtt.connectErrorString(ret) ) ;
       Serial.print( F("Retrying MQTT connection in 5 seconds...") ) ;
       mqtt.disconnect() ;
       delay( 5000 ) ;
       retries-- ;
       if ( retries == 0 ) {
         // basically die and wait for WDT to reset me
         while (1);
       }
  }
  Serial.println( F("MQTT Connected!") );
}
