#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

#define WIFI_SSID "your wifi ssid"
#define WIFI_PASS "your wifi password"

#define MQTT_SERV "io.adafruit.com"
#define MQTT_PORT 1883
#define MQTT_NAME "<your adafruit username>"
#define MQTT_PASS "<your feed key>"

WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, MQTT_SERV, MQTT_PORT, MQTT_NAME, MQTT_PASS);

Adafruit_MQTT_Subscribe onoff = Adafruit_MQTT_Subscribe(&mqtt, MQTT_NAME "/f/diffuser");

void setup()
{
    Serial.begin(9600);

    // Connect to WiFi
    Serial.print("\n\nConnecting Wifi... ");
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
    }

    Serial.println("OK!");

    // Subscribe to the onoff topic
    mqtt.subscribe(&onoff);

    pinMode(4, OUTPUT);
}

void loop()
{
    // Connect/Reconnect to MQTT
    MQTT_connect();

    // Read from our subscription queue until we run out, or
    // wait up to 5 seconds for subscription to update
    Adafruit_MQTT_Subscribe *subscription;
    while ((subscription = mqtt.readSubscription(5000)))
    {
        // If we're in here, a subscription updated...
        if (subscription == &onoff)
        {
            // Print the new value to the serial monitor
            Serial.print("onoff: ");
            Serial.println((char *)onoff.lastread);

            // If the new value is  "ON", turn the light on.
            // Otherwise, turn it off.
            if (!strcmp((char *)onoff.lastread, "ON"))
            {
                // active low logic
                digitalWrite(4, HIGH);
                delay(100);
                digitalWrite(4, LOW);
            }
            else
            {
                for (int i = 0; i < 2; i++)
                {
                    digitalWrite(4, HIGH);
                    delay(100);
                    digitalWrite(4, LOW);
                    delay(500);
                }
            }
        }
    }

    // ping the server to keep the mqtt connection alive
    if (!mqtt.ping())
    {
        mqtt.disconnect();
    }
}

void MQTT_connect()
{
    int8_t ret;
    // Stop if already connected
    if (mqtt.connected())
    {
        return;
    }

    Serial.print("Connecting to MQTT... ");
    uint8_t retries = 3;
    while ((ret = mqtt.connect()) != 0) // connect will return 0 for connected
    {
        Serial.println(mqtt.connectErrorString(ret));
        Serial.println("Retrying MQTT connection in 5 seconds...");
        mqtt.disconnect();
        delay(5000); // wait 5 seconds
        retries--;
        if (retries == 0)
        {
            // basically die and wait for WDT to reset me
            while (1)
                ;
        }
    }
    Serial.println("MQTT Connected!");
}