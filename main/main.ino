

/**
 * @file alerts.ino
 * @author Bryan Siepert for Adafruit Industries
 * @brief Show how to set adjust and use the sensor's included alert settings
 * @date 2020-11-10
 * 
 * @copyright Copyright (c) 2020
 * 
 */

/**
 * @file Getting_BPM_to_Monitor.ino
 * @author Bryan Siepert for Adafruit Industries
 * @brief Prints the BPM to the Serial Monitor, using the least lines of code and PulseSensor Library.
 * @date 2020-11-10
 * 
 * @copyright Copyright (c) 2020
 * 
 */

/**
 * @file main.ino
 * @author Abby Watterson
 * @brief Adapted from alerts.ino - Measuring temperature in dogs & Getting_BPM_to_Monitor.ino to get HRV of dogs
 * @date 2021-11-09
 * 
 */

#define USE_ARDUINO_INTERRUPTS true
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TMP117.h>
#include <Wire.h>
// Set-up low-level interrupts for most acurate BPM math.
#include <PulseSensorPlayground.h> // Includes the PulseSensorPlayground Library.

//  Variables
const int PulseWire = 0; // PulseSensor PURPLE WIRE connected to ANALOG PIN 0
const int LED13 = 13;    // The on-board Arduino LED, close to PIN 13.
int Threshold = 550;     // Determine which Signal to "count as a beat" and which to ignore.
                         // Use the "Gettting Started Project" to fine-tune Threshold Value beyond default setting.
                         // Otherwise leave the default "550" value.

PulseSensorPlayground pulseSensor; // Creates an instance of the PulseSensorPlayground object called "pulseSensor"

Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);

Adafruit_TMP117 tmp117;

void setup()
{
    // Set up for measuring Heart Rate Variability
    heartRateVariabilitySetup();

    // Set up for measuring Temperature
    temperatureSetup();
}

void loop()
{
    float heartRateVariability = measureHeartRateVariability();
    float temperature = measureTemperature();
}

// HEART RATE VARIABILITY FUNCTION
float measureHeartRateVariability()
{
    int hrv = pulseSensor.getInterBeatIntervalMs(); // Calls function on our pulseSensor object that returns IBI as an "int".
                                                    // "hrv" hold this IBI value now.

    if (pulseSensor.sawStartOfBeat())
    {                          // Constantly test to see if "a beat happened".
        Serial.print("HRV: "); // Print phrase "HRV: "
        Serial.println(hrv);   // Print the value inside of hrv.
    }
    delay(20);
    return hrv;
}

// HEART RATE VARIABILITY SETUP
void heartRateVariabilitySetup()
{
    Serial.begin(9600); // For Serial Monitor

    // Configure the PulseSensor object, by assigning our variables to it.
    pulseSensor.analogInput(PulseWire);
    pulseSensor.blinkOnPulse(LED13); //auto-magically blink Arduino's LED with heartbeat.
    pulseSensor.setThreshold(Threshold);

    // Double-check the "pulseSensor" object was created and "began" seeing a signal.
    if (pulseSensor.begin())
    {
        Serial.println("We created a pulseSensor Object !"); //This prints one time at Arduino power-up,  or on Arduino reset.
    }
}

// TEMPERATURE FUNCTION
float measureTemperature()
{
    display.clearDisplay();
    display.setCursor(0, 0);

    tmp117_alerts_t alerts;
    sensors_event_t temp;
    // Reading temp clears alerts, so read alerts first
    tmp117.getAlerts(&alerts); // get the status of any alerts
    tmp117.getEvent(&temp);    // get temperature

    Serial.print("Temperature: ");
    Serial.print(temp.temperature);
    Serial.println(" degrees C");

    Serial.print("High temperature alert active:");
    if (alerts.high)
    {
        Serial.println("True");
    }
    else
    {
        Serial.println("False");
    }

    Serial.print("Low temperature alert active:");
    if (alerts.low)
    {
        Serial.println("True");
    }
    else
    {
        Serial.println("False");
    }
    Serial.println("");

    //       Print to OLED
    display.print("Tmp:");
    display.print(temp.temperature, 1);
    display.println(" C");

    display.print("HI:");
    if (alerts.high)
    {
        display.print("Y");
    }
    else
    {
        display.print("N");
    }

    display.print(" LOW:");
    if (alerts.low)
    {
        display.println("Y");
    }
    else
    {
        display.println("N");
    }

    display.display();
    delay(300);
    return temp.temperature;
}

// TEMPERATURE SETUP
void temperatureSetup()
{
    Serial.begin(115200);
    while (!Serial)
        delay(10); // will pause Zero, Leonardo, etc until serial console opens
    Serial.println("Beginning TMP117 Temperature Recordings!");
    Serial.println("Waiting to find TMP117...");

    // Try to initialize!
    if (!tmp117.begin())
    {
        Serial.println("Failed to find TMP117 chip");
        while (1)
        {
            delay(10);
        }
    }
    Serial.println("TMP117 Found!");

    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
    { // Address 0x3C for 128x32
        Serial.println(F("SSD1306 allocation failed"));
        for (;;)
            ; // Don't proceed, loop forever
    }
    display.clearDisplay();
    display.display();
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setRotation(0);

    // Set the enable flag below to see how the low temp limit can be used as a
    // hysteresis value that defines the acceptable range for the temperature values where
    // the high temp alert is not active
    // tmp117.thermAlertModeEnabled(true);
    Serial.print("Therm mode enabled: ");
    if (tmp117.thermAlertModeEnabled())
    {
        Serial.println("True");
    }
    else
    {
        Serial.println("False");
    }

    // Threshold is set to that of a dog's temperature.
    tmp117.setHighThreshold(39.2);
    Serial.print("High threshold: ");
    Serial.println(tmp117.getHighThreshold(), 1);
    tmp117.setLowThreshold(38.3);
    Serial.print("Low threshold: ");
    Serial.println(tmp117.getLowThreshold(), 1);

    // tmp117.interruptsActiveLow(false);
    if (tmp117.interruptsActiveLow())
    {
        Serial.println("Alerts are active when the INT pin is LOW");
    }
    else
    {
        Serial.println("Alerts are active when the INT pin is HIGH");
    }

    Serial.println("");
    Serial.println("");
}