/*
   Sketch to handle each sample read from a PulseSensor.
   Typically used when you don't want to use interrupts
   to read PulseSensor voltages.

   Here is a link to the tutorial that discusses this code
   https://pulsesensor.com/pages/getting-advanced

   Copyright World Famous Electronics LLC - see LICENSE
   Contributors:
     Joel Murphy, https://pulsesensor.com
     Yury Gitman, https://pulsesensor.com
     Bradford Needham, @bneedhamia, https://bluepapertech.com

   Licensed under the MIT License, a copy of which
   should have been included with this software.

   This software is not intended for medical use.
*/

/*
   Every Sketch that uses the PulseSensor Playground must
   define USE_ARDUINO_INTERRUPTS before including PulseSensorPlayground.h.
   Here, #define USE_ARDUINO_INTERRUPTS false tells the library to
   not use interrupts to read data from the PulseSensor.

   If you want to use interrupts, simply change the line below
   to read:
     #define USE_ARDUINO_INTERRUPTS true

   Set US_PS_INTERRUPTS to false if either
   1) Your Arduino platform's interrupts aren't yet supported
   by PulseSensor Playground, or
   2) You don't wish to use interrupts because of the side effects.

   NOTE: if US_PS_INTERRUPTS is false, your Sketch must
   call pulse.sawNewSample() at least once every 2 milliseconds
   to accurately read the PulseSensor signal.
*/
#define USE_ARDUINO_INTERRUPTS false
#include <PulseSensorPlayground.h>
#include <SPI.h>
#include <WiFi101.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TMP117.h>
#include <Wire.h>
#include "arduino_secrets.h"

/* Temperature */
Adafruit_TMP117 tmp117;

/*

   The format of our output.

   Set this to PROCESSING_VISUALIZER if you're going to run
    the Processing Visualizer Sketch.
    See https://github.com/WorldFamousElectronics/PulseSensor_Amped_Processing_Visualizer

   Set this to SERIAL_PLOTTER if you're going to run
    the Arduino IDE's Serial Plotter.
*/
const int OUTPUT_TYPE = SERIAL_PLOTTER;

/*
   Pinout:
     PULSE_INPUT = Analog Input. Connected to the pulse sensor
      purple (signal) wire.
     PULSE_BLINK = digital Output. Connected to an LED (and 220 ohm resistor)
      that will flash on each detected pulse.
     PULSE_FADE = digital Output. PWM pin onnected to an LED (and resistor)
      that will smoothly fade with each pulse.
      NOTE: PULSE_FADE must be a pin that supports PWM.
       If USE_INTERRUPTS is true, Do not use pin 9 or 10 for PULSE_FADE,
       because those pins' PWM interferes with the sample timer.
*/
const int PULSE_INPUT = A0;
const int PULSE_BLINK = 13; // Pin 13 is the on-board LED
const int PULSE_FADE = 5;
const int THRESHOLD = 550; // Adjust this number to avoid noise when idle

/*
   samplesUntilReport = the number of samples remaining to read
   until we want to report a sample over the serial connection.

   We want to report a sample value over the serial port
   only once every 20 milliseconds (10 samples) to avoid
   doing Serial output faster than the Arduino can send.
*/
byte samplesUntilReport;
const byte SAMPLES_PER_SERIAL_SAMPLE = 10;

/* All the PulseSensor Playground functions. */
PulseSensorPlayground pulseSensor;

/* Temperature */
float temperature;

/* HRV ANALYSIS */
const String participant = "Cooper";
String stressLevel = "Okay";
int movingAvgIBI[10];
int baselineIBI = 471;
int count = 0;

/* Dog Appeasing Pheromones */
bool DAP = false;

/* WIFI SETUP */

char ssid[] = SECRET_SSID;   // your network SSID (name)
char pass[] = SECRET_PASS;   // your network password (use for WPA)
int status = WL_IDLE_STATUS; // the WiFi radio's status

char server1[] = "stress-free-dogs.herokuapp.com";
char server2[] = "10.0.0.136";

// Initialize the Ethernet client library
// with the IP address and port of the server
// that you want to connect to (port 80 is default for HTTP):
WiFiClient client;

void setup()
{
   // Configure pins for Adafruit ATWINC1500 Feather
   WiFi.setPins(8, 7, 4, 2);

   /*
      Use 115200 baud because that's what the Processing Sketch expects to read,
      and because that speed provides about 11 bytes per millisecond.

      If we used a slower baud rate, we'd likely write bytes faster than
      they can be transmitted, which would mess up the timing
      of readSensor() calls, which would make the pulse measurement
      not work properly.
   */
   Serial.begin(115200);

   // Configure the PulseSensor manager.
   pulseSensor.analogInput(PULSE_INPUT);
   pulseSensor.blinkOnPulse(PULSE_BLINK);
   pulseSensor.fadeOnPulse(PULSE_FADE);

   pulseSensor.setSerial(Serial);
   pulseSensor.setOutputType(OUTPUT_TYPE);
   pulseSensor.setThreshold(THRESHOLD);

   // Skip the first SAMPLES_PER_SERIAL_SAMPLE in the loop().
   samplesUntilReport = SAMPLES_PER_SERIAL_SAMPLE;

   // Now that everything is ready, start reading the PulseSensor signal.
   if (!pulseSensor.begin())
   {
      /*
         PulseSensor initialization failed,
         likely because our Arduino platform interrupts
         aren't supported yet.

         If your Sketch hangs here, try changing USE_PS_INTERRUPT to false.
      */
      for (;;)
      {
         // Flash the led to show things didn't work.
         digitalWrite(PULSE_BLINK, LOW);
         delay(50);
         digitalWrite(PULSE_BLINK, HIGH);
         delay(50);
      }
   }

   // Intialize Temperature Sensor
   if (!tmp117.begin())
   {
      Serial.println("Failed to find TMP117 chip");
      while (1)
      {
         delay(10);
      }
   }
   Serial.println("TMP117 Found!");

   // check for the presence of the shield:
   if (WiFi.status() == WL_NO_SHIELD)
   {
      Serial.println("WiFi shield not present");
      // don't continue:
      while (true)
         ;
   }

   // Attempt to connect to WiFi network:
   while (status != WL_CONNECTED)
   {
      Serial.print("Attempting to connect to SSID: ");
      Serial.println(ssid);
      // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
      status = WiFi.begin(ssid, pass);

      // wait 10 seconds for connection:
      delay(10000);
   }
   Serial.println("Connected to wifi");
   printWiFiStatus();
}

void loop()
{
   // Temperature
   sensors_event_t temp;

   int BPM = pulseSensor.getBeatsPerMinute();
   int IBI = pulseSensor.getInterBeatIntervalMs();

   /*
      See if a sample is ready from the PulseSensor.

      If USE_INTERRUPTS is true, the PulseSensor Playground
      will automatically read and process samples from
      the PulseSensor.

      If USE_INTERRUPTS is false, this call to sawNewSample()
      will, if enough time has passed, read and process a
      sample (analog voltage) from the PulseSensor.
   */
   if (pulseSensor.sawNewSample())
   {
      /*
         Every so often, send the latest Sample.
         We don't print every sample, because our baud rate
         won't support that much I/O.
      */
      if (--samplesUntilReport == (byte)0)
      {
         samplesUntilReport = SAMPLES_PER_SERIAL_SAMPLE;

         /*
            At about the beginning of every heartbeat,
            report the heart rate and inter-beat-interval.
         */
         if (pulseSensor.sawStartOfBeat())
         {
            // Reading temp clears alerts, so read alerts first
            tmp117.getEvent(&temp); // get temperature

            Serial.print("Temperature: ");
            Serial.print(temp.temperature);
            Serial.println(" degrees C");

            BPM = pulseSensor.getBeatsPerMinute();
            IBI = pulseSensor.getInterBeatIntervalMs();

            Serial.print("Heart Rate: ");
            Serial.println(BPM);
            if (count == 10)
            {
               // Add moving average detection for IBI here to indicate stress
               int sumIBI = 0;
               for (int i = 0; i < 10; i++)
               {
                  sumIBI += movingAvgIBI[i];
               }
               int avgIBI = sumIBI / 10;
               Serial.println(avgIBI);
               bool stressed = avgIBI < baselineIBI && temp.temperature < 35 && temp.temperature > 28;
               if (stressed)
               {
                  stressLevel = "Stressed";
                  DAP = true;
                  togglePheromonesOnOff();
                  Serial.println("Pheromone Diffuser On");
                  Serial.println("STATUS: Stress detected");
               }
               else if (!stressed)
               {
                  stressLevel = "Okay";
                  // Only toggle the pheromones off if the diffuser was previously on
                  if (DAP)
                  {
                     togglePheromonesOnOff();
                     Serial.println("Pheromone Diffuser Off");
                  }
                  DAP = false;
                  Serial.println("STATUS: Okay");
               }
               count = -1;
            }

            movingAvgIBI[count] = IBI;
            ++count;
            sendMeasurements(BPM, IBI, temp.temperature, stressLevel, DAP);
         }
      }

      /*******
        Here is a good place to add code that could take up
        to a millisecond or so to run.
      *******/
   }

   /******
      Don't add code here, because it could slow the sampling
      from the PulseSensor.
   ******/
}

/**************************************************************************/
/*                          Print Wifi Status                             */
/**************************************************************************/
void printWiFiStatus()
{
   // print the SSID of the network you're attached to:
   Serial.print("SSID: ");
   Serial.println(WiFi.SSID());

   // print your WiFi shield's IP address:
   IPAddress ip = WiFi.localIP();
   Serial.print("IP Address: ");
   Serial.println(ip);

   // print the received signal strength:
   long rssi = WiFi.RSSI();
   Serial.print("signal strength (RSSI):");
   Serial.print(rssi);
   Serial.println(" dBm");
}

/**************************************************************************/
/*                 Send Measurements to the Server                        */
/**************************************************************************/
void sendMeasurements(int heartRate, int ibi, float temperature, String stressLevel, bool dap)
{
   // Create query string
   String query = "participant=" + participant + "&heartRate=" + heartRate + "&ibi=" + ibi + "&temperature=" + temperature + "&stressLevel=" + stressLevel + "&dap=" + dap;

   Serial.println("\nStarting connection to Server 1...");
   // if you get a connection, report back via serial:
   if (client.connect(server1, 80))
   {
      Serial.println("Connected to Server 1");
      // Make the HTTP GET request:
      client.println("GET /add-data?" + query + " HTTP/1.1");
      client.println("Host: stress-free-dogs.herokuapp.com");
      client.println("Connection: close");
      client.println();
   }

   // if the server's disconnected, stop the client:
   if (!client.connected())
   {
      Serial.println();
      Serial.println("Disconnecting from Server 1.");
      client.stop();
   }
}

/**************************************************************************/
/*                Turn Diffuser On & Off w/ Pheromones                    */
/**************************************************************************/
void togglePheromonesOnOff()
{

   Serial.println("\nStarting connection to Server 2...");
   // if you get a connection, report back via serial:
   if (client.connect(server2, 80))
   {
      Serial.println("Connected to Server 2");
      // Make the HTTP GET request:
      client.println("GET /T  HTTP/1.1");
      client.println("Host: 10.0.0.136");
      client.println("Connection: close");
      client.println();
   }

   // if the server's disconnected, stop the client:
   if (!client.connected())
   {
      Serial.println();
      Serial.println("Disconnecting from Server 2.");
      client.stop();
   }
}
