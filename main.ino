#include "MQTT.h"
#include "FlashStorage.h"

SYSTEM_MODE(AUTOMATIC);

//#define PC_BROKER[] { 178,62,75,151 } //Change Broker accordingly
#define PC_DEVICE_ID System.deviceID() //Declare this in setup
#define PC_ACCESS_KEY "" //Leave blank for now
#define PC_ACCESS_SECRET "" //Leave blank for now

// Declarations.
int sensor = D7; /*!< The pin on which the sensor is connected. */
int relay1 = D0; /*!< The pin on which the relay is connected. */
bool relayOn = true; /*!< Stores the state of the relay (on or off). */
float runtime = 0; /*!< Stores the how long the device has been running. */
float kWh = 0.000; /*!< Stores how many kiloWatt hours have been measured. */
float kW = 0.000; /*!< Stores how many kiloWatts have been measured. */
float I = 0.000; /*!< Stores how many amperes have been measured. */
float PF = 1; /*!< Stores the power factor of the system. By default it is 1. */
float V = 230; /*!< Stores the input voltage in the system. By default it is 230 V. */
float price = 0; /*!< Stores the calculated price. */
float kWh_price = 117.86; /*!< Stores the current price per kiloWatt hour. */
time_t timeStarted; /*!< Stores the time when the device was started. */
time_t currentTime; /*!< Stores the current time at a point of a reading. */
int readingCounter = 1; /*!< Stores the number of readings taken. */
FlashStorage storage; /*!< An object of the FlashStorage class which is used to access EEPROM. */

volatile long pulseCount = 0; /*!< Stores the number of pulses measured. */

int i = 0;

// Signature for the pulseInt function.
void pulseInt(void);

// Callback signature for MQTT subscriptions.
void callback(char* topic, byte* payload, unsigned int length);

// MQTT variables
String MQTT_TOPIC; /*!< Stores the topic used to identify messages in MQTT. */
uint16_t qos2messageid = 0; /*!< Stores the QOS message for MQTT. */
uint16_t messageid; /*!< Stores the returned message ID from MQTT. */
int messageCounter = 0; /*!< Stores the number of messages sent during a send cycle. */

// MQTT client.
byte PC_BROKER[] { 178,62,75,151 }; /*!< Stores the IP address of the MQTT broker. */
MQTT client(PC_BROKER, 1883, callback);

//! A method which is called upon receiving a reply from the MQTT broker.
/*!
  \param topic A pointer to a char (array) value which contains the connection topic.
  \param payload A pointer to a byte (array) value which contains the message payload.
  \param length An integer which stores the length of the message.
*/
void callback(char* topic, byte* payload, unsigned int length) {
    Serial.println("Command received:");
    Serial.println((char*)payload);
};

// MQTT server sends ack message when using QOS1 or QOS2.
//! A method which is called whenever receiving a reply from the MQTT broker when using QOS 1 or 2.
/*!
  \param messageid An interger which stores the message ID.
*/
void qoscallback(unsigned int messageid)
{
  Serial.print("Ack Message ID: ");
  Serial.println(messageid);

  if (messageid == qos2messageid)
  {
    Serial.println("Release QoS2 Message");
    client.publishRelease(qos2messageid);
  }
}

/**
 * Docs for Connect()
 **/
//! A method which tries to connect to the MQTT broker, or keeps retrying until it succeeds.
/*!
*/
void connect() {
    while(!client.isConnected())
    {
        //Connect to the broker
        Serial.print("Attempting to Connect.");
        client.connect( PC_DEVICE_ID );
        if(client.isConnected())
        {
            Serial.println("Connected.");
        }
        else
        {
            Serial.print(".");
            delay(500);
        }
    }
}

//! A method which tries to connect to the MQTT broek, and will stop after 3 failed attempts.
/*!
*/
void attemptConnect()
{
  for (int y = 0; y < 3; y++)
  {
    Serial.println("Attempting to connect. Attempt: " + String((y + 1)) + ".");
    client.connect(PC_DEVICE_ID);

    if (client.isConnected())
    {
      Serial.println("Connected.");
      y = 5;
      delay(500);
    }
    else
    {
      delay(500);
    }
  }

  if (!client.isConnected())
  {
    Serial.println("Could not connect.");
  }
}

//! A method which sends data to the MQTT broker.
/*!
  \param s A string which stores the message to be sent to the MQTT broker.
*/
void sendData(String s)
{
  bool sent = false;

  while (sent == false)
  {
    if (client.isConnected())
    {
      client.publish(MQTT_TOPIC, s, MQTT::QOS1, &messageid);
      messageCounter = messageCounter + 1;
      Serial.println("YES. " + String(messageCounter) + " . " + String(messageid));
      sent = true;
    }
    else
    {
      while(!client.isConnected())
      {
        connect();
        client.addQosCallback(qoscallback);
        delay(5000);
        Particle.process();
      }

      messageCounter = 0;

      client.publish(MQTT_TOPIC, s, MQTT::QOS1, &messageid);
      messageCounter = messageCounter + 1;
      Serial.println("NO. " + String(messageCounter) + " . " + String(messageid));
      sent = true;
    }

    client.loop();
  }
}

//! A method returns the amount of time that the device has been running.
/*!
  \return Returns a float value representing how long the device has been running.
*/
float getTime()
{
  time_t currentTime;
  float hourStarted;
  float currentHour;

  currentTime = Time.now();

  hourStarted = (((((((float) Time.day(timeStarted)) * 24 * 60 * 60) +
  ((float) Time.hour(timeStarted)) * 60 * 60) +
   (((float) Time.minute(timeStarted)) * 60) +
    ((float) Time.second(timeStarted))) / 60 ) / 60);

    currentHour = (((((((float) Time.day(currentTime)) * 24 * 60 * 60) +
    ((float) Time.hour(currentTime)) * 60 * 60) +
     (((float) Time.minute(currentTime)) * 60) +
      ((float) Time.second(currentTime))) / 60 ) / 60);

    return (currentHour - hourStarted);
}

//! An interrupt service routine which calculates the values of readings from the device.
/*!
*/
void pulseInt()
{
  if(digitalRead(sensor) == HIGH)
  {
    runtime = getTime();
    pulseCount++;
    kWh = pulseCount * 0.001; // Wh. Watt hours.
    Serial.print(kWh);
    Serial.print(" kWh ");
    kW = kWh/runtime;
    I = (kW * 1000)/(PF * V);
    Serial.print(I);
    Serial.print(" Amps ");
    price = (kWh * kWh_price)/100;
    Serial.print(price);
    Serial.println(" Rand ");
  }
}

//! A method used to toggle the relay on and off.
/*!
  \param input A string which contains the identity of the user toggling the relay.
  \return An integer representing the success state of the cloud function.
*/
int toggleRelay(String input)
{
  if(relayOn == true)
  {
    relayOn = false;
    digitalWrite(relay1, LOW);
    Serial.println("Relay toggled off by " + input);
  }
  else
  {
    relayOn = true;
    digitalWrite(relay1, HIGH);
    Serial.println("Relay toggled on by " + input);
  }
}
/**
 * Docs for setup()
 **/
 //! A method which does the initial setup for the device.
 /*!
 */
void setup()
{
    Particle.connect();
    Serial.begin(9600);

    pinMode(relay1, OUTPUT);
    digitalWrite(relay1, HIGH);
    pinMode(sensor, INPUT_PULLDOWN);
    attachInterrupt(sensor, pulseInt, RISING);

    relayOn = true;
    Particle.function("relayToggle", toggleRelay);
    Particle.variable("relayStatus", relayOn);

    Time.zone(+2);

    while(!Serial.available())
    {
      Particle.process();
    }

    MQTT_TOPIC = "powercloud";
    Serial.println("Welcome to the PowerCloud Particle device.");

    timeStarted = Time.now();
    delay(5000);
    storage = FlashStorage();
    storage.clearMemory();
}

/**
 * Docs for loop()
 **/
 //! A method which continuously loops as the particle is running. It is responsible for controlling when a reading is made, storing it, and sending it to the MQTT broker.
void loop()
{
  FlashStorage::Reading reading;
  String output;
  uint16_t messageid;

  if ((Time.now() - timeStarted) == (60 * readingCounter))
  {
    Serial.println("Getting a reading.");

    readingCounter++;

    reading.current = I;
    reading.voltage = V;
    reading.power = kWh;
    reading.timeRead = Time.now();
    reading.sequence = ++i;

    attemptConnect();

    delay(2000);

    if (!client.isConnected())
    {
      Serial.println("Storing data.");
      storage.store(reading);
      client.loop();
      Particle.process();
    }
    else if (client.isConnected())
    {
      noInterrupts();
      Serial.println("Sending data.");
      if (storage.hasData() != -1)
      {
        FlashStorage::Reading retrieved = storage.dequeue();

        while(retrieved.sequence != -1)
        {
          output = String("{\"current\":") + String(retrieved.current) +
                   String(",\"voltage\":") + String(retrieved.voltage) +
                   String(",\"power\":") + String(retrieved.power) +
                   String(",\"time\":") + String(retrieved.timeRead) +
                   String("}");

          Serial.println(output);

          sendData(output);
          delay(5000);

          retrieved = storage.dequeue();

          client.loop();
          Particle.process();
        }
        storage.clearMemory();
      }
      output = String("{\"current\":") + String(reading.current) +
               String(",\"voltage\":") + String(reading.voltage) +
               String(",\"power\":") + String(reading.power) +
               String(",\"time\":") + String(reading.timeRead) +
               String("}");

      Serial.println(output);

      sendData(output);
      delay(5000);

      messageCounter = 0;
      client.disconnect();
      interrupts();
    }
  }
}
