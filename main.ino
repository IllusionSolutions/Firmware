#include "MQTT.h"
#include "FlashStorage.h"
#include "math.h"

SYSTEM_MODE(AUTOMATIC);

//#define PC_BROKER[] { 178,62,44,11 } //Change Broker accordingly
#define PC_DEVICE_ID System.deviceID() //Declare this in setup
#define PC_ACCESS_KEY "" //Leave blank for now
#define PC_ACCESS_SECRET "" //Leave blank for now

// Declarations.
int sensor = D7; /*!< The pin on which the sensor is connected. */
int relay1 = D0; /*!< The pin on which the relay is connected. */
int analogPin = A0; /*!< The pin which is used for ADC input. */
int period = 0; /*!< Stores the desired delay between readings, in seconds. */
String realTimeMessage = "{}"; /*!< Stores the real-time message to be sent to the web application. */
bool relayOn = true; /*!< Stores the state of the relay (on or off). */
bool measuringOn = true; /*!< Stores a state indicating whether the device is measuring or not. */
float runtime = 0; /*!< Stores the how long the device has been running. */
float kWh = 0.000; /*!< Stores how many kiloWatt hours have been measured. */
float kW = 0.000; /*!< Stores how many kiloWatts have been measured. */
float I = 0.000; /*!< Stores how many amperes have been measured. */
float PF = 1; /*!< Stores the power factor of the system. By default it is 1. */
float V = 230; /*!< Stores the input voltage in the system. By default it is 230 V. */
float price = 0; /*!< Stores the calculated price. */
float kWh_price = 117.86; /*!< Stores the current price per kiloWatt hour. */
int analogValue = 0; /*!< Stores the value read from the analog pin. */
int sequenceNumber = 0; /*!< Stores the sequence number of each reading. */
time_t timeStarted; /*!< Stores the time when the device was started. */
time_t currentTime; /*!< Stores the current time at a point of a reading. */
int readingCounter = 1; /*!< Stores the number of readings taken. */
int currentCount = 0; /*!< Tracks the number of current measurements currently taken. */
double upperThreshold = 0; /*!< Stores the threshold current limit which will switch the device off. */
float sumI = 0; /*!< Stores the aggregated sum of the current values. */
int ADC_BITS = 12; /*!< Stores the number of bits available on the ADC. */
int ADC_COUNTS = (1<<ADC_BITS); /*!< Stores the number of counts used by the ADC. */
float offsetI = ADC_COUNTS>>1; /*!< Stores the offset used to calculate current. */
FlashStorage storage; /*!< An object of the FlashStorage class which is used to access EEPROM. */

volatile long pulseCount = 0; /*!< Stores the number of pulses measured. */

// Signature for the pulseInt function.
void pulseInt(void);

// Callback signature for MQTT subscriptions.
void callback(char* topic, byte* payload, unsigned int length);

// MQTT variables
String MQTT_TOPIC; /*!< Stores the topic used to identify messages in MQTT. */
String NOTIFY_TOPIC; /*!< Stores the topic used to identify notifications in MQTT. */
uint16_t qos2messageid = 0; /*!< Stores the QOS message for MQTT. */
uint16_t messageid; /*!< Stores the returned message ID from MQTT. */
int messageCounter = 0; /*!< Stores the number of messages sent during a send cycle. */

// MQTT client.
byte PC_BROKER[] { 178,62,44,11 }; /*!< Stores the IP address of the MQTT broker. */
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
void sendData(String s, String topic)
{
  bool sent = false;

  while (sent == false)
  {
    if (client.isConnected())
    {
      client.publish(topic, s, MQTT::QOS1, &messageid);
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

      client.publish(topic, s, MQTT::QOS1, &messageid);
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
    kW = kWh/runtime;
    price = (kWh * kWh_price)/100;

    /*
    Serial.print(kWh);
    Serial.print(" kWh ");
    Serial.print(I);
    Serial.print(" Amps ");
    Serial.print(price);
    Serial.println(" Rand ");
    */
  }
}

//! A method which is used to read values from the analog pin (A0), and converts these values into their corresponding current values.
/*!
*/
void getCurrent()
{
  analogValue = analogRead(analogPin);

  offsetI = (offsetI + (analogValue - offsetI) / 4096);
  float filteredI = analogValue - offsetI;

  float sqI = filteredI * filteredI;
  sumI += sqI;
  currentCount++;

  if (currentCount == 500)
  {
    double I_RATIO = 53.78 * (3.3/ADC_COUNTS);
    double Irms = I_RATIO * sqrt(sumI / currentCount);
    Irms = Irms + (Irms * 0.05);
    I = Irms;
    sumI = 0;
    currentCount = 0;
  }
}

//! A method used to send notifications to the application server.
/*!
  \param user A string containing the identification of the user causing a notification.
  \param status A string containing a description of the event which occured.
  \param timeOfEvent A time_t containing the epoch time when the event occured.
*/
void notify(String user, String status, time_t timeOfEvent)
{
  uint16_t messageid;
  String message = String("{\"user\":\"") + user + String("\",\"status\":\"") + String(status) + String("\",\"time\":") + String(timeOfEvent) + String("}");

  Serial.println(message);
  sendData(message, NOTIFY_TOPIC);
  delay(1500);
  client.disconnect();
}

//! An exposed method used to toggle the relay on or off via the cloud.
/*!
  \param input A string which contains the identity of the user toggling the relay.
  \return An integer representing the success state of the cloud function.
*/
int toggleRelay(String input)
{
  time_t toggleTime = Time.now();

  if(relayOn == true)
  {
    relayOn = false;
    digitalWrite(relay1, LOW);
    notify(input, "Device switched off.", toggleTime);
  }
  else
  {
    relayOn = true;
    digitalWrite(relay1, HIGH);
    notify(input, "Device switched on.", toggleTime);
  }
}

//! An exposed method used to toggle whether the device is taking measurements, via the cloud.
/*!
  \param input A string which contains the identity of the user toggling measurements.
  \return An integer representing the success state of the cloud function.
*/
int toggleMeasuring(String input)
{
  time_t toggleTime = Time.now();

  if(measuringOn == true)
  {
    measuringOn = false;
    notify(input, "Device is now unauthorized.", toggleTime);
  }
  else
  {
    measuringOn = true;
    timeStarted = Time.now();
    readingCounter = 1;
    notify(input, "Device is now authorized.", toggleTime);
  }
}

//! An exposed method which sets the upper current threshold via the cloud. Default and maximum is 8.00 A.
/*!
  \param input A string which contains the desired value of the threshold.
  \return An integer representing the success state of the cloud function.
*/
int setThreshold(String input)
{
  time_t timeCalled = Time.now();
  String username;
  double inputThreshold;
  int delimiter;

  delimiter = input.indexOf(",");

  if (delimiter == -1)
  {
    Serial.println("Invalid message received: " + input);
  }
  else
  {
    username = input.substring(0, delimiter);
    inputThreshold = (double) input.substring((delimiter + 1)).toFloat();

    if (inputThreshold > 8)
    {
      upperThreshold = 8.00;
      String status = "Upper threshold set to " + String(upperThreshold);
      notify(username, status, timeCalled);
    }
    else if (inputThreshold < 0.00)
    {
      upperThreshold = 0.00;
      String status = "Upper threshold set to " + String(upperThreshold);
      notify(username, status, timeCalled);
    }
    else
    {
      upperThreshold = inputThreshold;
      String status = "Upper threshold set to " + String(upperThreshold);
      notify(username, status, timeCalled);
    }
  }
}

//! An exposed method which sets the desired delay between readings in seconds. Default is 60 seconds.
/*!
  \param input A string which contains the desired value of the delay.
  \return An integer representing the success state of the cloud function.
*/
int setPeriod(String input)
{
  time_t timeCalled = Time.now();
  String username;
  int inputPeriod;
  int delimiter;

  delimiter = input.indexOf(",");

  if (delimiter == -1)
  {
    Serial.println("Invalid message received: " + input);
  }
  else
  {
    username = input.substring(0, delimiter);
    inputPeriod = (int) input.substring((delimiter + 1)).toInt();

    if (inputPeriod < 30)
    {
      period = 30;
      timeStarted = Time.now();
      readingCounter = 1;
      String status = "Period set to: " + String(period);
      notify(username, status, timeCalled);
    }
    else if (inputPeriod > 3600)
    {
      period = 3600;
      timeStarted = Time.now();
      readingCounter = 1;
      String status = "Period set to: " + String(period);
      notify(username, status, timeCalled);
    }
    else
    {
      period = inputPeriod;
      timeStarted = Time.now();
      readingCounter = 1;
      String status = "Period set to: " + String(period);
      notify(username, status, timeCalled);
    }
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
    setADCSampleTime(ADC_SampleTime_480Cycles);

    relayOn = true;
    Particle.function("relayToggle", toggleRelay);
    Particle.variable("relayStatus", relayOn);

    upperThreshold = 8.00;
    Particle.function("setThreshold", setThreshold);
    Particle.variable("threshold", upperThreshold);

    period = 60;
    Particle.function("setPeriod", setPeriod);
    Particle.variable("period", period);

    Particle.function("measureTog", toggleMeasuring);
    Particle.variable("measuring", measuringOn);

    Particle.variable("realTime", realTimeMessage);

    Time.zone(+2);

    while(!Serial.available())
    {
      Particle.process();
    }

    MQTT_TOPIC = "powercloud";
    NOTIFY_TOPIC = "powernotify";
    Serial.println("Welcome to the PowerCloud Particle device.");

    timeStarted = Time.now();
    delay(2000);
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

  getCurrent();

  if ((I > upperThreshold) && (relayOn == true))
  {
    toggleRelay("Device threshold");
  }

  if (measuringOn == true)
  {
    realTimeMessage = String("{\"current\":") + String(I) +
                      String(",\"power\":") + String(kWh) +
                      String(",\"time\":") + String(Time.now()) +
                      String("}");

    if ((Time.now() - timeStarted) == (period * readingCounter))
    {
      Serial.println("Getting a reading.");

      readingCounter++;

      reading.current = I;
      reading.voltage = V;
      reading.power = kWh;
      reading.timeRead = Time.now();
      reading.sequence = ++sequenceNumber;

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

            sendData(output, MQTT_TOPIC);
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

        sendData(output, MQTT_TOPIC);
        delay(5000);

        messageCounter = 0;
        client.disconnect();
        interrupts();
      }
    }
  }
}
