#include "MQTT.h"
#include "FlashStorage.h"

//#define PC_BROKER[] { 178,62,75,151 } //Change Broker accordingly
#define PC_DEVICE_ID System.deviceID() //Declare this in setup
#define PC_ACCESS_KEY "" //Leave blank for now
#define PC_ACCESS_SECRET "" //Leave blank for now

// The Photon's onboard LED.
int LED = D7;

// MQTT variables
String MQTT_TOPIC;
uint16_t qos2messageid = 0;

// Callback signature for MQTT subscriptions.
void callback(char* topic, byte* payload, unsigned int length);

// MQTT client.
byte PC_BROKER[] { 178,62,75,151 };
MQTT client(PC_BROKER, 1883, callback);

void callback(char* topic, byte* payload, unsigned int length) {
    Serial.println("Command received:");
    Serial.println((char*)payload);
};

// MQTT server sends ack message when using QOS1 or QOS2.
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



FlashStorage storage;
/**
 * Docs for setup()
 **/
void setup() {
    Serial.begin(9600);
    while(!Serial.available()) Particle.process();
    MQTT_TOPIC = "powercloud";
    Serial.println("Hi, I'm Muriel, what's your name?");
    storage = FlashStorage();
}

int test_count = 0;

/**
 * Docs for loop()
 **/
void loop()
{
    if (!client.isConnected())
    {
        connect();
        client.addQosCallback(qoscallback);
    }

    if (test_count == 0 && client.isConnected())
    {
      uint16_t messageid;
      storage.clearMemory();

      FlashStorage::Reading data;

      for (int i = 0; i < 8; i++)
      {
          data.current = 10.0 + i;
          data.voltage = 10.0 + i;
          data.power = 10.0 + i;
          data.timeRead = Time.now();
          data.sequence = i;

          storage.store(data);

          delay(1000);
      }

      test_count = 1;

    }
    else if (test_count == 1 && client.isConnected())
    {
      FlashStorage::Reading retrieved = storage.dequeue();
      String output;
      uint16_t messageid;

      while (retrieved.sequence != -1)
      {
        output = String("{\"current\":") + String(retrieved.current) +
                 String(",\"voltage\":") + String(retrieved.voltage) +
                 String(",\"truePower\":") + String(retrieved.power) +
                 String(",\"time\":") + String(retrieved.timeRead) +
                 String("}");

        Serial.println(output);

        client.publish(MQTT_TOPIC, output, MQTT::QOS1, &messageid);

        delay(2000);

        retrieved = storage.dequeue();
      }

      test_count = 2;
    }

    client.loop();
    delay(1000);
}
