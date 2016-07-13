#include "MQTT.h"

//#define PC_BROKER[] { 178,62,75,151 } //Change Broker accordingly
#define PC_DEVICE_ID System.deviceID() //Declare this in setup
#define PC_ACCESS_KEY "" //Leave blank for now
#define PC_ACCESS_SECRET "" //Leave blank for now

// The Photon's onboard LED.
int LED = D7;

String MQTT_TOPIC;

//address variable for storing EEPROM
int addr = 0;
bool stored;
/**
 * Docs for Readings
 **/
struct Readings {
  uint8_t version;
  float field1;
  float field2;
  float field3;
};

Readings data;

// Callback signature for MQTT subscriptions.
void callback(char* topic, byte* payload, unsigned int length);

bool storeVal(Readings data);

/**
 * Docs for storeVal()
 **/
void callback(char* topic, byte* payload, unsigned int length) {
    Serial.println("Command received:");
    Serial.println((char*)payload);
};
bool full = false;
/**
 * Docs for storeVal()
 **/
bool storeVal(Readings data)
{
    if(EEPROM.read(addr) == 255)
    {
        Serial.println("Attempting to store");
        EEPROM.put(addr, data);
        addr = addr + 50;
        return true;
    }
    else
    {
        Serial.println("Address full");
        full = true;
        return false;
    }
}

// MQTT client.
byte PC_BROKER[] { 178,62,75,151 };
MQTT client(PC_BROKER, 1883, callback);

/**
 * Docs for Connect()
 **/
void connect() {
    Serial.println();
    Serial.print("ID: ");
    Serial.println(PC_DEVICE_ID);
    Serial.println("\nConnecting to broker.");


    while(!client.isConnected())
    {
        //Connect to the broker
        client.connect( PC_DEVICE_ID );

        if(client.isConnected())
        {
            digitalWrite(LED, HIGH);
            Serial.println("connected!");
            client.subscribe("changeBroker");
        }
        else
        {
            Serial.print(".");
            delay(500);
        }
    }
}


/**
 * Docs for setup()
 **/
void setup() {
    Serial.begin(9600);
    while(!Serial.available()) Particle.process();
    MQTT_TOPIC = "powercloud";
    Serial.println("Setup process complete!");
    stored = false;
}

/**
 * Docs for loop()
 **/
int i = 1;
void loop()
{
    if (!client.isConnected())
    {
        connect();
    }

    if (!full)
    {
      Readings temp = {i, 24.12f + i, 13.0f + i, 330.12f + i};
      i++;
      storeVal(temp);
    }
    else
    {
      Readings container;
      EEPROM.get(addr, container);

      Serial.print("At address: ");
      Serial.print(addr);
      Serial.println();
      Serial.println(container.version);
      Serial.println(container.field1);
      Serial.println(container.field2);
      Serial.println(container.field3);
      Serial.println();
      //Serial.println("Hi " + (char) EEPROM.read(0));
    }

    client.loop();
    //client.publish(MQTT_TOPIC, "Muriel says Hi!");
    delay(1000);
}
