#include "MQTT.h"
#include "FlashStorage.h"

//#define PC_BROKER[] { 178,62,75,151 } //Change Broker accordingly
#define PC_DEVICE_ID System.deviceID() //Declare this in setup
#define PC_ACCESS_KEY "" //Leave blank for now
#define PC_ACCESS_SECRET "" //Leave blank for now

// The Photon's onboard LED.
int LED = D7;
String MQTT_TOPIC;

// Callback signature for MQTT subscriptions.
void callback(char* topic, byte* payload, unsigned int length);

void callback(char* topic, byte* payload, unsigned int length) {
    Serial.println("Command received:");
    Serial.println((char*)payload);
};

// MQTT client.
byte PC_BROKER[] { 178,62,75,151 };
MQTT client(PC_BROKER, 1883, callback);

//

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

/**
 * Docs for loop()
 **/
void loop()
{
    if (!client.isConnected())
    {
        connect();
    }

    client.loop();
    //client.publish(MQTT_TOPIC, "Muriel says Hi!");
    delay(1000);
}
