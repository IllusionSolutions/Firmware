#include "MQTT/MQTT.h"

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
    client.publish(MQTT_TOPIC, "Muriel says Hi!");
    delay(1000);
}
