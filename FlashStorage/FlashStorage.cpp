#include "FlashStorage.h"

#if defined(ARDUINO)
#include "Arduino.h"
#elif defined(SPARK)
#include "application.h"
#endif

FlashStorage::FlashStorage()
{

}

FlashStorage::~FlashStorage()
{

}

void FlashStorage::clearMemory()
{
    EEPROM.clear();
}

bool FlashStorage::store(Reading data)
{
    if(!full())
    {
        Serial.println("Attempting to store");
        EEPROM.put(last_address, data);
        last_address = last_address + sizeof(data);
        return true;
    }
    else
    {
        Serial.println("Address full");
        return false;
    }
}

bool FlashStorage::store(float current, float voltage,float power,time_t timeOfReading, int sequence)
{
  Reading data;
  data.current=current;
  data.voltage=voltage;
  data.power=power;
  data.timeRead=timeOfReading;
  data.sequence=sequence;
  
    if(!full())
    {
        Serial.println("Attempting to store");
        EEPROM.put(last_address, data);
        last_address = last_address + sizeof(data);
        return true;
    }
    else
    {
        Serial.println("Address full");
        return false;
    }
}

FlashStorage::Reading FlashStorage::getLast()
{
  Reading temp;
  return temp;
}

int FlashStorage::hasData()
{
  return -1;
}

bool FlashStorage::full()
{
  return false;
}

/*bool storeVal(Readings data)
{
    if(EEPROM.read(addr) == 255)
    {
        Serial.println("Attempting to store");
        EEPROM.put(addr, data);
        int = sizeof(data);
        addr = addr + sizeof;
        return true;
    }
    else
    {
        Serial.println("Address full");
        full = true;
        return false;
    }
}
*/
