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

}

bool FlashStorage::store(Reading data)
{
  return false;
}

bool FlashStorage::store(float current, float voltage,float power,time_t timeOfReading, int sequence)
{
  return false;
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
