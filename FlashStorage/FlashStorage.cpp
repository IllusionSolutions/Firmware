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
    last_address = 0;
    front_address = 0;
}

bool FlashStorage::store(Reading data)
{
    if(!full())
    {
        Serial.println("Attempting to store");
        EEPROM.put(last_address, data);
        size_t size = sizeof(data);
        int size_int = static_cast<int>(size);
        last_address = last_address + size_int + 1;
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
        size_t size = sizeof(data);
        int size_int = static_cast<int>(size);
        last_address = last_address + size_int + 1;
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

  size_t size = sizeof(temp);
  int size_int = static_cast<int>(size);
  int previous_address;

  if(last_address != 0)
  {
    previous_address = last_address - 1 - size;
    EEPROM.get(previous_address, temp);
  }
  else
  {
      temp.sequence = -1;
  }

  return temp;
}

int FlashStorage::hasData()
{
  int length = (int) EEPROM.length() - 1;
  int start = 0;
  int address = -1;
  uint8_t value;

  while(start < length)
  {
    EEPROM.get(start, value);
    if (value != 0xFF)
    {
      address = start;
    }
    start = start + 1;
  }
  return address;
}

bool FlashStorage::full()
{
  size_t size = sizeof(Reading);
  int size_int = static_cast<int>(size);
  int length = (int) EEPROM.length() - 1;

  if ((length - size) > last_address)
  {
    return false;
  }
  return true;
}

FlashStorage::Reading FlashStorage::dequeue()
{
  Reading temp;

  size_t size = sizeof(temp);
  int size_int = static_cast<int>(size);

  if(last_address != front_address)
  {
    EEPROM.get(front_address, temp);
    front_address = front_address + size_int + 1;
  }
  else
  {
    temp.sequence = -1;
  }
  return temp;
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
