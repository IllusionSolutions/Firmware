#ifndef FLASHSTORAGE_H
#define FLASHSTORAGE_H

#if defined(SPARK) || (PLATFORM_ID==88)
#include "spark_wiring_string.h"
#include "spark_wiring_usbserial.h"
#elif defined(ARDUINO)
#include "Client.h"
#endif

/*! \brief FlashStorage, a class which provides methods that can reliably store and retrieve data
 *         from the EEPROM.
 */
class FlashStorage {

  private:
    int last_address; /*!< The last address which was used to store the readings. */
    size_t lengthOfMemory; /*!< The length of the entire EEPROM. */
    bool isFull; /*!< Checks if the EEPROM is full. */
    int front_address; /*!< The next address in the queue. */

  public:

    //! A structure intended to store the values read from SPI
    struct Reading {
      int sequence; /*!< The sequence of the current reading in memory */
      time_t timeRead; /*!< The time the reading was taken */
      float current; /*!< The current read */
      float voltage; /*!< The voltage read */
      float power; /*!< The power read */
    };

    //! The default constructor.
    /*!
      Constructor initializes the private variables in this class.
    */
    FlashStorage();

    //! The destructor.
    ~FlashStorage();

    //! A method which reliably clears the EEPROM.
    void clearMemory();

    //! A method which stores data to the EEPROM.
    /*!
      \param data Struct object which contains values read from SPI.
      \return Boolean, if storage was successful, true, else false.
    */
    bool store(Reading data);

    //! A method which stores data to the EEPROM.
    /*!
      \param current A float which has the current reading.
      \param voltage A float which has the voltage reading.
      \param power A float which has the power reading.
      \param timeOfReading A time_t which contains time in which values were read.
      \param sequence An int which indicates the sequence of the current reading.
      \return Boolean, if storage was successful, true, else false.
    */
    bool store(float current, float voltage,float power,time_t timeOfReading, int sequence);

    //! A method which returns the last object stored to EEPROM
    /*!
      \return Returns a struct of type Reading.
    */
    Reading getLast();

    //! A method which checks to see if the EEPROM is full.
    /*!
      \return A boolean with the result.
    */
    bool full();

    //! A method which checks to see if the EEPROM has data.
    /*!
      \return -1 if the EEPROM is empty, address as an int indicating the position of the last stored object.
    */
    int hasData();

    //! A method which returns the next object stored in EEPROM, from the beginning.
    /*!
        \return Returns a struct of type Reading.
    */
    Reading dequeue();
};

#endif
