// Gas monitoring device / Code from EquinoxeFR
// RFXmeter code from pyrou https://github.com/pyrou/x10rf
// OOK oregon encoding from http://connectingstuff.net/blog/encodage-protocoles-oregon-scientific-sur-arduino/


#include <x10rf.h>
#include <Energia.h>
#include <avr/eeprom.h>
#include <dht.h>


#define SCHEMA 69 // Change value to initialize or reset the EEPROM
//#define DEBUG 1 // serial debug
#define DHT22_PIN 5 // Temp/humidity sensor pin
#define txPin 7 // RF 433MHz module
#define txRetry 5 // delay for retry when sending packets
#define transmitLedPin 13 
#define reedLedPin 12
#define reedInterrupt 0 // Reed switch pin
#define delayPulse 5 // wait between two pulses. Avoid false reading with slow rotation


#define SEND_HIGH() digitalWrite(txPin, HIGH)
#define SEND_LOW() digitalWrite(txPin, LOW)
const unsigned long TIME = 512;
const unsigned long TWOTIME = TIME*2;
byte OregonMessageBuffer[9];
dht DHT;
unsigned long pulses = 0;
unsigned long previousPulses = 0;
int timer1_counter;
volatile unsigned long timerEeprom = 0;
volatile unsigned long timerSend = 0;
volatile unsigned long timerDebounce = 0;
volatile unsigned long lastDebounce = 0;

x10rf myx10 = x10rf(txPin,transmitLedPin,txRetry);

void setup()
{
  byte status=0;
  myx10.begin();
  
#ifdef DEBUG
  Serial.begin(9600);
#endif

  // Timer1 5Hz
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  timer1_counter = 53036;   // preload timer 65536-16MHz/256/5Hz
  TCNT1 = timer1_counter;   // preload timer
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  interrupts();             // enable all interrupts

    // EEProm init or read
  eeprom_read_block((void*)&status, (void*)0, sizeof(status));
  if (status != SCHEMA)
  {
#ifdef DEBUG
    Serial.println("New EEPROM detected. Initializing...");
#endif
    status=SCHEMA;
    // writing SCHEMA version
    eeprom_write_block((const void*)&status, (void*)0, sizeof(status));
    // writing initial pulse count (0)
    eeprom_write_block((const void*)&pulses, (void*)1, sizeof(pulses));
  }
  else
  {
    // reading pulses from EEprom
    eeprom_read_block((void*)&pulses, (void*)1, sizeof(pulses));
  }
  previousPulses=pulses;
#ifdef DEBUG
  Serial.print("Reading data from eeprom: ");
  Serial.println(pulses);
#endif
  // Setup OOK packets for Oregon sensors
  byte ID[] = {
    0x1A,0x2D  };
  setType(OregonMessageBuffer, ID);
  setChannel(OregonMessageBuffer, 0x20);
  setId(OregonMessageBuffer, 0xBB);

  // Setup pins
  pinMode(transmitLedPin,OUTPUT);
  pinMode(reedLedPin,OUTPUT);
  pinMode(reedInterrupt,INPUT);

  // Interrupt for reed switch
  attachInterrupt(reedInterrupt, debounceInterrupt, RISING);
}

// Main loop
void loop()
{

  // Saving pulses to EEprom every hour (100 000 write cycles allowed)  
  if (timerEeprom >= 3600*5)
  {
    timerEeprom=0;
    if (pulses != previousPulses)
    {
#ifdef DEBUG
      Serial.println("Saving data to eeprom");
#endif
      eeprom_write_block((const void*)&pulses, (void*)1, sizeof(pulses));
    }

  }

  // Sending RF packets every 30s  
  if (timerSend >= 30*5)
  {
    timerSend=0;
    myx10.RFXmeter(12,0,pulses);
    int chk = DHT.read22(DHT22_PIN);
#ifdef DEBUG
    Serial.print("Temp: ");
    Serial.println(DHT.temperature, 1);
    Serial.print("Humidity: ");
    Serial.println(DHT.humidity, 1);
#endif
    setBatteryLevel(OregonMessageBuffer, 1);
    setTemperature(OregonMessageBuffer, DHT.temperature);
    setHumidity(OregonMessageBuffer, DHT.humidity); 
    calculateAndSetChecksum(OregonMessageBuffer);
    sendOregon(OregonMessageBuffer, sizeof(OregonMessageBuffer));
    SEND_LOW();
    delayMicroseconds(TWOTIME*8);
    sendOregon(OregonMessageBuffer, sizeof(OregonMessageBuffer));
    SEND_LOW();

  }
}


// Timer1 interrupt. Managing timers
ISR(TIMER1_OVF_vect)        // interrupt service routine 
{
  TCNT1 = timer1_counter;   // preload timer
  timerEeprom++;
  timerSend++;
  timerDebounce++;
}


// Reed switch interrupt with debounce
void debounceInterrupt() {
#ifdef DEBUG
  Serial.println("Int !!!");
  Serial.println(timerDebounce);
  Serial.println(lastDebounce);
#endif
  if ((timerDebounce - lastDebounce) >= delayPulse * 5) 
  {                                                  
    lastDebounce = 0;
    timerDebounce = 0;
    Interrupt();
  }

}

// Reed switch
void Interrupt() {
#ifdef DEBUG
  Serial.println("New pulse !");
#endif
  pulses++;
  digitalWrite(reedLedPin,HIGH);
  delay(800);
  digitalWrite(reedLedPin,LOW);
}

// Other code is from http://connectingstuff.net/blog/encodage-protocoles-oregon-scientific-sur-arduino/
// Copy and paste :)


/**
 * \brief    Send logical "0" over RF
 * \details  azero bit be represented by an off-to-on transition
 * \         of the RF signal at the middle of a clock period.
 * \         Remenber, the Oregon v2.1 protocol add an inverted bit first
 */
inline void sendZero(void)
{
  SEND_HIGH();
  delayMicroseconds(TIME);
  SEND_LOW();
  delayMicroseconds(TWOTIME);
  SEND_HIGH();
  delayMicroseconds(TIME);
}

/**
 * \brief    Send logical "1" over RF
 * \details  a one bit be represented by an on-to-off transition
 * \         of the RF signal at the middle of a clock period.
 * \         Remenber, the Oregon v2.1 protocol add an inverted bit first
 */
inline void sendOne(void)
{
  SEND_LOW();
  delayMicroseconds(TIME);
  SEND_HIGH();
  delayMicroseconds(TWOTIME);
  SEND_LOW();
  delayMicroseconds(TIME);
}

/**
 * Send a bits quarter (4 bits = MSB from 8 bits value) over RF
 *
 * @param data Source data to process and sent
 */

/**
 * \brief    Send a bits quarter (4 bits = MSB from 8 bits value) over RF
 * \param    data   Data to send
 */
inline void sendQuarterMSB(const byte data)
{
  (bitRead(data, 4)) ? sendOne() : sendZero();
  (bitRead(data, 5)) ? sendOne() : sendZero();
  (bitRead(data, 6)) ? sendOne() : sendZero();
  (bitRead(data, 7)) ? sendOne() : sendZero();
}

/**
 * \brief    Send a bits quarter (4 bits = LSB from 8 bits value) over RF
 * \param    data   Data to send
 */
inline void sendQuarterLSB(const byte data)
{
  (bitRead(data, 0)) ? sendOne() : sendZero();
  (bitRead(data, 1)) ? sendOne() : sendZero();
  (bitRead(data, 2)) ? sendOne() : sendZero();
  (bitRead(data, 3)) ? sendOne() : sendZero();
}

/******************************************************************/
/******************************************************************/
/******************************************************************/

/**
 * \brief    Send a buffer over RF
 * \param    data   Data to send
 * \param    size   size of data to send
 */
void sendData(byte *data, byte size)
{
  for(byte i = 0; i < size; ++i)
  {
    sendQuarterLSB(data[i]);
    sendQuarterMSB(data[i]);
  }
}

/**
 * \brief    Send an Oregon message
 * \param    data   The Oregon message
 */
void sendOregon(byte *data, byte size)
{
  sendPreamble();
  //sendSync();
  sendData(data, size);
  sendPostamble();
}

/**
 * \brief    Send preamble
 * \details  The preamble consists of 16 "1" bits
 */
inline void sendPreamble(void)
{
  byte PREAMBLE[]={
    0xFF,0xFF  };
  sendData(PREAMBLE, 2);
}

/**
 * \brief    Send postamble
 * \details  The postamble consists of 8 "0" bits
 */
inline void sendPostamble(void)
{
#ifdef THN132N
  sendQuarterLSB(0x00);
#else
  byte POSTAMBLE[]={
    0x00  };
  sendData(POSTAMBLE, 1); 
#endif
}

/**
 * \brief    Send sync nibble
 * \details  The sync is 0xA. It is not use in this version since the sync nibble
 * \         is include in the Oregon message to send.
 */
inline void sendSync(void)
{
  sendQuarterLSB(0xA);
}

/******************************************************************/
/******************************************************************/
/******************************************************************/

/**
 * \brief    Set the sensor type
 * \param    data       Oregon message
 * \param    type       Sensor type
 */
inline void setType(byte *data, byte* type)
{
  data[0] = type[0];
  data[1] = type[1];
}

/**
 * \brief    Set the sensor channel
 * \param    data       Oregon message
 * \param    channel    Sensor channel (0x10, 0x20, 0x30)
 */
inline void setChannel(byte *data, byte channel)
{
  data[2] = channel;
}

/**
 * \brief    Set the sensor ID
 * \param    data       Oregon message
 * \param    ID         Sensor unique ID
 */
inline void setId(byte *data, byte ID)
{
  data[3] = ID;
}

/**
 * \brief    Set the sensor battery level
 * \param    data       Oregon message
 * \param    level      Battery level (0 = low, 1 = high)
 */
void setBatteryLevel(byte *data, byte level)
{
  if(!level) data[4] = 0x0C;
  else data[4] = 0x00;
}

/**
 * \brief    Set the sensor temperature
 * \param    data       Oregon message
 * \param    temp       the temperature
 */
void setTemperature(byte *data, float temp)
{
  // Set temperature sign
  if(temp < 0)
  {
    data[6] = 0x08;
    temp *= -1; 
  }
  else
  {
    data[6] = 0x00;
  }

  // Determine decimal and float part
  int tempInt = (int)temp;
  int td = (int)(tempInt / 10);
  int tf = (int)round((float)((float)tempInt/10 - (float)td) * 10);

  int tempFloat =  (int)round((float)(temp - (float)tempInt) * 10);

  // Set temperature decimal part
  data[5] = (td << 4);
  data[5] |= tf;

  // Set temperature float part
  data[4] |= (tempFloat << 4);
}

/**
 * \brief    Set the sensor humidity
 * \param    data       Oregon message
 * \param    hum        the humidity
 */
void setHumidity(byte* data, byte hum)
{
  data[7] = (hum/10);
  data[6] |= (hum - data[7]*10) << 4;
}

/**
 * \brief    Sum data for checksum
 * \param    count      number of bit to sum
 * \param    data       Oregon message
 */
int Sum(byte count, const byte* data)
{
  int s = 0;

  for(byte i = 0; i<count;i++)
  {
    s += (data[i]&0xF0) >> 4;
    s += (data[i]&0xF);
  }

  if(int(count) != count)
    s += (data[count]&0xF0) >> 4;

  return s;
}

/**
 * \brief    Calculate checksum
 * \param    data       Oregon message
 */
void calculateAndSetChecksum(byte* data)
{
#ifdef THN132N
  int s = ((Sum(6, data) + (data[6]&0xF) - 0xa) & 0xff);

  data[6] |=  (s&0x0F) << 4;     
  data[7] =  (s&0xF0) >> 4;
#else
  data[8] = ((Sum(8, data) - 0xa) & 0xFF);
#endif
}



