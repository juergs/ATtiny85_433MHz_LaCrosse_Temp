/************************************************************************
 *  433MHz - Dallas18B20  temperature-sensor [@FHEM.de]
 *  
 *  juergs, 16.10.2016, initial version.
 *  juergs, 04.02.2017, reorganized version. (Temp-Dallas-Sensor added.)
 *  juergs, 15.12.2018, reorganized for Dallas temperatur sensor only 
 *  *********************************************************************
 * Special thanks to: http://www.f6fbb.org/domo/sensors/tx3_th.php
 * Special thanks to: http://www.f6fbb.org/domo/sensors/tx_signals.php
 * Special thanks to: http://forum.arduino.cc/index.php?topic=155483.0
 * Special thanks to: https://forum.fhem.de/index.php/topic,50333.0.html
 * Special thanks to: http://forum.arduino.cc/index.php?topic=390843.msg2692215#msg2692215
 * Special thanks to: http://www.guillier.org/blog/2014/09/lacrosse-tx2tx3-sensors-and-digispark/
 * Special thanks to: https://github.com/guillier/433mhz_receiver
 * Special thanks to: http://forum.arduino.cc/index.php?PHPSESSID=78ulth4v3mi4eisfjti1kpa576&topic=155483.msg1165647#msg1165647
 * Special thanks to: http://www.guillier.org/blog/tag/433mhz.html
 * *********************************************************************
 *  ATMEL ATTINY 25/45/85 omly!
 *
 *                      +-\/-+
 *     RES   (D5) PB5  1|    |8  Vcc
 *     TX433 (D3) PB3  2|    |7  PB2 (D 2) Ain1  Bodenfeuchte-Pulse  
 *     Serial(D4) PB4  3|    |6  PB1 (D 1) pwm1  Bodenfeuchte-Power
 *                GND  4|    |5  PB0 (D 0) pwm0  Dallas-Sensor Data DQ
                        +----+
 *  
 *  Install: ATtiny-Models in Arduino IDE:
 *  http://highlowtech.org/?p=1695
 *  https://raw.githubusercontent.com/damellis/attiny/ide-1.6.x-boards-manager/package_damellis_attiny_index.json,   -> for preferences additional board manager url + installation
 *  
 *  ATTINY:  
 *    https://cpldcpu.wordpress.com/2014/04/25/the-nanite-85/ 
 *    https://thewanderingengineer.com/2014/08/11/pin-change-interrupts-on-attiny85/
 *  
 *  LaCrosse-TH3-protocol decoded:
 *    http://www.f6fbb.org/domo/sensors/tx3_th.php
 *    
 * The LaCrosse-protocol seems to be:
 *
 *     Bits 0-3: header
 *     Bits 4-11: device ID, changes when replacing the batteries. Unlike in the post linked above, bit 11 does not appear to be a checksum.
 *     Bits 12-15: either 1111 for automatic transmission (once every 60 seconds) or 1011 for manual transmission (using the button in the battery compartment). Manual transmission does not update the weather station.
 *     Bits 16-27: encode the temperature. The system of encoding decimal digits seems to be ditched in favor of a more elegant one: apply a NOT (change 1 to 0 and 0 to 1), convert to base 10, divide by 10 (into a float), subtract 50, and the result is the temperature in C.
 *     Bits 28-35: encode the relative humidity. Apply a NOT, convert to base 10, and the result is the relative humidity in %.
 *     Bits 36-43: appear to encode a checksum (though I plan to double-check if this is not the dew point, also reported by the weather station).
      
 *     Example:
 *     HHHH 1000 0010 1111 1101 0010 1111 1101 0011 1010 0100
 *     encoding T=22.0C and RH=44%
 *  
 */

/******************************************************************************************************************************************************/
/* ATtiny 84/85 SerialMonitor Test 
   -------------------------------
 Senden via "SoftwareSerial" - TX an Pin  4 (= Pin3 am Attiny85-20PU)
 Senden via "SoftwareSerial" - TX an Pin  7 (= Pin6 am Attiny84-10PU)
 Empfangen via "SoftwareSerial" - RX an Pin 99 (Dummy um Hardwarepin zu sparen)
 * AVR half-duplex software UART supporting single pin operation 
    http://nerdralph.blogspot.com/2014/01/avr-half-duplex-software-uart.html
*/
/*
*                                ATtiny-85-Version!!!
*
*******************************************************************************************
*   Bei Programmer-Fehler: Dallas Sensor entfernen! Wenn: Mosi = Data & SCK = Sensor !
*   Auf Pin-Belegung der Platine achten!!!!! -> Board_Typ setzen!
*******************************************************************************************
*      Auszug aus der Beschreibung des Herstellers:
– 120 Powerful Instructions – Most Single Clock Cycle Execution
– 32 x 8-bit General Purpose Working Registers
– Up to 20 MIPS Througput at 20 MHz
– 8K Bytes of In-System Programmable Program Memory Flash
– 512 Bytes In-System Programmable EEPROM
– 512 Bytes Internal SRAM
– Write/Erase Cycles: 10,000 Flash/100,000 EEPROM
– Data retention: 20 Years at 85°C/100 Years at 25°C
– 8-bit Timer/Counter with Prescaler and Two PWM Channels
– 8-bit High Speed Timer/Counter with Separate Prescaler
– USI – Universal Serial Interface with Start Condition Detector
– 10-bit ADC: 4 Single Ended Channels, 2 Differential ADC Channel Pairs
– Temperature Measurement
– On-chip Analog Comparator
– Low Power Idle, ADC Noise Reduction, and Power-down Modes
– Internal Calibrated Oscillator
– Six Programmable I/O Lines
– 2.7 - 5.5V Operating Voltage

- Gehäuseform: DIP, 8-polig, Herstellerbezeichnung "ATtiny85-20PU"

*** Fuses auf 8Mhz internal clock gestellt (Standard) und Clock-Divider nicht gesetzt!
avrdude: safemode: Fuses OK (H:FF, E:DF, L:E2)
Uploading to I/O board using 'USBasp'
Uploader started for board ATtiny w/ ATtiny85
Uploader will use programmer name: usbasp
C:\Users\js\AppData\Local\arduino15\packages\arduino\tools\avrdude\6.0.1-arduino5\bin\avrdude "-CC:\Users\js\AppData\Local\arduino15\packages\arduino\tools\avrdude\6.0.1-arduino5/etc/avrdude.conf" -v -pattiny85 -cusbasp -Pusb "-Uflash:w:C:\Users\js\AppData\Local\Temp\VMicroBuilds\ATtiny85_LaCrosse_TempSensor\attiny_attiny_attiny85/ATtiny85_LaCrosse_TempSensor.ino.hex:i"
avrdude: Version 6.0.1, compiled on Apr 15 2015 at 19:59:58
Copyright (c) 2000-2005 Brian Dean, http://www.bdmicro.com/
Copyright (c) 2007-2009 Joerg Wunsch
System wide configuration file is "C:\Users\js\AppData\Local\arduino15\packages\arduino\tools\avrdude\6.0.1-arduino5/etc/avrdude.conf"
Using Port                    : usb
Using Programmer              : usbasp
AVR Part                      : ATtiny85
Chip Erase delay              : 400000 us
PAGEL                         : P00
BS2                           : P00
RESET disposition             : possible i/o
RETRY pulse                   : SCK
serial program mode           : yes
parallel program mode         : yes
Timeout                       : 200
StabDelay                     : 100
CmdexeDelay                   : 25
SyncLoops                     : 32
ByteDelay                     : 0
PollIndex                     : 3
PollValue                     : 0x53
Memory Detail                 :
Block Poll               Page                       Polled
Memory Type Mode Delay Size  Indx Paged  Size   Size #Pages MinW  MaxW   ReadBack
----------- ---- ----- ----- ---- ------ ------ ---- ------ ----- ----- ---------
eeprom        65    12     4    0 no        512    4      0  4000  4500 0xff 0xff
flash         65     6    32    0 yes      8192   64    128 30000 30000 0xff 0xff
signature      0     0     0    0 no          3    0      0     0     0 0x00 0x00
lock           0     0     0    0 no          1    0      0  9000  9000 0x00 0x00
lfuse          0     0     0    0 no          1    0      0  9000  9000 0x00 0x00
hfuse          0     0     0    0 no          1    0      0  9000  9000 0x00 0x00
efuse          0     0     0    0 no          1    0      0  9000  9000 0x00 0x00
calibration    0     0     0    0 no          2    0      0     0     0 0x00 0x00
Programmer Type : usbasp
Description     : USBasp, http://www.fischl.de/usbasp/


ATMEL ATTINY 25/45/85 / ARDUINO

+-\/-+
Ain0 (D 5) PB5  1|    |8  Vcc
Ain3 (D 3) PB3  2|    |7  PB2 (D 2) Ain1
Ain2 (D 4) PB4  3|    |6  PB1 (D 1) pwm1
GND  4|    |5  PB0 (D 0) pwm0
+----+

Install ATtiny - Models in Arduino IDE:
http://highlowtech.org/?p=1695

ATtiny:
https://cpldcpu.wordpress.com/2014/04/25/the-nanite-85/

Powersave modes:
================
http://www.rocketscream.com/blog/2011/07/04/lightweight-low-power-arduino-library/
http://www.gammon.com.au/forum/?id=11488&reply=9#reply9
http://gammon.com.au/forum/?id=11497&reply=6#reply6
http://www.gammon.com.au/power
https://forum.arduino.cc/index.php?topic=326237.0
https://www.insidegadgets.com/2011/02/05/reduce-attiny-power-consumption-by-sleeping-with-the-watchdog-timer/
... und weitere Powersave-Gedanken:
https://harizanov.com/2013/08/every-%CE%BCa-counts/
http://electronics.stackexchange.com/questions/49182/how-can-i-get-my-atmega328-to-run-for-a-year-on-batteries

Hier noch ein weiteres Projekt mit RFM69:
http://johan.kanflo.com/the-aaduino/

und der Sketch dazu:
https://github.com/kanflo/aaduino

*
* The LaCrosse-protocol seems to be:
*
Bits 0-3: header
Bits 4-11: device ID, changes when replacing the batteries. Unlike in the post linked above, bit 11 does not appear to be a checksum.
Bits 12-15: either 1111 for automatic transmission (once every 60 seconds) or 1011 for manual transmission (using the button in the battery compartment). Manual transmission does not update the weather station.
Bits 16-27: encode the temperature. The system of encoding decimal digits seems to be ditched in favor of a more elegant one: apply a NOT (change 1 to 0 and 0 to 1), convert to base 10, divide by 10 (into a float), subtract 50, and the result is the temperature in C.
Bits 28-35: encode the relative humidity. Apply a NOT, convert to base 10, and the result is the relative humidity in %.
Bits 36-43: appear to encode a checksum (though I plan to double-check if this is not the dew point, also reported by the weather station).

Example:
HHHH 1000 0010 1111 1101 0010 1111 1101 0011 1010 0100
encoding T=22.0C and RH=44%


//**************************************************************
// Thanks to: http://www.f6fbb.org/domo/sensors/tx3_th.php
// Thanks to: http://forum.arduino.cc/index.php?topic=155483.0
// Thanks to: https://forum.fhem.de/index.php/topic,50333.0.html
//**************************************************************
//  OneWire DS18S20, DS18B20, DS1822 Temperature Version 
//  using this Arduino Library: http://www.pjrc.com/teensy/td_libs_OneWire.html
//  The DallasTemperature library can do all this work for you!
//  http://milesburton.com/Dallas_Temperature_Control_Library
//  http://images.google.de/imgres?imgurl=http://www.tweaking4all.com/wp-content/uploads/2014/03/ds18b20-waterproof.jpg&imgrefurl=http://www.tweaking4all.com/hardware/arduino/arduino-ds18b20-temperature-sensor/&h=988&w=800&tbnid=mowdJDteDQmw_M:&tbnh=104&tbnw=84&docid=7g-v-bKlWHiqKM&usg=__9sTNcsYyWEgAZF-aP5rpUuvCyio=&sa=X&ved=0ahUKEwiRvJfp44HMAhVDDCwKHc1OBgcQ9QEIKjAB
//  http://www.tweaking4all.com/hardware/arduino/arduino-ds18b20-temperature-sensor/
//  https://github.com/PaulStoffregen/OneWire
    https://arduino-info.wikispaces.com/Brick-Temperature-DS18B20
    https://arduino-info.wikispaces.com/MultipleTemperatureSensorsToLCD
    http://www.pjrc.com/teensy/td_libs_OneWire.html

    Hardware Overview and HowTo:
    http://www.tweaking4all.com/hardware/arduino/arduino-ds18b20-temperature-sensor/

    Additionals:
    https://gcc.gnu.org/onlinedocs/gcc-3.1/cpp/Standard-Predefined-Macros.html
    https://gcc.gnu.org/onlinedocs/gcc-3.1/cpp/Invocation.html#Invocation
    https://gcc.gnu.org/onlinedocs/gcc/Preprocessor-Options.html
      
    http://www.ernstc.dk/arduino/tinycom.html

    ATTiny-Boards:
    https://www.iot-experiments.com/attiny85-development-board/
    
    How to connect multiple 18B20-Sensors:
    The DS18B20 Digital Temperature sensor allows you to set multiple in parallel. When doing this, the OneWire library will read all sensors.
    For larger networks of sensors (more than 10), using smaller resistors should be considered, for example 1.6 KΩ or even less.
    It has been observed that large amounts of sensors (more than 10) in the same network can cause issues (colliding data),
    and for that purpose an additional resistor of say 100 … 120 Ω should be added between the data line to the Arduino and the data pin of the sensor, for each sensor !

    Fuse-Settings for ATtiny85 for internal 16.5 MHz:
    ====================================================
    LF = 0xF1 ( PLL-Clock = 16.5 MHz clock, not internal 8 MHz)
    HF = 0xDF
    EF = 0xFF
    LB = 0x03
    
*****************************************************************************************************************************************************/
//--- external libs: see also http://www.ernstc.dk/arduino/tinycom.html
#include <SoftwareSerial.h>
#include <avr/power.h>  // for adc power save 

//--- leave these libs in ino-directory
#include "Narcoleptic.h"
#include "LaCrosse.h"
#include "OneWire.h"

#define SN ("DS18B20-Sensor-433 using LaCrosse TX3 protocol")
#define SV ("1.0 vom 15.12.2018")

//--- conditionals
//--- zum aktivieren Kommentierung entfernen            
//#define USE_SEPARATE_BATTERIE_ID 
#define USE_WITH_DALLAS_SENSOR 
//#define USE_SOFT_SERIAL_FOR_DEBUG         

#define DALLAS_SENSOR_PIN         0     //      D0 = Pin 5
#define TX_433_PIN                3     //      D3 = Pin2 = PIN_SEND (TX) in LaCrosse.cpp

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#define SENSOR_ID                 80
#define SENSORID_BATTERIE         81
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#define OW_ROMCODE_SIZE           8

#ifdef USE_WITH_DALLAS_SENSOR
   //--- die 18B20-Instanz setzen 
   OneWire  ds(DALLAS_SENSOR_PIN);     // on arduino port (a pullup 4.7K resistor is necessary, between Vcc and DQ-Pin   1=GND 2=DQ 3=Vcc )
#endif

//--- use with leds? 
#define LED_ONTIME                100  // Number of cycles for LEDs to stay on, when only temporary

//-- how many AA Cells do you want? Set 15 per cell, so 1*AA = 15, 2*AA=30
#define MAXBATTERY                 15  // Maximum voltage of Battery in 100 mV, for percentage calculation
#define DEEP_SLEEP_MINUTES          3    // duration of sleep phase

#ifdef USE_SOFT_SERIAL_FOR_DEBUG
  SoftwareSerial            softSerial(99, 4);            // RX, TX   // bei Tiny 85
#endif 
//--- Sensor instance 
OneWire                   dallas(DALLAS_SENSOR_PIN);  // on arduino port pin 2 (a 4.7K resistor is necessary, between Vcc and DQ-Pin   1=GND 2=DQ 3=Vcc )

unsigned int  led_startup;
float         controller_VCC = 0.0; 
long          lngVcc    = 0;

/*
 * Internals to follow, nothing to adjust
 */
byte          led_temporary = 1;                // Are LEDs jumpered to stay on=0, otherwise 1 
float         batteriespannung = 0.0;

//---------------------------------------------------------------------
//--- prototypes 
long  getVcc();
float ReadSingleOneWireSensor(OneWire ds); 
//---------------------------------------------------------------------
//---------------------------------------------------------------------
void setup() 
{
  //--- preset SensorId & TX instance 
  LaCrosse.bSensorId = SENSOR_ID;  
  pinMode(TX_433_PIN, OUTPUT);  
  digitalWrite(TX_433_PIN, LOW); 

  //--- init Softwareserial, only 1 pin is used, Trick (!): rx defined as pin 99! 
  //--- needs more flash memory! 
  #ifdef USE_SOFT_SERIAL_FOR_DEBUG
    softSerial.begin(38400);
  #endif
  delay(1000);    
}
//---------------------------------------------------------------------
void loop() 
{
  // comming from wake-up?
  //pinMode(DALLAS_SENSOR_PIN, OUTPUT);
  LaCrosse.setTxPinMode(OUTPUT);
  
  power_adc_enable();
  delay(500); // ms, needed for settling DS18B20
  
  //--- [0] Betriebsspannung auslesen  
  lngVcc          = getVcc();   // as long
  controller_VCC  = (lngVcc/1000.0);  // as float in Volt, not millivolts (sent as HUM !
  
  //--- [2] read Dallas-Sensor
  float theta = ReadSingleOneWireSensor(dallas);

  #ifdef USE_SOFT_SERIAL_FOR_DEBUG
    //--- debug-output-block
    //softSerial.print("Vcc: ");
    //softSerial.print( (float) controller_VCC, 1);
    //softSerial.print("   Vcc_read: ");
    //softSerial.print((long) lngVcc);
    //softSerial.print("   ");  
    softSerial.print("Feuchte: ");
    softSerial.print( (float) bodenfeuchte, 1);
    softSerial.print("    ");
    softSerial.print("Temp: ");
    softSerial.println( (float) theta, 1);
  #endif 
  
  //--- transfer measured values to LaCrosse-instance
  LaCrosse.bSensorId = SENSOR_ID;
  LaCrosse.t = theta;    //--- alias temperature;  
  LaCrosse.sendTemperature();
  
  LaCrosse.sleep(1);        /* 1 second, no power-reduction! */

  #ifdef USE_SEPARATE_BATTERIE_ID 
    LaCrosse.bSensorId = SENSORID_BATTERIE;
  #endif 
  
  //LaCrosse.h = bodenfeuchte/1000; //   controller_VCC;    
  //LaCrosse.sendHumidity();
  //LaCrosse.sleep(1);        /* 1 second, no power-reduction! */


  //--- preserve more power during sleep phase 
  pinMode(DALLAS_SENSOR_PIN, INPUT);  
  LaCrosse.setTxPinMode(INPUT);

  //--- switch AD-converter off
  power_adc_disable(); 

  //--- fall to deep powersave-sleep, see notes in comments and 
  Narcoleptic.delay_minutes(DEEP_SLEEP_MINUTES);

  //--- deep sleep or test?
  //delay(10000); // 10 Sec 
}
//---------------------------------------------------------------------
long getVcc() 
{
  //--- read 1.1V reference against AVcc
  //--- set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  
  
  delay(2); //-- wait for Vref to settle

  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA, ADSC)); // measuring

  uint8_t low = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both

  long result = (high << 8) | low;

  /***************************************************************************************
  *  Berechnung/Skalierung mit manueller Messung der Betriebsspannung:
  *
  *        internal1.1Ref = 1.1 * Vcc1 (per voltmeter) / Vcc2 (per readVcc() function)
  *                       = 1.1 * 5126         / 5258 => 1.09 ==> 1.09*1023*1000 = 1097049
  ****************************************************************************************/

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000

 //result = 1097049L / result; // korrigierter Wert bei 3V3 muesste fuer jeden Controller bestimmt werden, obiger Wert scheint allgemeiner zu sein.

  return result; // Vcc in millivolts
}
//-------------------------------------------------------------------------
float ReadSingleOneWireSensor(OneWire ds)
{
  //--- 18B20 stuff
  byte i;
  byte type_s;
  byte data[12];
  byte addr[8];
  float celsius = 12.3;

  if (!ds.search(addr))
  {
    ds.reset_search();
    delay(250);
    return celsius;
  }

  if (OneWire::crc8(addr, 7) != addr[7])
  {
    return celsius;
  }

  //--- the first ROM byte indicates which chip
  switch (addr[0])
  {
  case 0x10:
    //Serial.println("  Chip = DS18S20");  // or old DS1820
    type_s = 1;
    break;
  case 0x28:
    // Serial.println("  Chip = DS18B20");
    type_s = 0;
    break;
  case 0x22:
    // Serial.println("  Chip = DS1822");
    type_s = 0;
    break;
  default:
    // Serial.println("Device is not a DS18x20 family device.");
    return celsius;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44,1 );  //--- start conversion, use alternatively ds.write(0x44,1) with parasite power on at the end

  delay(1000);        //--- maybe 750ms is enough, maybe not
                      //--- we might do a ds.depower() here, but the reset will take care of it.

  ds.reset();         //--- DS18B20 responds with presence pulse
                      //--- match ROM 0x55, sensor sends ROM-code command ommitted here.
  ds.select(addr);
  ds.write(0xBE);     //--- read scratchpad
  for (i = 0; i < 9; i++)
  {
    //--- we need 9 bytes, 9th byte is CRC, first 8 are data
    data[i] = ds.read();
  }

  //--- Convert the data to actual temperature
  //--- because the result is a 16 bit signed integer, it should
  //--- be stored to an "int16_t" type, which is always 16 bits
  //--- even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s)
  {
    raw = raw << 3;     //--- 9 bit resolution default
    if (data[7] == 0x10)
    {
      //--- "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    };
  }
  else
  {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    // default is 12 bit resolution, 750 ms conversion time
    if (cfg == 0x00) raw = raw & ~7;        // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3;   // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1;   // 11 bit res, 375 ms
  };

  celsius = (float)raw / 16.0;    //fahrenheit = celsius * 1.8 + 32.0;

  //---- Check if any reads failed and exit early (to try again).  
  if (isnan(celsius))
  {
    //--- signalize error condition 
    celsius = -99.9;
  };
  return celsius;
}
//-------------------------------------------------------------------------
void blink(byte pin, int delay_ms)
{
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  delay(delay_ms);
  digitalWrite(pin, LOW);
  delay(delay_ms);
}
//---------------------------------------------------------------------
//---------------------------------------------------------------------
//---------------------------------------------------------------------
// <eof>
//---------------------------------------------------------------------

