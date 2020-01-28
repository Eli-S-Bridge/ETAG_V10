/*
 * Manchester.h
 *
 * @author Tho Trinh, Eli Bridge, Jay Wilhelm
 * @version 08-02-2019
 *
 * This library (created for Arduino) allows the user to read in RFID data using Manchester Decoding
 * More info on Manchester Decoding can be found here: http://www.priority1design.com.au/em4100_protocol.html.
 * The functions FastRead and INT_demodOut work together to create an interrupt driven means of reading
 * RFID tags. The FastRead function sets up interrupts that call the INT_demodOut handler as the input from
 * an RFID front end IC (i.e. an EM4095) provides high/low modulation from its output pin. The interrupt
 * handler measures the durations of each pulse to generate a data stream. It allows for a quick determination 
 * as to whether a tag is present, and if not the read attempt is ended quickly. If a tag is detected,
 * the read attempt is continued until the operation times out (according to a user specified read time)
 * or a tag is read successfully. The function will seek out the ID initialization code (nine 1s in a row)
 * and will perform a parity check to make sure the data read in are accurate. 
 */

#pragma once

#ifndef MANCHESTER_H_
#define MANCHESTER_H_

/**********Included Needed Files********************/
#include <Wire.h>
#include <SPI.h>
#include <SD.h>              // include the standard SD card library

/***********Include needed constants to set up pins***********/
#define serial SerialUSB     // Designate the USB connection as the primary serial comm port
#define DEMOD_OUT_1      41  // (PB22) this is the target pin for the raw RFID data from RF circuit 1
#define DEMOD_OUT_2      42  // (PB23) this is the target pin for the raw RFID data from RF circuit 2
#define SHD_PINA         48  // (PB16) Setting this pin high activates RFID circuit 1 
#define SHD_PINB         49  // (PB17) Setting this pin high activates RFID circuit 2 



//Declare needed global variables in here
byte rParity;                         // temporary storage of parity data.
unsigned int parityFail;              // Indicates if there was a parity mismatch (i.e. a failed read)
unsigned int pulseCount;              // For counting pulses from RFID reader
byte OneCounter;                      // For counting the number of consecutive 1s in RFID input -- 9 ones signals the beginning of an ID code
byte longPulseDetected = 0;           // A long pulse must first be read from the RFID output to begin reading a tag ID - This variable indicates whether a long pulse has happened
byte pastPulseLong = 0;               // Indicates whether the past pulse was long (1) or short (0).
byte RFIDbitCounter;                  // Counts the number of bits that have gone into an RFID tag read
byte RFIDbyteCounter;                 // Counts the number of bytes that have gone into an RFID tag read
byte RFIDbytes[11];                   // Array of bytes for storing all RFID tag data (ID code and parity bits)
int IntPin;                           // Pin for RFID input (interrupt pin)

/******************Functions Declarations***********************/
void processTag(byte *RFIDtagArray, char *RFIDstring, byte RFIDtagUser, unsigned long *RFIDtagNumber);
//checks if there is a parity fail when a pulse has been detected, if the parity is fine, then the tag will start reading in data.
byte FastRead(byte whichCircuit, byte checkDelay, unsigned int readTime);
void INT_demodOut();
void shutDownRFID();

/*********************Functions Definitions*************************/
/*
 * Function combines the individual tag lines into one hexadecimal number.
 * @parameters -
 *      RFIDtagArray - byte array of length 5 to store the individual bytes of a Tag ID
 *      RFIDstring - charArray(String) of length 10 that stores the TagID
 *      RFIDtagUser - byte that stores the first(most signficant) byte of a tag ID(user #)
 *      RFIDtagNumber - unsigned long that stores bytes 1 through 4 of a tag ID(user #)
 * @return -
 *      nothing
 */
void processTag(byte *RFIDtagArray, char *RFIDstring, byte RFIDtagUser, unsigned long *RFIDtagNumber)
{
  // process each byte (could do a loop but.....)
  RFIDtagArray[0] = ((RFIDbytes[0] << 3) & 0xF0) + ((RFIDbytes[1] >> 1) & 0x0F);
  String StringOne = String(RFIDtagArray[0], HEX);
  if(RFIDtagArray[0] < 0x10) {StringOne = String("0" + StringOne);}
  RFIDtagUser = RFIDtagArray[0];

  RFIDtagArray[1] = ((RFIDbytes[2] << 3) & 0xF0) + ((RFIDbytes[3] >> 1) & 0x0F);
  String StringTwo = String(RFIDtagArray[1], HEX);
  if(RFIDtagArray[1] < 0x10) {StringTwo = String("0" + StringTwo);}
  *RFIDtagNumber = RFIDtagArray[1] << 24;

  RFIDtagArray[2] = ((RFIDbytes[4] << 3) & 0xF0) + ((RFIDbytes[5] >> 1) & 0x0F);
  String StringThree = String(RFIDtagArray[2], HEX);
  if(RFIDtagArray[2] < 0x10) {StringThree = String("0" + StringThree);}
  *RFIDtagNumber = *RFIDtagNumber + (RFIDtagArray[2] << 16);

  RFIDtagArray[3] = ((RFIDbytes[6] << 3) & 0xF0) + ((RFIDbytes[7] >> 1) & 0x0F);
  String StringFour = String(RFIDtagArray[3], HEX);
  if(RFIDtagArray[3] < 0x10) {StringFour = String("0" + StringFour);}
  *RFIDtagNumber = *RFIDtagNumber + (RFIDtagArray[3] << 8);

  RFIDtagArray[4] = ((RFIDbytes[8] << 3) & 0xF0) + ((RFIDbytes[9] >> 1) & 0x0F);
  String StringFive = String(RFIDtagArray[4], HEX);
  if(RFIDtagArray[4] < 0x10) {StringFive = String("0" + StringFive);}
  *RFIDtagNumber = *RFIDtagNumber + RFIDtagArray[4];
  String(StringOne + StringTwo + StringThree + StringFour + StringFive).toCharArray(RFIDstring, 11); //updates the RFIDstring with the five new strings

//  serial.println();
//  serial.println(StringOne);
//  serial.println(StringTwo);
//  serial.println(StringThree);
//  serial.println(StringFour);
//  serial.println(StringFive);
//  serial.println(RFIDstring);

  do
   {
      *RFIDstring = toupper( *RFIDstring );  //capitalize each character in the char array

      RFIDstring++;
   }   while ( *RFIDstring != '\0' );

}

/*Sees if a tag is present and returns a 0 or 1 depending on whether or not it can be read
 * @parameters -
  *     checkDelay - a byte telling how long in milliseconds to check to see if
  *                  a tag is present (Tag is only partially read during this time --
  *                  This is just a quick way of detirmining if a tag is present or not
  *     whichCircuit - byte showing which RFID circuit(antenna) is reading
  *     readTime - How long in milliseconds to try to read a tag if a tag was initially detected
  *
  *     @returns - 0 or 1 whether tag can be read in or not
  */
byte FastRead(byte whichCircuit, byte checkDelay, unsigned int readTime) {
  if (whichCircuit == 1) {
    digitalWrite(SHD_PINA, LOW);       // Turn on primary RFID circuit
    digitalWrite(SHD_PINB, HIGH);      // Turn off secondary RFID circuit
    IntPin = DEMOD_OUT_1;              // Circuit 1 input source 
    
  } else {
    digitalWrite(SHD_PINA, HIGH);      // Turn off primary RFID circuit
    digitalWrite(SHD_PINB, LOW);       // Turn on secondary RFID circuit
    IntPin = DEMOD_OUT_2;              // Circuit 2 input source 
  }
  pinMode(IntPin, INPUT);        // set up RFID data pin as an input
  // serial.println("fast read activated...");
  rParity = 0;
  parityFail = 0x07FF;  // start with 11 bits set and clear one for every line-parity check that passes, and clear the last for the column parity check
  pulseCount = 0;
  OneCounter = 0;
  longPulseDetected = 0;
  pastPulseLong = 0;
  RFIDbyteCounter = 0;
  RFIDbitCounter = 4;                  // counts backwards from 4 to zero
  memset(RFIDbytes, 0, sizeof(RFIDbytes));  // Clear RFID memory space
  unsigned long currentMillis = millis();   // To determine how long to poll for tags, first get the current value of the built in millisecond clock on the processor
  unsigned long stopMillis = currentMillis + readTime;
  attachInterrupt(digitalPinToInterrupt(IntPin), INT_demodOut, CHANGE);

  // delay(checkTime);
  delay(checkDelay);
  // serial.print("pulses detected... ");
  // serial.println(pulseCount, DEC);
  if (pulseCount > (checkDelay - 25)) {     // May want a separate variable for threshold pulse count.
    while (millis() < stopMillis & parityFail != 0) {
      delay(1);
    }
  } else {
    detachInterrupt(digitalPinToInterrupt(IntPin));
    shutDownRFID();        // Turn off both RFID circuits
    // serial.print("nothing read... ");
    return (0);
  }

  detachInterrupt(digitalPinToInterrupt(IntPin));
  shutDownRFID();        // Turn off both RFID circuits
  if (parityFail == 0) {
    //serial.print("parityOK... ");
    return (1);
  } else {
    //serial.print("parity fail... ");
    return (0);
  }

}

/*
 * the ISR function called for attachInterrupt.
 * start adding data to rfidbytes, this reads in tag data
 */
void INT_demodOut()
{
  volatile uint32_t timeNow = micros();              // Store the current microsecond timer value in timeNow
  volatile static uint32_t lastTime = 0;             // Clear this variable
  uint16_t fDiff = timeNow - lastTime;               // Calculate time elapsed since the last execution of this function
  lastTime = timeNow;                                // Establish a new value for lastTime
  // int8_t fTimeClass = ManchesterDecoder::tUnknown;// ??????
  int16_t fVal = digitalRead(IntPin);         // set fVal to the opposite (!) of the value on the RFID data pin (default is pin 30).
  byte RFbit = 255;                                  // set to default, 255, (no bit read)

  if (fDiff > 395 & fDiff < 600) {
    pulseCount++;
    longPulseDetected = 1;
    pastPulseLong = 1;
    RFbit = 200;                                     // Indicate that successful reading is still going on
    if (OneCounter < 9) {
      fVal == 1 ? OneCounter++ : OneCounter = 0;     // If we have read a 1 add to the one counter. if not clear the one counter
    } else {
      RFbit = fVal;
    }
  }
  if (fDiff < 395 & fDiff > 170) {
    pulseCount++;
    RFbit = 200;                                         // Indicate that successful reading is still going on
    if (longPulseDetected == 1 && pastPulseLong == 1) {  // Before this input means anything we must have registered one long bit and the last pulse must have been long (or a transition bit)
      if (OneCounter < 9) {                              // Only write tag bits when we have read 9 ones.
        fVal == 1 ? OneCounter++ : OneCounter = 0;       // If we have read a 1 add to the one counter. if not clear the one counter
      } else {
        RFbit = fVal;
      }
      pastPulseLong = 0;                             // Indicate that the last pulse was short
    } else {
      pastPulseLong = 1;                             // Indicate that the last pulse was long.
                                                     // This is not really true, but the second of two consecutive short pulses means the next pulse should indicate a read bit.
    }
  }

  // Now check if RFbit was changed from 255 and if so add to the data compiled in RFIDbytes
  if (RFbit < 100) {
    RFbit == 1 ? bitSet(RFIDbytes[RFIDbyteCounter], RFIDbitCounter) : bitClear(RFIDbytes[RFIDbyteCounter], RFIDbitCounter); // Set or clear the RFID data bit
    if (RFIDbitCounter > 0) {
      rParity = rParity ^ RFbit;   // Calculate running parity bit -- Exclusive or between row parity variable and current RF bit
      RFIDbitCounter--;
    } else {

      if ((RFIDbitCounter == 0) & (RFIDbyteCounter < 10)) {  // Indicates we are at the end of a line - Do a line parity check
        byte tb = RFIDbytes[RFIDbyteCounter];
        rParity = ((tb >> 4) & 1) ^ ((tb >> 3) & 1) ^ ((tb >> 2) & 1) ^ ((tb >> 1) & 1);
        rParity == (tb & 1) ? bitClear(parityFail, RFIDbyteCounter) : bitSet(parityFail, RFIDbyteCounter); // Check parity match and adjust parityFail
        rParity = 0;
        RFIDbyteCounter++;
        RFIDbitCounter = 4;
      }

      if ((RFIDbitCounter == 0) & (RFIDbyteCounter == 10)) { // Indicates we are on the last bit of an ID code
        // test all column parity
        byte xorByte = (RFIDbytes[10] & B00011111) >> 1;
        for (byte i = 0; i <= 9; i++) { // loop through bytes 1 though 9 (parity row included on first interation - should Xor out to zero
          xorByte = xorByte ^  (RFIDbytes[i] >> 1);
        }
        if (xorByte == 0) {
          bitClear(parityFail, RFIDbyteCounter) ;            // If parity checks out clear the last bit
        }
      }
    }
  }

  if ((RFbit == 255) & (pulseCount != 0)) {                  // no pulse detected, clear everything except pulseCount
    rParity = 0;
    parityFail = 0x07FF;
    OneCounter = 0;
    longPulseDetected = 0;
    pastPulseLong = 0;
    RFIDbyteCounter = 0;
    RFIDbitCounter = 4;                                      // counts backwards from 4 to zero
    memset(RFIDbytes, 0, sizeof(RFIDbytes));                 // Clear RFID memory space
  }
}

void shutDownRFID() {    // Just shut down both RFID circuit
  digitalWrite(SHD_PINA, HIGH);             // Turn off primary RFID circuit
  digitalWrite(SHD_PINB, HIGH);             // Turn off secondary RFID circuit
}

 #endif
