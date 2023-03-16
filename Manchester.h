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
uint16_t crc;
uint8_t crcOK;                         
unsigned int parityFail;              // Indicates if there was a parity mismatch (i.e. a failed read)
uint16_t pulseCount;                  // For counting pulses from RFID reader
uint8_t pulse2 = 0;                   // a 1 indicates we are looking at the second pulse of a zero bit in Biphase encoding
byte OneCounter;                      // For counting the number of consecutive 1s in RFID input -- 9 ones signals the beginning of an ID code
uint16_t tenZ;                        // For counting the number of consecutive 0s in RFID input -- 10 zeros signals the beginning of an ISO ID code
byte longPulseDetected = 0;           // A long pulse must first be read from the RFID output to begin reading a tag ID - This variable indicates whether a long pulse has happened
byte pastPulseLong = 0;               // Indicates whether the past pulse was long (1) or short (0).

union                                          // Make a union structure for tracking input bits
{struct
  {uint8_t bitCounter; uint8_t byteCounter;};  // 2 bytes that make up RFID counter
  uint16_t counter;                            // Name for 16bbit variable
} RFID;                                        // Name of the union variable (use RFID.counter or RFID.byteCounter or RFID.bitCounter)



uint16_t RFIDCounter;                 // MSB is byte counter, LSB is bit counter


byte RFIDbitCounter;                  // Counts the number of bits that have gone into an RFID tag read
byte RFIDbyteCounter;                 // Counts the number of bytes that have gone into an RFID tag read
byte RFIDbytes[16];                   // Array of bytes for storing all RFID tag data (ID code and parity bits)
uint8_t messageBytes;                 // Number of bytes in RFID message (5 for EM4100; 8 for normal ISO; 
int IntPin;                           // Pin for RFID input (interrupt pin)
uint16_t temp[600];
uint16_t tc = 0;

/******************Functions Declarations***********************/
void processTag(byte *RFIDtagArray, char *RFIDstring, byte RFIDtagUser, unsigned long *RFIDtagNumber);
//checks if there is a parity fail when a pulse has been detected, if the parity is fine, then the tag will start reading in data.
byte FastRead(byte whichCircuit, unsigned int checkDelay, unsigned int readTime);
byte ISOFastRead(byte whichCircuit, unsigned int checkDelay, unsigned int readTime);
void INT_demodOut();
void ISOINT_demodOut();
void shutDownRFID();
uint16_t crc16k(uint16_t crc, uint8_t *mem, uint8_t len);

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
  do {
     *RFIDstring = toupper( *RFIDstring );  //capitalize each character in the char array
     RFIDstring++;
   } while ( *RFIDstring != '\0' );
}

void processISOTag(byte *RFIDtagArray, char *RFIDstring, uint16_t *countryCode, uint8_t *tagTemp, uint32_t *RFIDtagNumber)
{
  for(uint8_t i = 0; i <= 5; i++) {RFIDtagArray[i] = RFIDbytes[i];}
  *tagTemp = RFIDbytes[10];
  *RFIDtagNumber = (RFIDtagArray[3]<<24) + (RFIDtagArray[2]<<16) + (RFIDtagArray[1]<<8) + RFIDtagArray[0];
  *countryCode = (RFIDtagArray[5]<<2) + (RFIDtagArray[4]>>6);
  sprintf(RFIDstring, "%03X.%02X%02X%02X%02X%02X", 
             *countryCode, (RFIDbytes[4] & 0b00111111), RFIDbytes[3], RFIDbytes[2], RFIDbytes[1], RFIDbytes[0]);
    do {
     *RFIDstring = toupper( *RFIDstring );  //capitalize each character in the char array
     RFIDstring++;
   } while ( *RFIDstring != '\0' );
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
byte FastRead(byte whichCircuit, unsigned int checkDelay, unsigned int readTime) {
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














byte ISOFastRead(byte whichCircuit, unsigned int checkDelay, unsigned int readTime) {
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
  //serial.println("fast read activated...");
  rParity = 0;
  parityFail = 0x07FF;  // start with 11 bits set and clear one for every line-parity check that passes, and clear the last for the column parity check
  crc = 0;
  crcOK = 0;
  pulseCount = 0;
  tenZ = 0xFFFF;
  longPulseDetected = 0;
  pastPulseLong = 0;
  RFID.byteCounter = 0;
  RFID.bitCounter = 10;                  // counts bits in each RFID byte
  memset(RFIDbytes, 0, sizeof(RFIDbytes));  // Clear RFID memory space
  unsigned long currentMillis = millis();   // To determine how long to poll for tags, first get the current value of the built in millisecond clock on the processor
  unsigned long stopMillis = currentMillis + readTime;
  attachInterrupt(digitalPinToInterrupt(IntPin), ISOINT_demodOut, CHANGE);

  // delay(checkTime);
  delay(checkDelay);
  //serial.print("pulses detected... ");
  //serial.println(pulseCount, DEC);
  if (pulseCount > (checkDelay - 25)) {     // May want a separate variable for threshold pulse count.
      while (millis() < stopMillis & crcOK != 3) {
        delay(1);
      }
      //serial.print("Exiting read loop... ");
  } else {
    detachInterrupt(digitalPinToInterrupt(IntPin));
    shutDownRFID();        // Turn off both RFID circuits
    //serial.println("nothing detected... ");
    return (0);
  }

  detachInterrupt(digitalPinToInterrupt(IntPin));
  //serial.println("read completed... ");
  //serial.println(crcOK);
  shutDownRFID();        // Turn off both RFID circuits
  if(crcOK < 3) {  //Start over if crc did not check out.
    return (0); 
  } else {
    return (1);
  }
}

/*
 * the ISR function called for attachInterrupt.
 * start adding data to rfidbytes, this reads in tag data
 */
void ISOINT_demodOut()
{
  if(crcOK != 3) { //Do nothing if tag read is complete
    //Get time elapsed since last interrupt
    volatile uint32_t timeNow = micros();              // Store the current microsecond timer value in timeNow
    volatile static uint32_t lastTime = 0;             // Clear this variable
    uint16_t fDiff = timeNow - lastTime;               // Calculate time elapsed since the last execution of this function
    lastTime = timeNow;                                // Establish a new value for lastTime
  
    //Use pulse interval to interpret input
    uint8_t switchVar = 0;                             //default switchVar value 
    if(fDiff > 85 & fDiff < 170) {switchVar = 1;}      //Short pulse switchVar value
    if(fDiff > 200 & fDiff < 275) {switchVar = 2;}     //Long pulse switchVar value
    if((RFID.byteCounter==9) && (RFID.bitCounter==8)){ //Time to check CRC.
      crc = crc16k(0x0000, RFIDbytes, 8);
      if(crc == (RFIDbytes[9]<<8) + RFIDbytes[8]){
        crcOK=1;
//        if (RFIDbytes[6] & 0b00000001 == 0) {  // This signals an end to the read when there is no auxilliary data
//          crcOK = 3;
//          switchVar = 3;
//        }   
      } else { 
        switchVar = 0;   // If CRC fails start over
      }
    }
    if((RFID.byteCounter==12) && (RFID.bitCounter==8)) {switchVar = 3;}
    
    //EXECUTION PATHS for different pulse lengths
    switch (switchVar) {  
       case 1: {  
              //serial.print("short pulse "); 
              //serial.print(pulseCount);
              if(pulse2 == 0) {  //Ignore second pulse in zero bits
                if(RFID.bitCounter != 8) {   //bit Counter should never be 8 on a short pulse.
                  //serial.print(" first short pulse ");
                  pulse2 = 1;
                  pulseCount++;
                  uint16_t tempZ = tenZ & 0b0000001111111111;
                  if(tempZ != 0) {
                    tenZ = tenZ << 1; //Shift over, leave lsb as zero 
                    //serial.print("Add 0 ");
                    //serial.println(tenZ, BIN);
                  } else {
                    //serial.print("GOT 10 Z ");
                    //serial.println(RFID.bitCounter, BIN);
                    bitClear(RFIDbytes[RFID.byteCounter], RFID.bitCounter); //Not needed if RFID bytes are already zero
                    RFID.bitCounter++;
                  }
                } else {
                  RFID.byteCounter = 0;   //reset counter
                  RFID.bitCounter = 10;   //reinitialize counter
                  tenZ = 0xFFFF;          //restart search for 10 zeros
                }
              } else {
                  pulse2 = 0;
                  //serial.print(" 2nd short pulse (ignored) ");
              }
              //serial.println(); 
              break;
            }
    
       case 2: {   
              //serial.print("LONG PULSE ");
              //serial.println(RFID.counter);
              pulse2 = 0;
              pulseCount++;
              uint16_t tempZ = tenZ & 0b0000001111111111;
              if(tempZ != 0) {
                 tenZ = tenZ << 1; //Shift over
                 tenZ = tenZ + 1;  //Make lsb one    
              } else {
                //serial.println(" Already GOT 10 ZEROS ");
                //serial.println(RFID.bitCounter);
                if(RFID.bitCounter < 8) {
                  bitSet(RFIDbytes[RFID.byteCounter], RFID.bitCounter);
                  RFID.bitCounter++;
                  } else {
                    if(RFID.bitCounter == 8){
                      //serial.print(RFIDbytes[RFID.byteCounter], HEX);
                      //serial.print(" ");
                      RFID.bitCounter = 0;
                      RFID.byteCounter++;
                    }
                    if(RFID.bitCounter >= 9){   //Bit counter initially set to 10.
                      RFID.bitCounter = 0;
                      RFID.byteCounter = 0;
                      //RFID.byteCounter++;
                    }
                  }
  
              }
           break;     
       }
       case 0: {   //Pulse not of right length - start over.
          crcOK = 0;
          RFID.byteCounter = 0;   //reset counter
          RFID.bitCounter = 10;   //reinitialize counter
          tenZ = 0xFFFF;          //restart search for 10 zeros
          pulse2=0;               //second short burst disabled
          break;
        }  
        
     case 3: {   //read completed
        if(crcOK>0) {crcOK=3;}
//        serial.println();
//        for(byte i=0; i<14; i++) {
//            serial.print(RFIDbytes[i], HEX);
//            serial.print(" ");
//        }
//        serial.print("CRC: ");
//        serial.println(crc, HEX);
//        serial.print("CRC OK: ");
//        serial.println(crcOK);
        break;
     }
    }
  }
}


uint16_t crc16k(uint16_t crc1, uint8_t *mem, uint8_t len) { //Calculates CRC (use 0x0000 for crc variable)

    uint8_t *data = mem;
    
    if (data == NULL){
        return 0;
    }
    while (len--) {
        crc1 ^= *data++;
        for (uint8_t k = 0; k < 8; k++)
            crc1 = crc1 & 1 ? (crc1 >> 1) ^ 0x8408 : crc1 >> 1;
    }
    //serial.print(crc1, HEX);
    return crc1;
}



 #endif
