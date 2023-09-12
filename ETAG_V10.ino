/*
  Data logging sketch for the ETAG RFID Reader
  ETAG READER Version 10.0
  PROCESSOR SAMD21JXXX
  USE BOARD DEFINITION: ETAG RFID V2 D21
  Code by:
   Jay Wilhelm
   Tho Trinh
   Eli Bridge
   Alexander Moreno
   David Mitchell

  Sept 2019

  Updates for 2022
  Ignore line returns in menu

  Licenced in the public domain

  REQUIREMENTS:
  Power supply for the board should come from the USB cable or a 5V battery or DV power source.
  A 3.3V CR1025 coin battery should be installed on the back side of the board to maintain the date
      and time when the board is disconnected from a primary power source.

  LIBRARIES:
  The RV3129 library is accessible on github here:
  https://github.com/OUIDEAS/RV-3129_Arduino_Library.git

  FLASH MEMORY STRUCTURE:
  Adesto® AT45DB321E
  34,603,008 bits of memory are organized as 8,192 pages of 528 bytes each.
  23-bit address sequence - The first 13 bits (PA12 - PA0) specify the page.
  The last 10 bits (BA9 - BA0) specify the byte address within the page.
  Data can be written/erased as pages, blocks (8 pages) or sectors (usually 128 pages).
  First sector = pages 0-7; others are much larger
  Pages count up as 0x00000000, 0x00000400, 0x00000800, 0x00000C00, 0x00001000, 0x00001400, 0x00001800, 0x00001C00, 0x00002000
                    page 0      page 1      page 2      page 3      page 4      page 5      page 6      page  7     page 8
  To get page address, take just the page number (1, 2, 3) and shft it 10 bits left.                   
  
  Page 0 is reserved for parameters and counters (Addresses 0x00000400 to ??
    byte 3 - (0x403) Set to 0xAA once the memory address counter is intialized. (important mainly for the very first initialization process with a completely blank Flash memory)
    bytes 4-7 - (0x404-0x407) next four bytes are for the reader ID charaters
    byte 13 = Logging mode - log to SD in real time or just use Flash
  Pages 1-7 are reserved for RFID tag codes. (sector 1a is pages 0-7)
  Pages 8 (0x00 00 20 00) to 127 (all of sector 1b) are for logging and error codes.
  Remaining pages (and sectors) are for RFID data. First address for data storage is page 128 (0x00 02 00 00)

  New memory access system (Mar 2023). Memory location (memLoc) counts up from zero. First memory location for RFID 
  is 4224 (page 8 byte 0) or whatever is specified in datStart. Translation of memLoc to flash address as well as crossing page boundaries 
  is done in readFlash and writeFlash functions. These functions must recieve arrays, but can deal with
  single bytes by setting nchar to 1.

  RFID data lines are stored in a manner that prevents having 0xFF at the beginning or end of a line
  Each line begins with a byte that indicates the antenna number (usually 1 or 2) and the tag type (1=EM4100, 2=ISO11784/5)
  Then there is a VARIABLE NUMBER OF BYTES for the RFID code (5 bytes for EM4100, 8 bytes for ISO)
  Last is the unix timestamp. To ensure that the line does not end with 0xFF we order the bytes for
  the timestamp to make the most significant byte last in the sequence. 
  So, it goes unixTime.b1, unixTime.b2, unixTime.b3, unixTime.b4.

Nov 8, 2019 - Added Memory address lookup - address pointer no longer used. 
Nov 10, 2019 - Added dual logging modes. 
Nov 10, 2019 - Fixed bug in sleepTimer function that stopped the clock.
Dec 15, 2019 - Changed default mode to no-SD, added LED flash when logging starts
July 2022 - revised memory structure to allow for more tag storage.
          - revised memory to compress times using unix format 
          - set up serial input code to ignore line feeds and returns
          - revised data transfer to generate old and new files. 

 Mar 2023 - simplified Flash memory writing - one write command and one read command that automatically writes across pages.
          - Added ISO11784/5 tag reading capability and temperature tags!!!! set global ISO variable to  1 for ISO tags.
          - Progressed toward elimination of string variables (still some strings used with the real time clock)
          - revamped SD card storage functions
          - enabled log files for both SD and Flash logging modes
          - Revamped file storage. New file created for each data transfer.
          
 log events are coded as follows
 1 - "Logging_started"
 2 - "Going_to_sleep "
 3 - "Wake_from_sleep" 

 TO DO: Build in clock error detection??
 
*/


// ***********INITIALIZE INCLUDE FILES AND I/O PINS*******************
#include "RV3129.h"          // include library for the real time clock - must be installed in libraries folder
#include <Wire.h>            // include the standard wire library - used for I2C communication with the clock
#include <SD.h>              // include the standard SD card library
#include <SPI.h>             // include standard SPI library
#include "Manchester.h"


#define serial SerialUSB       // Designate the USB connection as the primary serial comm port - note lowercase "serial"
#define SD_FAT_TYPE         3  // Type 3 Reads all card formats
#define SPI_SPEED          SD_SCK_MHZ(4) .  //Finds fastest speed.
//#define DEMOD_OUT_1      41  // (PB22) this is the target pin for the raw RFID data from RF circuit 1
//#define DEMOD_OUT_2      42  // (PB23) this is the target pin for the raw RFID data from RF circuit 2
//#define SHD_PINA         48  // (PB16) Setting this pin high activates RFID circuit 1
//#define SHD_PINB         49  // (PB17) Setting this pin high activates RFID circuit 2
#define SDselect           46  // (PA21) Chip select for SD card. Make this pin low to activate the SD card for SPI communication
#define SDon               45  // (PA06) Provides power to the SD via a high side mosfet. Make this pin low to power on the SD card
#define FlashCS            44  // (PA05) Chip select for flash memory. Make this pin low to activate the flash memory for SPI communication
#define LED_RFID           43  // (PA27) Pin to control the LED indicator.
#define INT1               47  // (PA20) Clock interrupt for alarms and timers on the RTC
#define MOTR               2   // used for sleep function (need to investigate this). 
RV3129 rtc;   //Initialize an instance for the RV3129 real time clock library.

// ************************* initialize variables******************************                      

uint32_t datStart = 4224;      //initial RFID tag memory address
uint32_t logStart = 528;       //initial log memory address

uint8_t ISO = 0;               //set to 1 to read ISO11874/5 tags, 0 = EM4100 tags

char deviceID[5] = "RFID";            // User defined name of the device                 
String deviceIDstr;                   // String verion of device name.
//char fName[13];                       // Used for writing to SD card
String dataFile;                      // Stores text file name for SD card writing.
String logFile;                       // Stores text file name for SD card writing.

union             //Make a union structure for dealing with unix time conversion
{
  struct
  {
    uint8_t b1;   //first (LSB) byte of unix time 
    uint8_t b2;   //second byte
    uint8_t b3; 
    uint8_t b4;   //most significant byte (should be stored last in flash because it is never 0xFF)
  };
  uint32_t unixLong;   //Name for 32 bit unix time value
} unixTime;            //Name of the union variable (use unixTime.unixLong or unixTime.b3) 

uint32_t unixPast = 0;     // stores the past unix time - for use with delayTime

//String currRFID;                      // stores current RFID number and RF circuit - for use with delayTime
//String pastRFID = "XXXXXXXXXX1";      // stores past RFID number and RF circuit - for use with delayTime

uint32_t currRFID;                      // stores current RFID number and RF circuit - for use with delayTime
uint16_t currRFID2;                      // stores current RFID number and RF circuit - for use with delayTime
uint32_t pastRFID = 0xFFFFFFFF;      // stores past RFID number and RF circuit - for use with delayTime
uint16_t pastRFID2 = 0xFFFF;      // stores past RFID number and RF circuit - for use with delayTime

             
uint32_t memLoc;                     //current memory location.
uint32_t logLoc;                     //current log memory location.

uint32_t flashAddr;
uint32_t logFlashAddr;
uint32_t dataLine;                  
uint32_t logLine;
unsigned int pageAddress;             // page address for flash memory
unsigned int byteAddress;             // byte address for flash memory
unsigned int logPageAddress;          // page address for flash memory log messages
unsigned int logByteAddress;          // byte address for flash memory log messages
unsigned int lastDataAddr = 510;       // last byte address used for writing RFID data (10 bytes per line)
unsigned int lastLogAddr = 520;        // last byte address used for writing RFID data (5 bytes per line)

uint8_t RFcircuit = 1;                 // Used to determine which RFID circuit is active. 1 = primary circuit, 2 = secondary circuit.
uint8_t pastCircuit = 0xFF;            // Used for repeat reads

String currentDate;                   // USed to get the current date in mm/dd/yyyy format (we're weird in the US)
String currentTime;                   // Used to get the time
String currentDateTime;               // Full date and time string
String timeString;                    // String for storing the whole date/time line of data
unsigned int timeIn[12];              // Used for incoming serial data during clock setting
String logMessage;
char logMess[16]; 
byte menu;                            // Keeps track of whether the menu is active.

// Global variable for tag codes

char RFIDstring[10];                  // Stores the TagID as a character array (10 character string)
char ISOstring[14];                   // Country code, period, and 10 characters ("003.03B3AB35D9")
uint16_t RFIDtagUser = 0;             // Stores the first (most significant) byte of a tag ID (user number)
unsigned long RFIDtagNumber = 0;      // Stores bytes 1 through 4 of a tag ID (user number)
uint8_t RFIDtagArray[6];              // Stores the five or 6 (ISO) individual bytes of a tag ID.
uint16_t IDCRC;                        // CRC calculated to determine repeat reads
uint16_t pastCRC;                      // used to determine repeat reads
//uint8_t pastRFID[7];                  // Used for checking current reads against past reads (extra byte for RF circuit).
uint8_t tagTemp;                      // Temperature byte from temperature sensing tags.
uint16_t countryCode;

// Character arrays for text handling 
char cArray1[64];                     // Global general purpose character array - Can store 64 characters
char cArray2[64];                     // Global general purpose character array - Can store 64 characters
char flashData[12];

// ********************CONSTANTS (SET UP LOGGING PARAMETERS HERE!!)*******************************
const byte checkTime = 30;                          // How long in milliseconds to check to see if a tag is present (Tag is only partially read during this time -- This is just a quick way of detirmining if a tag is present or not
const unsigned int pollTime1 = 200;                 // How long in milliseconds to try to read a tag if a tag was initially detected (applies to both RF circuits, but that can be changed)
const unsigned int delayTime = 8;                   // Minimim time in seconds between recording the same tag twice in a row (only applies to data logging--other operations are unaffected)
const unsigned long pauseTime = 100;                // CRITICAL - This determines how long in milliseconds to wait between reading attempts. Make this wait time as long as you can and still maintain functionality (more pauseTime = more power saved)
uint16_t pauseCountDown = pauseTime / 31.25;        // Calculate pauseTime for 32 hertz timer
byte pauseRemainder = ((100*pauseTime)%3125)/100;   // Calculate a delay if the pause period must be accurate
//byte pauseRemainder = 0 ;                         // ...or set it to zero if accuracy does not matter

const byte slpH = 99;                            // When to go to sleep at night - hour
const byte slpM = 00;                            // When to go to sleep at night - minute
const byte wakH = 99;                            // When to wake up in the morning - hour
const byte wakM = 00;                            // When to wake up in the morning - minute
const unsigned int slpTime = slpH * 100 + slpM;  // Combined hours and minutes for sleep time
const unsigned int wakTime = wakH * 100 + wakM;  // Combined hours and minutes for wake time

/* The reader will output Serial data for a certain number of read cycles;
   then it will start using a low power sleep mode during the pauseTime between read attempts.
   The variable stopCycleCount determines how many read cycles to go
   through before using the low-power sleep.
   Once low-power sleep is enabled, the reader will not be able to output
   serial data, but tag reading and data storage will still work.
*/
unsigned int cycleCount = 0;          // counts read cycles
unsigned int stopCycleCount = 500;     // How many read cycles to maintain serial comminications
bool Debug = 1;                       // Use to stop serial messages once sleep mode is used.
byte SDOK = 1;
char logMode;

//////SETUP//////////////SETUP//////////////SETUP//////////////SETUP////////

void setup() {  // setup code goes here, it is run once before anything else

// ********************SET UP INPUT & OUTPUT PINS*******************************
  pinMode(LED_RFID, OUTPUT);      // pin for controlling the on-board LED
  digitalWrite(LED_RFID, HIGH);   // turn the LED off (LOW turns it on)
  pinMode(SDselect, OUTPUT);      // Chip select pin for SD card must be an output
  pinMode(SDon, OUTPUT);          // Chip select pin for SD card must be an output
  pinMode(FlashCS, OUTPUT);       // Chip select pin for Flash memory
  digitalWrite(SDon, LOW);        // turns on the SD card.
  digitalWrite(SDselect, HIGH);   // Make both chip selects high (not selected)
  digitalWrite(FlashCS, HIGH);    // Make both chip selects high (not selected)
  pinMode(SHD_PINA, OUTPUT);      // Make the primary RFID shutdown pin an output.
  digitalWrite(SHD_PINA, HIGH);   // turn the primary RFID circuit off (LOW turns on the EM4095)
  pinMode(SHD_PINB, OUTPUT);      // Make the secondary RFID shutdown pin an output.
  digitalWrite(SHD_PINB, HIGH);   // turn the secondary RFID circuit off (LOW turns on the EM4095)
  pinMode(INT1, INPUT);           // Make the alarm pin an input


// ********************EXECUTED STARTUP CODE *******************************
  blinkLED(LED_RFID, 4, 400);     // blink LED to provide a delay for serial comms to come online
  serial.begin(9600);
  serial.println(); serial.println(); serial.println();

  //Check flash memory and initialize if needed
  readFlash(3, cArray1, 1);   //Read a particular byte from the flash memory; 
  if (cArray1[0] != 0xAA) {  //if the byte is 0xFF then the flash memory needs to be initialized         
    serial.println("Initializing Flash Memory..."); //Message
    cArray1[0] = 0xAA;
    writeFlash(3, cArray1, 1);               //Write a different byte to this memory location
  }

  readFlash(4, deviceID, 4); //Get and display the device ID
  if (deviceID[1] == 0xFF) {  //If this byte is empty set a default deviceID
    serial.println("Setting default device ID - Please update this!!!");
    deviceID[0] = 'R'; deviceID[1] = 'F'; deviceID[2] = '0'; deviceID[3] = '1';
    writeFlash(4, deviceID, 4);  //write to flash memory without updating flash address
  }
  //serial.print("Device ID: "); serial.println(String(deviceID));             //Display device ID

  memLoc = getMemLoc(datStart, 4324848);            //Last page address is 8191, beginning of last page is 528 * 8191 = 4324848
  serial.print("Current RFID memory location: ");
  serial.println(memLoc, DEC);
  logLoc = getMemLoc(logStart, datStart-528);       //Page Range for log lines 
  serial.print("Current log memory location: ");
  serial.println(logLoc, DEC);
  
  readFlash(0x0D, cArray1, 1);  //get the logmode
  logMode = cArray1[0];         // define logmode variable
  if(logMode != 'S' && logMode != 'F') { //If log mode is not established then do so
    serial.println("Setting log mode to Flash mode");
    cArray1[0] = 'F';
    writeFlash(0x0D, cArray1, 1);    
    logMode = 'F';
  }

  deviceIDstr = String(deviceID);
  logFile = String(deviceIDstr +  "LOG.TXT");
  dataFile = String(deviceIDstr + "DATA.TXT");


  // Initialize SD card
  SDOK = 0;
  SDstart();                                             // Enable SD
  File testFile = SD.open("test1234.txt", FILE_WRITE);           // Initialize the SD card and open the file "datalog.txt" or create it if it is not there.
  if(testFile) {  
      SDOK = 1;                                // indicates SD card detected
      serial.println("SD card detected.");     // OK message
      testFile.close();
      SD.remove("test1234.txt");               // delete the dummy file                      
      SDstop();                                // may not be needed.
      appendMemRFID();
      appendMemLog();  
  } else {     
      serial.println("No SD card detected.");       // error message
      if(logMode == 'S') {;
        for(byte i = 0; i < 4; i++) {               // Send out repeated error messages
          serial.println("SD card missing or damaged: logging only to flash memory");  // error message
          blinkLED(LED_RFID, 1, 1000);    // long LED flash for warning
          SDOK = 2;                       // indicates SD card not detected but should have been
        }
      }
  }

  // start clock functions and check time
  Wire.begin();  //Start I2C bus to communicate with clock.
  if (rtc.begin() == false) {  // Try initiation and report if something is wrong
    serial.println("Something wrong with clock");
  } else {
    if(rtc.is12Hour()==true) {rtc.set24Hour();}   //Make sure we are in 24 hour mode??
  }

  doMenu(); 
   
  RFcircuit = 1;
  blinkLED(LED_RFID, 3,100);
}

////////////////////////////////////////////////////////////////////////////////
////////MAIN////////////////MAIN////////////////MAIN////////////////MAIN////////
////////////////////////////////////////////////////////////////////////////////

void loop() { // Main code is here, it loops forever:

///////Check Sleep//////////////Check Sleep///////////
//Check to see if it is time to execute nightime sleep mode.

  rtc.updateTime();                                            // Get an update from the real time clock
  
  if(Debug) serial.println(showTime());                        // Show the current time 
  int curTimeHHMM = rtc.getHours() * 100 + rtc.getMinutes();   // Combine hours and minutes into one variable                     
  if (curTimeHHMM == slpTime) {                                // Check to see if it is sleep time
     //String SlpStr =  showTime() + " Go_to_sleep";             // if it's time to sleep make a log message
     if(Debug) {
        serial.println("Going to sleep at ");                                // print log message
        serial.println(showTime());
     }
     unixTime.unixLong = getUnix();
     //serial.println(unixTime.unixLong, DEC);
     char lg[5] = {12, unixTime.b1, unixTime.b2, unixTime.b3, unixTime.b4};
     logLoc = writeFlash(logLoc, lg, 5);
     if(SDOK == 1 && logMode == 'S') {writeSDLine(logFile, 12, lg);}           // save log message if SD writes are enabled
     sleepAlarm();                                             // sleep using clock alarm for wakeup
     rtc.updateTime();                                         // get time from clock
     //SlpStr =  "Wake up from sleep mode at " + showTime();     // log message
     unixTime.unixLong = getUnix();
     //serial.println(unixTime.unixLong, DEC);
     lg[0]=13; lg[1]=unixTime.b1; lg[2]=unixTime.b2; lg[3]=unixTime.b3; lg[4]=unixTime.b4;
     logLoc = writeFlash(logLoc, lg, 5);
     if(SDOK == 1 && logMode == 'S') {writeSDLine(logFile, 13, lg);}           // save log message if SD writes are enabled
  }

//////Read Tags//////////////Read Tags//////////
//Try to read tags - if a tag is read and it is not a recent repeat, write the data to the SD card and the backup memory.

  bool readSuc = 0; 
  uint32_t oldMem;
  String SDsaveString; 
  if(ISO==1) { readSuc = ISOFastRead(RFcircuit, checkTime, pollTime1); } 
  if(ISO==0) { readSuc = FastRead(RFcircuit, checkTime, pollTime1); }

  if (readSuc == 1) {
    rtc.updateTime(); 
    if(ISO==0) {
      processTag(RFIDtagArray, RFIDstring, RFIDtagUser, &RFIDtagNumber);            // Parse tag data into string and hexidecimal formats
      sprintf(cArray1, "%02X%02X%02X%02X%02X, %d, %02d/%02d/%04d %02d:%02d:%02d",
        RFIDtagArray[0], RFIDtagArray[1], RFIDtagArray[2], RFIDtagArray[3], RFIDtagArray[4],  
        RFcircuit, rtc.getMonth(), rtc.getDate(), rtc.getYear()+2000,
        rtc.getHours(), rtc.getMinutes(), rtc.getSeconds());
      cArray1[34] = '\0';
    }                 
    if(ISO==1) {
      processISOTag(RFIDtagArray, RFIDstring, &countryCode, &tagTemp, &RFIDtagNumber);
      countryCode = (RFIDtagArray[5]<<2) + (RFIDtagArray[4]>>6);
      sprintf(cArray1, "%03X.%02X%02X%02X%02X%02X, %03d, %d, %02d/%02d/%04d %02d:%02d:%02d",
                 countryCode, (RFIDtagArray[4] & 0b00111111), RFIDtagArray[3], RFIDtagArray[2], RFIDtagArray[1], RFIDtagArray[0], 
                 tagTemp, RFcircuit, rtc.getMonth(), rtc.getDate(), rtc.getYear()+2000,
                 rtc.getHours(), rtc.getMinutes(), rtc.getSeconds());
      cArray1[43] = '\0';
    }
    currRFID = (RFIDtagArray[0]<<24) + (RFIDtagArray[1]<<16) + (RFIDtagArray[2]<<8) + (RFIDtagArray[3]);   //Put RFID code and Circuit into two variable to identify repeats
    currRFID2 = (RFIDtagArray[4]<<8 + RFcircuit);                                                          //Put RFID code and Circuit into two variable to identify repeats
    unixTime.unixLong = getUnix();                      //Update unix time value to identify repeat reads and employ delay time.
    if((currRFID != pastRFID) | (currRFID2 != pastRFID2) | (unixTime.unixLong-unixPast >= delayTime)) {                      // See if the tag read is a recent repeat
      oldMem = memLoc;
      if(ISO==0) {
          flashData[0]=RFcircuit; 
          flashData[1]=RFIDtagArray[0]; flashData[2]=RFIDtagArray[1]; flashData[3]=RFIDtagArray[2]; flashData[4]=RFIDtagArray[3]; flashData[5]=RFIDtagArray[4];      // Create an array representing an entire line of data
          flashData[6]=unixTime.b1; flashData[7]=unixTime.b2; flashData[8]=unixTime.b3; flashData[9]=unixTime.b4;
          memLoc = writeFlash(memLoc, flashData, 10);   //write array  
      }
      if(ISO==1) {
          flashData[0]=RFcircuit + 0b10000000; //Flag very first bit to denote ISO code 
          flashData[1]=RFIDtagArray[0]; flashData[2]=RFIDtagArray[1]; flashData[3]=RFIDtagArray[2]; flashData[4]=RFIDtagArray[3]; flashData[5]=RFIDtagArray[4]; flashData[6]=RFIDtagArray[5];     // Create an array representing an entire line of data
          flashData[7]=tagTemp;
          flashData[8]=unixTime.b1; flashData[9]=unixTime.b2; flashData[10]=unixTime.b3; flashData[11]=unixTime.b4;
          memLoc = writeFlash(memLoc, flashData, 12);   //write array  
      }
      if(SDOK == 1 & logMode == 'S') {
         if(Debug) {serial.println("Storing on SD card and flash memory.");}
         writeSDLine(dataFile, 0, cArray1);
      }

     pastRFID = currRFID;            //First of three things to identify repeat reads
     pastRFID2 = currRFID2;          //Second of three things to identify repeat reads
     unixPast = unixTime.unixLong;   //Third  of three things to identify repeat reads 
     if(Debug) {
        //serial.print(SDsaveString); 
        serial.print(cArray1);
        serial.print(" logged to flash address "); 
        serial.println(oldMem);
      }
    } else {
      if(Debug) serial.println("Repeat - Data not logged");                             // Message to indicate repeats (no data logged)
    }
    blinkLED(LED_RFID, 2,5);
 }

//////////Pause//////////////////Pause//////////
//After each read attempt execute a pause using either a simple delay or low power sleep mode.

  if(cycleCount < stopCycleCount){   // Pause between read attempts with delay or a sleep timer 
    if(serial.available()) {
      byte C1 = serial.read();                          // read input from the user
      if(C1 == 'm') {                                     // Do nothing with character that are not numbers or letter (ignore line returns and such)
      doMenu();
    }
    }
    delay(pauseTime);                // Use a simple delay and keep USB communication working
    cycleCount++ ;                   // Advance the counter                         
  }else{
    sleepTimer(pauseCountDown, pauseRemainder);
  }

//Alternate between circuits (comment out to stay on one cicuit).
   RFcircuit == 1 ? RFcircuit = 2 : RFcircuit = 1; //if-else statement to alternate between RFID circuits

}

  



/////////////////////////////////////////////////////////////////////////////
/////FUNCTIONS/////////FUNCTIONS/////////FUNCTIONS/////////FUNCTIONS/////////
/////////////////////////////////////////////////////////////////////////////

//repeated LED blinking function
void blinkLED(uint8_t ledPin, uint8_t repeats, uint16_t duration) { //Flash an LED or toggle a pin
  pinMode(ledPin, OUTPUT);             // make pin an output
  for (int i = 0; i < repeats; i++) {  // loop to flash LED x number of times
    digitalWrite(ledPin, LOW);         // turn the LED on (LOW turns it on)
    delay(duration);                   // pause again
    digitalWrite(ledPin, HIGH);        // turn the LED off (HIGH turns it off)
    delay(duration);                   // pause for a while
  }                                    // end loop
}                                      // End function


  //////////////MENU////////////MENU////////////MENU////////////MENU////////
  //Display all of the following each time the main menu is called up.
  void doMenu() {
    byte menu = 1;
    while (menu == 1) {
      serial.println();
      rtc.updateTime();
      serial.println(showTime());          
      serial.println("Device ID: " + String(deviceID)); //Display device ID
      if(logMode == 'S') {serial.println("Logging mode: S (Data saved to SD card and Flash Mem)");}
      if(logMode == 'F') {
        serial.println("Logging mode: F (Data saved to flash mem only; no SD card logging)");
        //SDOK = 0;     //Turns off SD logging
      }
      serial.println();
  
      // Ask the user for instruction and display the options
      serial.println("What to do? (input capital letters)");
      serial.println("  C = set clock");
      serial.println("  B = Display backup memory and log history");
  //    serial.println("  D = Display logging history");
      serial.println("  E = Erase (reset) backup memory");
      serial.println("  I = Set device ID");
      serial.println("  M = Change logging mode");
      serial.println("  W = Write ALL flash data to SD card (includes duplicates)");
  
      //Get input from user or wait for timeout
      char incomingByte = getInputByte(15000); 
      String printThis = String("Value recieved: ") + incomingByte;
      serial.println(printThis);
      //serial.println(incomingByte, DEC);
      if(incomingByte < 47) {    //Ignore punctuation and line returns and such.
        incomingByte = 'X';
      } 
      switch (incomingByte) {                                               // execute whatever option the user selected
        default:
          menu = 0;         //Any non-listed entries - set menu to 0 and break from switch function. Move on to logging data
          break;
        case 'C': {                                                         // option to set clock
            inputTime();                                                    // calls function to get time values from user
            break;                                                        //  break out of this option, menu variable still equals 1 so the menu will display again
          }
        case 'I': {
            inputID(4);   //  calls function to get a new feeder ID
            break;       //  break out of this option, menu variable still equals 1 so the menu will display again
          }
        case 'E': {
              serial.println("To proceed enter 'ERASE' in capital letters");  // Ask for user input
              byte IDin = getInputString(10000);                  // Get input from user (specify timeout)
              if((cArray1[0]=='E') && (cArray1[1]=='R') && (cArray1[2]=='A') && (cArray1[3]=='S') && (cArray1[4]=='E')) {                             // if the string is the right length...
                eraseBackup('m');
              }
   
              break;   //  break out of this option, menu variable still equals 1 so the menu will display again
          }
        case 'B': {
            extractMemRFID(1, datStart);
            extractMemLog(1, logStart);
            break;       //  break out of this option, menu variable still equals 1 so the menu will display again
          }
  //      case 'D': {
  //          writeMemLog();
  //          break;       //  break out of this option, menu variable still equals 1 so the menu will display again
  //        }
        case 'M': {
            readFlash(0x0D, cArray1, 1);  //get the logmode
            logMode = cArray1[0];
            if(logMode != 'S') {
              cArray1[0] = 'S';
              writeFlash(0x0D, cArray1, 1);
              serial.println("Logging mode S");
              serial.println("Data saved immediately SD card and Flash Mem");
              if(SDOK == 0) {SDOK = 1;}
            } else {
              cArray1[0] = 'F';
              writeFlash(0x0D, cArray1, 1);
              serial.println("Logging mode F");
              serial.println("Data saved to Flash Mem only (no SD card needed)");
              SDOK = 0;
            }
            readFlash(0x0D, cArray1, 1);  //get the logmode
            logMode = cArray1[0];
            serial.println("log mode variable: ");
            serial.println(logMode);
            break;       //  break out of this option, menu variable still equals 1 so the menu will display again
          }  
          case 'W': {
            if (SDOK == 1) {
              extractMemRFID(3, datStart); //write RFID data to SD card
              extractMemLog(3, logStart);  //write Log data to SD (always do this second so the file names match up).
            } else {
              serial.println("SD card missing");
            }
            break;       //  break out of this option, menu variable still equals 1 so the menu will display again
          }  
        } //end of switch
    } //end of while(menu = 1)
    rtc.updateTime();
    unixTime.unixLong = getUnix();
    char logInfo[5] = {11, unixTime.b1, unixTime.b2, unixTime.b3, unixTime.b4};
    serial.print("writing start info to log at "); serial.println(logLoc);
    logLoc = writeFlash(logLoc, logInfo, 5);
    if(SDOK == 1 && logMode== 'S') {
      serial.println("Writing start up to SD log file");
      writeSDLine(logFile, 11, logInfo);     //log to SD card file
    }
}


//Recieve a byte (charaacter) of data from the user- this times out if nothing is entered
char getInputByte(uint32_t timeOut) {                 // Get a single character
  char readChar = '?';                                // Variable for reading in character
  uint32_t sDel = 0;                                  // Counter for timing how long to wait for input
  while (serial.available() == 0 && sDel < timeOut) { // Wait for a user response
    delay(1);                                         // Delay 1 ms
    sDel++;                                           // Add to the delay count
  }
  while (serial.available()) {                        // If there is a response then perform the corresponding operation
    byte R1 = serial.read();                          // read input from the user
    if(R1 > 47) {                                     // Do nothing with character that are not numbers or letter (ignore line returns and such)
      readChar = R1;
    }
  }
  return readChar;                                    //Return the value. It will be "?" if nothing is recieved.
}

//Recieve a character array of data from the user- this times out if nothing is entered
byte getInputString(uint32_t timeOut) {               // Get a character array from the user and put in global array variale. Return the number of bytes read in
  uint32_t sDel = 0;                                  // Counter for timing how long to wait for input
  byte charCnt = 0;
  while (serial.available() == 0 && sDel < timeOut) { // Wait for a user response
    delay(1);                                         // Delay 1 ms
    sDel++;                                           // Add to the delay count
  }
  if (serial.available()) {                           // If there is a response then read in the data
    delay(40);                                        // long delay to let all the data sink into the buffer
    while (serial.available()) {
      byte R1 = serial.read();                          // read the entry from the user
      if(R1 > 47) {
        serial.println(R1, DEC);
        cArray1[charCnt] = R1;                          // ignore punctuation, line returns, etc
        charCnt++;
      }
    }
  }
//  serial.println("char count is ");
//  serial.println(charCnt);
//  serial.println("array is ");
//  for (int i = 0; i <= charCnt; i++) {
//    serial.print(cArray1[i]);
//  }
//  serial.println();
  return charCnt;                                     //Return the number of characters recieved. It will be zero if nothing is read in
  }


//CLOCK FUNCTIONS///////////

//Get date and time strings and cobine into one string
String showTime() { //Get date and time strings and cobine into one string
  currentDateTime = String(rtc.stringDateUSA()) + " " + String(rtc.stringTime()); 
  return currentDateTime;
}

void showTimeArray(char *TA) {
   sprintf(TA, "%02d/%02d/%04d %02d:%02d:%02d",
       rtc.getMonth(), rtc.getDate(), rtc.getYear()+2000,
       rtc.getHours(), rtc.getMinutes(), rtc.getSeconds() );
   TA[19] = '\0';
}


//Function to get user data for setting the clock
void inputTime() {                               // Function to set the clock
  serial.println("Enter mmddyyhhmmss");          // Ask for user input
  byte timeInput = getInputString(20000);        // Get a string of data and supply time out value
  if (timeInput == 12) {                  // If the input string is the right length, then process it
    serial.println("time read in");                   // Show the string as entered  
    byte mo = (cArray1[0]-48) * 10 + (cArray1[1] - 48);  //Convert two ascii characters into a single decimal number
    byte da = (cArray1[2]-48) * 10 + (cArray1[3] - 48);  //Convert two ascii characters into a single decimal number
    byte yr = (cArray1[4]-48) * 10 + (cArray1[5] - 48);  //Convert two ascii characters into a single decimal number
    byte hh = (cArray1[6]-48) * 10 + (cArray1[7] - 48);  //Convert two ascii characters into a single decimal number
    byte mm = (cArray1[8]-48) * 10 + (cArray1[9] - 48);  //Convert two ascii characters into a single decimal number
    byte ss = (cArray1[10]-48) * 10 + (cArray1[11] - 48);  //Convert two ascii characters into a single decimal number
    if (rtc.setTime(ss, mm, hh, da, mo, yr + 2000, 1) == false) {     // attempt to set clock with input values
      serial.println("Something went wrong setting the time");        // error message
    }
  } else {
    serial.println("Time entry error");           // error message if string is the wrong lenth
  }
}

// sleep and wake up using the alarm  on the real time clock 
void sleepAlarm(){                            // sleep and wake up using the alarm  on the real time clock 
   rtc.setAlarm(0, wakM, wakH, 1, 1, 1, 19);  // set alarm: sec, min, hr, date, mo, wkday, yr (only min and hr matter)
   rtc.writeRegister(0x02, 0);                // write a zero to the flags register to clear all flags.
   rtc.enableDisableAlarm(B00000111);         // Enable daily alarm - responds to hour, minute, and second (set last three enable bits)
   rtc.enableAlarmINT(1);                     // Enable the clock interrupt output.
   lpSleep();                                 // sleep funciton - sleep until alarm interrupt
   blinkLED(LED_RFID, 5, 200);                // blink to indicate wakeup
   rtc.writeRegister(0x02, 0);                // clear clock flags to turn off alarm.
}

// Sleep and wake up using a 32-hertz timer on the real time clock 
void sleepTimer(uint16_t pCount, byte pRemainder){ // Sleep and wake up using a 32-hertz timer on the real time clock 
   rtc.writeRegister(0x02, 0);       // write a zero to the flags register to clear all flags.
   rtc.setTimer(pCount);             // set timer countdown (32-hertz timer) 
   rtc.enableTimerINT(1);            // enable the clock interrupt output
   rtc.setCTRL1Register(B10011011);  // set control register to enable a 32 Hertz timer.
   lpSleep();                        // call sleep funciton (you lose USB communicaiton here)
   //blinkLED(LED_RFID, 1, 30);     // blink indicator - processor reawakened
   delay(pRemainder);                // additional delay for accuracy
   rtc.enableTimerINT(0);            // disable the clock interrupt output
   rtc.writeRegister(0x02, 0);       // write a zero to clear all interrupt flags.
}
    
uint32_t getUnix() {
    int8_t my = (rtc.getMonth() >= 3) ? 1 : 0;
    uint16_t y = (rtc.getYear()+2000) + my - 1970;
    uint16_t dm = 0;
    for (int i = 0; i < rtc.getMonth() - 1; i++) dm += (i<7)?((i==1)?28:((i&1)?30:31)):((i&1)?31:30);
    return (((rtc.getDate()-1+dm+((y+1)>>2)-((y+69)/100)+((y+369)/100/4)+365*(y-my))*24ul+rtc.getHours())*60ul+rtc.getMinutes())*60ul+rtc.getSeconds();
}

uint32_t getUnix2(byte yr, byte mo, byte da, byte hh, byte mm, byte ss) {
    int8_t my = (mo >= 3) ? 1 : 0;
    uint16_t y = (yr+2000) + my - 1970;
    uint16_t dm = 0;
    for (int i = 0; i < mo - 1; i++) dm += (i<7)?((i==1)?28:((i&1)?30:31)):((i&1)?31:30);
    return (((da-1+dm+((y+1)>>2)-((y+69)/100)+((y+369)/100/4)+365*(y-my))*24ul+hh)*60ul+mm)*60ul+ss;
}

 void convertUnix(uint32_t t) { //Takes a unix number and stores various time values to timeIn[12]
        t += 0 * 3600ul;
        timeIn[5] = t % 60ul;   //Seconds stored in timeIn[5]
        t /= 60ul;
        timeIn[4] = t % 60ul;  //Minutes stored to timeIn[4]
        t /= 60ul;
        timeIn[3] = t % 24ul;  //Hours stored to timeIn[3]
        t /= 24ul;
        byte dayOfWeek = (t + 4) % 7;
        if (!dayOfWeek) dayOfWeek = 7;
        uint32_t z = t + 719468;
        uint8_t era = z / 146097ul;
        uint16_t doe = z - era * 146097ul;
        uint16_t yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
        uint16_t y = yoe + era * 400;
        uint16_t doy = doe - (yoe * 365 + yoe / 4 - yoe / 100);
        uint16_t mp = (doy * 5 + 2) / 153;
        timeIn[1] = doy - (mp * 153 + 2) / 5 + 1;  //Day stored to timeIn[1]
        uint8_t month = mp + (mp < 10 ? 3 : -9);            
        timeIn[0] = month;  //Month stored to timeIn[0]
        y += (month <= 2);
        timeIn[2] = y; //Year stored to timeIn[2]
    }




////FLASH MEMORY FUNCTIONS////////////////////

//Enable the flash chip
void flashOn(void) {              // Enable the flash chip
  pinMode(FlashCS, OUTPUT);       // Chip select pin for Flash memory set to output
  digitalWrite(SDselect, HIGH);   // make sure the SD card select is off
  digitalWrite(FlashCS, LOW);     // activate Flash memory
  SPI.begin();                    // Enable SPI communication for Flash Memory
  SPI.setClockDivider(SPI_CLOCK_DIV16); // slow down the SPI for noise and signal quality reasons.
}

//Disable the flash chip
void flashOff(void) {           // Disable the flash chip
  SPI.end();                    // turn off SPI
  digitalWrite(FlashCS, HIGH);  // deactivate Flash memory
}

uint32_t writeFlash(unsigned long fLoc, char *cArr, uint16_t nchar) {  //write bytes in array; must send array, but can read single byte if nchar=1.
  uint16_t addr = fLoc%528;                //calculate byte address
  uint32_t wAddr = ((fLoc/528)<<10) + addr;  //calculate full flash address 
  flashOn();                               // activate flash chip
  SPI.transfer(0x58);                  // opcode for read modify write
  SPI.transfer((wAddr >> 16) & 0xFF);  // first of three address bytes
  SPI.transfer((wAddr >> 8) & 0xFF);   // second address byte
  SPI.transfer(wAddr & 0xFF);          // third address byte
  if(nchar + addr <= 527) {            // If a page overflow will not happen write all the bytes
    for (int n = 0; n < nchar; n++) {  // loop through the bytes
      //serial.println(cArr[n], DEC);
      SPI.transfer(cArr[n]);           // write the byte
    }
  }
  if(nchar + addr > 527) {                    // If a page overflow will happen write as many bytes as possible
     //serial.println("page cross write");
     for (int n = 0; n <= 527-addr; n++) {    // loop through the bytes
        SPI.transfer(cArr[n]); 
//        serial.println(cArr[n]);
        }
     flashOff();                            // turn off SPI
     delay(30);                              // delay, In datasheet 4ms needed to complete write - in practice about 15, sometimes longer.
     //serial.println("crossing page boundary");
     wAddr = ((fLoc/528)+1)<<10;            // calculate new flash address by advancing the page and setting byte address to 0
     flashOn();
     SPI.transfer(0x58);                  // opcode for read modify write
     SPI.transfer((wAddr >> 16) & 0xFF);  // first of three address bytes
     SPI.transfer((wAddr >> 8) & 0xFF);   // second address byte
     SPI.transfer(wAddr & 0xFF);          // third address byte
     for (int n = 528-addr; n < nchar; n++) {    // loop through the rest of the bytes
        //serial.println(cArr[n]);
        SPI.transfer(cArr[n]);           // write bytes
        }         
  }
  flashOff();                            // turn off SPI
  delay(20);
  return fLoc + nchar;                   // calculate next flash location
}

void readFlash(uint32_t fLoc, char *carr, uint16_t nchar) {  // read one or more bytes from flash memory; must send array, but can read single bytes
  uint16_t addr = fLoc%528;                  //calculate byte address
  uint32_t wAddr = ((fLoc/528)<<10) + addr;  //calculate full flash address
  flashOn();                                 // activate flash chip
  SPI.transfer(0x03);                        // opcode for low freq read
  SPI.transfer((wAddr >> 16) & 0xFF);        // first of three address bytes
  SPI.transfer((wAddr >> 8) & 0xFF);         // second address byte
  SPI.transfer(wAddr & 0xFF);                // third address byte
  //  if(nchar==1){carr[0] = SPI.transfer(0);}
  if(nchar + addr <= 527) {                 // If a page overflow will not happen read all the bytes
    for (int n = 0; n < nchar; n++) {
      carr[n] = SPI.transfer(0);            // read the byte
      //serial.println(carr[n]);
      }       
  }
  if(nchar + addr > 527) {                    // If a page overflow will happen read as many bytes as possible
    //serial.println("Page cross read.");
    for (int n = 0; n <= 527-addr; n++) {
      carr[n] = SPI.transfer(0);
      //serial.println(carr[n]); 
      }
    //serial.println("crossing page boundary");
    flashOff();                            // turn off SPI
    delay(15);
    wAddr = ((fLoc/528)+1)<<10;            // calculate new flash address by advancing the page and leaving byte address at 0
    flashOn();
    SPI.transfer(0x03);                  // opcode for read modify write
    SPI.transfer((wAddr >> 16) & 0xFF);  // first of three address bytes
    SPI.transfer((wAddr >> 8) & 0xFF);   // second address byte
    SPI.transfer(wAddr & 0xFF);          // third address byte
    for (int n = 528-addr; n < nchar; n++) {    // loop through the rest of the bytes
        carr[n] = SPI.transfer(0);       // read byte
        //serial.println(carr[n]);
    }                                      
  }
flashOff(); 
}

// Input the device ID and write it to flash memory
void inputID(uint32_t writeAddr) {                                         // Function to input and set feeder ID
  serial.println("Enter four alphanumeric characters");  // Ask for user input
  byte IDin = getInputString(10000);                  // Get input from user (specify timeout)
  if (IDin == 4) {                             // if the string is the right length...
    deviceID[0] = cArray1[0];                      // Parse the bytes in the string into a char array
    deviceID[1] = cArray1[1];
    deviceID[2] = cArray1[2];
    deviceID[3] = cArray1[3];
    writeFlash(writeAddr, deviceID, 4);                 // Write the array to flash
  } else {
    serial.println("Invalid ID entered");                // error message if the string is the wrong lenth
  }
}

void getLogMessage(uint8_t x1) {
  //serial.print(x1);
  // "Logging_started" = 11
  // "Going_to_sleep_" = 12
  // "Wake_from_sleep" = 13
  
  if(x1 == 11) {
    logMess[0]='L'; logMess[1]='o'; logMess[2]='g'; logMess[3]='g'; logMess[4]='i'; 
    logMess[5]='n'; logMess[6]='g'; logMess[7]='_'; logMess[8]='s'; logMess[9]='t'; 
    logMess[10]='a'; logMess[11]='r'; logMess[12]='t'; logMess[13]='e'; logMess[14]='d';
    logMess[15]='\0';}
  if(x1 == 12) {
    logMess[0]='G'; logMess[1]='o'; logMess[2]='i'; logMess[3]='n'; logMess[4]='g'; 
    logMess[5]='_'; logMess[6]='t'; logMess[7]='o'; logMess[8]='_'; logMess[9]='s'; 
    logMess[10]='l'; logMess[11]='e'; logMess[12]='e'; logMess[13]='p'; logMess[14]='_';
    logMess[15]='\0';}
  if(x1 == 13) {
    logMess[0]='W'; logMess[1]='a'; logMess[2]='k'; logMess[3]='e'; logMess[4]='_'; 
    logMess[5]='f'; logMess[6]='r'; logMess[7]='o'; logMess[8]='m'; logMess[9]='_'; 
    logMess[10]='s'; logMess[11]='l'; logMess[12]='e'; logMess[13]='e'; logMess[14]='p';
    logMess[15]='\0';}
}



void eraseBackup(char eMode) {  //erase chip and replace stored info
  if(eMode == 'a') {   //Erase absolutely everything
    serial.println("This will take 80 seconds");
    flashOn();
    SPI.transfer(0xC7);  // opcode for chip erase: C7h, 94h, 80h, and 9Ah
    SPI.transfer(0x94);    
    SPI.transfer(0x80);    
    SPI.transfer(0x9A);             
    digitalWrite(FlashCS, HIGH);    //Deassert cs for process to start 
    delay(80000);
    flashOff();                         // Turn off SPI
    serial.println("DONE!");
    serial.println("You must now reestablish all parameters");
    return;
  } 
  if(eMode == 'm') {  //Erase tag data only using memLoc as limit
    //uint16_t startPage = 8; 
    uint16_t endPage = (memLoc / 528) + 1;
    for(uint16_t i=1; i <= endPage; i++) {
      uint32_t pg = i << 10;
      serial.print("Erasing Page ");
      serial.println(i, DEC);
      flashOn();
      SPI.transfer(0x81);                    // opcode for page erase: 0x81
      SPI.transfer((pg >> 16) & 0xFF);    // first of three address bytes
      SPI.transfer((pg >> 8) & 0xFF);     // second address byte
      SPI.transfer(pg & 0xFF);            // third address byte
      digitalWrite(FlashCS, HIGH);           // Deassert cs for process to start 
      delay(200);                             // delay for page erase
      flashOff();
    }
    memLoc=datStart;     // reset memory addresses
    logLoc=logStart;     // reset memory addresses
  }
}

void showFlash (uint32_t fStart, uint32_t fEnd) {  
  serial.println("Printing full flash memory");
  char flashArr[12];
  uint32_t jc;
  while(fStart < fEnd) {
     jc = 10;
     serial.print(fStart, DEC); serial.print("   ");
     readFlash(fStart, flashArr, 12);
     for(uint8_t i=0; i < 10; i++) {
        serial.print(flashArr[i], HEX); serial.print(" ");
     }
     if(flashArr[0] == 129) {
      for(uint8_t i=10; i < 12; i++) {
          serial.print(flashArr[i], HEX); serial.print(" ");
       }
       jc = 12;
     }
     serial.println();
     fStart = fStart + jc;
  } 
}


void appendMemLog() { //// Read in last log line on SD card. Find matching line in Flash, Write remaining data to SD.
  char SDArray[37];   // Array with 250 bytes
  for(uint8_t i = 0; i < 250; i++) {SDArray[i] = 0xFF;} //initialize array.
  char logLine[5];     //
  char flashArr[500]; //large array for reading flash data.
  uint32_t startPos = 0;    // start position for matching lines.
  uint32_t posSD = 0;  // SD card file position
  uint32_t fLen;       // length of SD card file
  uint32_t fLoc;       // flash data location
  
  File myfile;     // for reading from SD

  //First check make sure flash log has at least one line of data
  if((logLoc - logStart) < 3) {
    serial.println("No log data to transfer");
    return;
  }

  //Get last line of SD file  
  if(SDOK == 1) {
    SDstart();
    if (!SD.exists(logFile)) {
      serial.println("No log file detected on SD card, need to make new SD file");
      extractMemLog(3, logStart);  //dump all data to sd card here....
      return;
    }
    if (SD.exists(logFile)) {
      myfile = SD.open(logFile, FILE_READ);
      fLen = myfile.size();
      if(fLen < 38) { //no data or corrupt data in file, erase file and write all flash memory
        myfile.close();               //Close file 
        SD.remove(logFile);           //Delete the file that may have bad data
        extractMemRFID(3, logStart);  //Write all flash data to new file
        return;
      }
      if((fLen > 35) && fLen < 75) { posSD = 0; }  // only one line. SD card file position for first line
      if(fLen >= 75) { posSD = fLen - 38; }
      //serial.print("file length: "); serial.println(fLen , DEC);
      //serial.print("getting last line from: "); serial.println(posSD, DEC);
      myfile.seek(posSD);

      for(uint8_t i = 0; i < 37; i++) {
        SDArray[i] = myfile.read();
        }      
      compressLogLine(SDArray, logLine);


      //now seek to match logLine with data in flash
      fLoc = logStart;                 //Start at beginning of log data
      //serial.print("fLoc: "); serial.println(fLoc, DEC);
      uint16_t fA1 = 0;
      while((fLoc < logLoc) && (startPos == 0)) {
        //serial.println("Reading flash");
        readFlash(fLoc, flashArr, 500);
        for(fA1 = 0; fA1 < 495; fA1 = fA1 + 5) {
           //serial.print("Matching "); serial.print(flashArr[fA1], HEX); serial.print(flashArr[fA1+1], HEX); serial.print(flashArr[fA1+2], HEX); serial.print(flashArr[fA1+3], HEX); serial.println(flashArr[fA1+4], HEX);
           //serial.print("with "); serial.print(logLine[0], HEX); serial.print(logLine[1], HEX); serial.print(logLine[2], HEX); serial.print(logLine[3], HEX); serial.println(logLine[4], HEX);
           if((logLine[0]==flashArr[fA1]) && (logLine[1]==flashArr[fA1+1]) && (logLine[2]==flashArr[fA1+2]) && (logLine[3]==flashArr[fA1+3]) && (logLine[4]==flashArr[fA1+4])) {
              startPos = fLoc + fA1 + 5;  //Add five to get to next line.
              serial.print("Matching log data found on SD card. ");
              if(startPos >= logLoc) {
                serial.println("Log Data up to date, no data transfer needed.");
                return; 
              }
              serial.print("Appending new data starting at "); serial.println(startPos, DEC);
              extractMemLog(3, startPos);
              return;
           }
           if(flashArr[fA1]==0xFF) {
              serial.print("end of flash data - no match - append everything."); 
              extractMemLog(3, logStart);
              return;
           }
           
        }
        fLoc = fLoc + fA1;
        //serial.print("fLoc is now "); serial.println(fLoc, DEC);
      }
    }
  }
}



void appendMemRFID() { // Read in last line from SD card. Find matching line in Flash, Write remaining data to SD.
  //serial.println("Appending RFID data to SD card.");
  char SDArray[50];   // Array with 250 bytes
  for(uint8_t i = 0; i < 250; i++) {SDArray[i] = 0xFF;} //initialize array.
  char flashArr[500]; //large array for reading flash data.
  char SDline[50]; //array used for Flash data matching
  uint8_t startPos;    // start position for SD array compression/processing
  uint8_t stopPos;    // start position for SD array compression/processing
  uint32_t posSD = 0;  // SD card file position
  uint32_t fLen;       // length of SD card file
  uint32_t fLoc;
  File myfile;     // for reading from SD

  //First check make sure flash has at least one line of data
  if((memLoc - datStart) < 9) {
    serial.println("No RFID data to transfer");
    return;
  }

  //Get last line of SD file  
  if(SDOK == 1) {
    //serial.print("Reading last line from SD card file: "); serial.println(dataFile);
        SDstart();
    if (!SD.exists(dataFile)) {
      serial.println("No RFID file detected on SD card, need to make new sd file");
      extractMemRFID(3, datStart);  //dump all data to sd card here....
      return;
    }
    if (SD.exists(dataFile)) {
      myfile = SD.open(dataFile, FILE_READ);
      fLen = myfile.size();
      if(fLen < 36) { //no data or corrupt data in file, erase file and write all flash memory
        myfile.close();         //Close file 
        SD.remove(dataFile);    //Delete the file that may have bad data
        extractMemRFID(3, datStart);
        return;
      }
      
      fLen < 50 ? posSD = 0 : posSD = fLen-50;
      //serial.print("reading characters starting at number "); serial.println(posSD, DEC);
      myfile.seek(posSD);   //go to read start position
      uint8_t lineLen = 35;     // 1 indicates ISO line, 0 indicated EM4100 line
      
      for(uint8_t i = 0; i < 50; i++) {
        uint8_t SB1 = myfile.read();
        if(posSD == 0) {SDArray[i] = SB1;}
        if((posSD == 0) & (i == fLen)) {
          lineLen = fLen - 1; 
          break;
        }
        if(posSD > 0) {
          if(SB1 == 13) {  //look for carriage return
            if(i < 10) {lineLen = 44;} // if i is less than 10 then the data line must be longer than 40 
            SB1 = myfile.read(); //Read but ignore - should be line feed
            for(uint8_t j = 0; j < lineLen; j++) {
               SDArray[j] = myfile.read();
            }    
          }
        }
      }

      uint8_t SDLineBytes = compressSDLine(SDArray, SDline, lineLen);
//      serial.println(); serial.println();

      //Loop through flash data to find the matching line.
      uint8_t BX = SDline[6];                   //default match for low byte of timestamp is 6th byth of byte array
      if(SDLineBytes == 12) {BX = SDline[8];}   //change to 8th byte for ISO tags
      fLoc = datStart;                          //Start at beginning of flash data
      while(fLoc < memLoc) {
        uint16_t fA1 = 0;
        readFlash(fLoc, flashArr, 500);
        while(fA1 < 485) {
          //serial.print("evaluating flash data at "); serial.println(fLoc+fA1, DEC);
          if(flashArr[fA1] == 0xFF) {
             serial.println("No matching RFID data found - appending all of flash memory.");
             extractMemRFID(3, datStart);
             return;
          }
          uint8_t lineLen = 0;
          if(flashArr[fA1] < 100) {lineLen = 10;} else {lineLen = 12;} //detect tag type and set line length
            if(flashArr[fA1+lineLen-4] == BX){
              bool match = compareArrays(flashArr, SDline, fA1, 0, SDLineBytes);
              if(match) {
                uint32_t startPoint = fLoc + fA1 + SDLineBytes;  
                serial.print("Matching RFID data found on SD card. ");  //serial.println(startPoint, DEC);
                if(startPoint < memLoc) {
                  serial.print(" Appending new data starting at "); serial.println(startPoint, DEC);
                  extractMemRFID(3, startPoint);      
                return;
                } else {
                  serial.println("RFID Data up to date, no data transfer needed.");
                  serial.println();
                  return;
                } 
              }
            }
          fA1 = fA1 + lineLen;
          }
          fLoc = fLoc + fA1;
      } 
      // no match if you made it this far. Append everything
      serial.println("No matching data found - appending everything from flash memory.");
      extractMemRFID(3, datStart);
      return;
    }
  }
}



boolean compareArrays(char *a, char *b, uint16_t start_a, uint16_t start_b, uint8_t len) { //define arrays, start points, and lenght of comparison
      // test each element to be the same. if not, return false
      //serial.println("checking bytes ");
      uint16_t m = start_b;
      for (uint16_t n = start_a; n < len + start_a; n++) {
        //serial.print("iteration "); serial.print(n, DEC); serial.print(": "); serial.print(a[n], HEX); serial.print(" vs "); serial.println(b[m], HEX);
        if (a[n] != b[m]) {
          //serial.println("FAIL!!");
          return false;
        }
        m++;
      }
      //serial.println(); serial.println("all matches OK");
      return true; //if you get this far all matches are OK
}

uint8_t compressLogLine(char *SDarr, char *line) { //SDarr = array of chars, line = array to write compressed data, lfn = first byte of input charater array, lnSt = where to start writing, leng = how many bytes in each character line. 
  // "Logging_started" = 11
  // "Going_to_sleep_" = 12
  // "Wake_from_sleep" = 13
  
//  serial.print("Compressing line: ");
//  for(uint8_t i = 0; i < 37; i++){
//    serial.print(SDarr[i]); 
//  }
//  serial.println();
  
  switch(SDarr[0])
  {
    case 'L' : line[0] = 11; break;
    case 'G' : line[0] = 12; break;
    case 'W' : line[0] = 13; break;
  }
  
  byte mo = (char2hex(SDarr[17])*10) + char2hex(SDarr[18]);
  byte da = (char2hex(SDarr[20])*10) + char2hex(SDarr[21]);
  byte yr = (char2hex(SDarr[25])*10) + char2hex(SDarr[26]);
  byte hh = (char2hex(SDarr[28])*10) + char2hex(SDarr[29]);
  byte mm = (char2hex(SDarr[31])*10) + char2hex(SDarr[32]);
  byte ss = (char2hex(SDarr[34])*10) + char2hex(SDarr[35]);
  
  //serial.print(mo,DEC); serial.print(" "); serial.print(da,DEC); serial.print(" "); serial.print(yr,DEC); serial.print(" "); serial.print(hh,DEC); serial.print(" "); serial.print(mm,DEC); serial.print(" "); serial.print(mo,DEC); serial.print(" "); serial.print(ss,DEC); serial.println(" "); 
  
  unixTime.unixLong = getUnix2(yr, mo, da, hh, mm, ss);
  line[1] = unixTime.b1;
  line[2] = unixTime.b2;
  line[3] = unixTime.b3;
  line[4] = unixTime.b4;

  //serial.print(line[1], HEX); serial.print(" "); serial.print(line[1], HEX); serial.print(" "); serial.print(line[2], HEX); serial.print(" "); serial.print(line[3], HEX); serial.print(" "); serial.println(line[4], HEX);

}

uint8_t compressSDLine(char *SDarr, char *line, uint8_t leng) { //SDarr = array of chars, line = array to write compressed data, lfn = first byte of input charater array, lnSt = where to start writing, leng = how many bytes in each character line. 
  serial.println();
  if(leng > 38) { //longer than 34 = ISO tag
    //serial.println("Compressing ISO tag.");
    line[0] = char2hex(SDarr[21]) | 0b10000000;
    countryCode = (char2hex(SDarr[0]) << 8) + (char2hex(SDarr[1]) << 4) + char2hex(SDarr[2]);
    line[6] = countryCode >> 2;      //country code is encoded across two byte, and byte are read in reverse order for ISO tags.
    line[5] = (char2hex(SDarr[4]) << 4) + char2hex(SDarr[5]); 
    line[5] = (line[5] & 0b00111111) + (countryCode << 6);     // tack on two bits from country code.
    line[4] = (char2hex(SDarr[6]) << 4) + char2hex(SDarr[7]);
    line[3] = (char2hex(SDarr[8]) << 4) + char2hex(SDarr[9]);
    line[2] = (char2hex(SDarr[10]) << 4) + char2hex(SDarr[11]);
    line[1] = (char2hex(SDarr[12]) << 4) + char2hex(SDarr[13]);
    line[7] = (char2hex(SDarr[16]) * 100) + (char2hex(SDarr[17]) * 10) + char2hex(SDarr[18]);
    byte mo = (char2hex(SDarr[24])*10) + char2hex(SDarr[25]);
    byte da = (char2hex(SDarr[27])*10) + char2hex(SDarr[28]);
    byte yr = (char2hex(SDarr[32])*10) + char2hex(SDarr[33]);
    byte hh = (char2hex(SDarr[35])*10) + char2hex(SDarr[36]);
    byte mm = (char2hex(SDarr[38])*10) + char2hex(SDarr[39]);
    byte ss = (char2hex(SDarr[41])*10) + char2hex(SDarr[42]);
    unixTime.unixLong = getUnix2(yr, mo, da, hh, mm, ss);
    line[8] = unixTime.b1;
    line[9] = unixTime.b2;
    line[10] = unixTime.b3;
    line[11] = unixTime.b4;

//    for(uint8_t i = 0; i<12; i++) {
//      serial.print(line[i], HEX); serial.print(" ");
//    }
    return 12;
  }
  if(leng < 38) { //shorter than 34 = EM4100 tag
    //serial.println("Compressing EM4100 tag.");
    line[0] = char2hex(SDarr[12]);
    line[1] = (char2hex(SDarr[0]) << 4) + char2hex(SDarr[1]); 
    line[2] = (char2hex(SDarr[2]) << 4) + char2hex(SDarr[3]);
    line[3] = (char2hex(SDarr[4]) << 4) + char2hex(SDarr[5]);
    line[4] = (char2hex(SDarr[6]) << 4) + char2hex(SDarr[7]);  
    line[5] = (char2hex(SDarr[8]) << 4) + char2hex(SDarr[9]);
    //serial.print(" - "); serial.print(" ")
    byte mo = (char2hex(SDarr[15])*10) + char2hex(SDarr[16]);
    byte da = (char2hex(SDarr[18])*10) + char2hex(SDarr[19]);
    byte yr = (char2hex(SDarr[23])*10) + char2hex(SDarr[24]);
    byte hh = (char2hex(SDarr[26])*10) + char2hex(SDarr[27]);
    byte mm = (char2hex(SDarr[29])*10) + char2hex(SDarr[30]);
    byte ss = (char2hex(SDarr[32])*10) + char2hex(SDarr[33]);
    //serial.print(mo, DEC); serial.print(da, DEC); serial.print(yr, DEC); serial.print(hh, DEC); serial.print(mm, DEC); serial.print(ss, DEC);
    unixTime.unixLong = getUnix2(yr, mo, da, hh, mm, ss);
    //serial.print(" "); serial.print(unixTime.unixLong, DEC);
    line[6] = unixTime.b1;
    line[7] = unixTime.b2;
    line[8] = unixTime.b3;
    line[9] = unixTime.b4;

//    for(uint8_t i = 0; i<10; i++) {
//      serial.print(line[i], HEX); serial.print(" ");
//    }
    return 10;
  }
}


char char2hex(char ch)
{
  char rT;
  switch(ch)
  {
    case '0': rT = 0; break;
    case  '1' : rT = 1; break;
    case  '2': rT = 2; break;
    case  '3': rT = 3; break;
    case  '4' :rT = 4; break;
    case  '5': rT = 5; break;
    case  '6': rT = 6; break;
    case  '7': rT = 7; break;
    case  '8': rT = 8; break;
    case  '9': rT = 9; break;
    case  'A': rT = 10; break;
    case  'B': rT = 11; break;
    case  'C': rT = 12; break;
    case  'D': rT = 13; break;
    case  'E': rT = 14; break;
    case  'F' : rT = 15; break;
    default: rT = 0; break;
  }
  return rT;
}


void extractMemRFID(uint8_t prntWrt, uint32_t flashStart) {  //Takes data from Flash and prints to screen and/or writes to SD card
  if(memLoc <= flashStart) {
    serial.println("no RFID data to write");
    return;
  }
  bool prnt = bitRead(prntWrt, 0);
  bool wrt = bitRead(prntWrt, 1);
  char BA[500];           //define byte array to store (500 bytes)
  uint32_t dMem = flashStart;  //counter for memory position
  File myFile;

  //Check if file on SD card exists. if not create it.
  if(wrt && SDOK == 1) {
    SDstart();
    if(!SD.exists(dataFile)) {
      serial.println("Creating new file on SD card");
      myFile = SD.open(dataFile, FILE_WRITE);
      delay(10);
      myFile.close();
    } 
    
    //else {
    //  serial.println("file on SD card found");
    //}
    //SDstop();
  }
  
  serial.print("transferring data from "); serial.print(flashStart, DEC); serial.print(" to "); serial.println(memLoc, DEC);
  while(dMem < memLoc) {                 // read batches of flash data until end of data is reached.   
     //showFlash(dMem, dMem+500);
     digitalWrite(LED_RFID, LOW);   // Flash LED to indicate progress             
     readFlash(dMem, BA, 500);       // Read in batch of data

     digitalWrite(LED_RFID, HIGH);  // Flash LED to indicate progress 
     flashOff();                    // Make sure flash is off 
     //serial.print("Flash batch read in starting at "); serial.println(dMem, DEC);
     uint16_t b = 0;                // initialize byte counter

     //Write lines to SD card and/or serial
     if(wrt && (SDOK == 1)) {  //open SD card file if needed
         myFile = SD.open(dataFile, FILE_WRITE);  //Open for appending new data to file
     } 
     
     while((b < 480) & (dMem < memLoc)) {       //Don't go through end of array, in case of cutting off data lines.
        //serial.print("first byte of line at position "); serial.print(b, DEC); serial.print(" is "); serial.println(BA[b], HEX);
        if((BA[b] == 129) | (BA[b] == 130)) {
            //serial.println("Writing ISO line");
            unixTime.b1 = BA[b+8]; unixTime.b2 = BA[b+9]; unixTime.b3 = BA[b+10]; unixTime.b4 = BA[b+11];
            convertUnix(unixTime.unixLong);  // convert unix time. Time values get stored in array timeIn, bytes 0 through 5.
            static char text[41]; 
            countryCode = (BA[b+6]<<2) + (BA[b+5]>>6);
            sprintf(text, "%03X.%02X%02X%02X%02X%02X, %03d, %d, %02d/%02d/%04d %02d:%02d:%02d",
                 countryCode, (BA[b+5] & 0b00111111), BA[b+4], BA[b+3], BA[b+2], BA[b+1], BA[b+7], (BA[b] & 0x0F),  
                 timeIn[0], timeIn[1], timeIn[2], timeIn[3], timeIn[4], timeIn[5]);
            //if(prnt) {serial.print(dMem, DEC); serial.print(" "); serial.println(text);}
            if(prnt) {serial.println(text);}
            if(wrt && SDOK == 1) {myFile.println(text);}
            b = b + 12;
            dMem = dMem + 12;
         } 
         if((BA[b] == 1) | (BA[b] == 2)) {
            //serial.println("Writing EM4100 line");
            unixTime.b1 = BA[b+6]; unixTime.b2 = BA[b+7]; unixTime.b3 = BA[b+8]; unixTime.b4 = BA[b+9];
            convertUnix(unixTime.unixLong);
            static char text[41];
            sprintf(text, "%02X%02X%02X%02X%02X, %d, %02d/%02d/%04d %02d:%02d:%02d",
                  BA[b+1], BA[b+2], BA[b+3], BA[b+4], BA[b+5], BA[b], timeIn[0], timeIn[1], timeIn[2], timeIn[3], timeIn[4], timeIn[5]);
            //if(prnt) {serial.print(dMem, DEC); serial.print(" "); serial.println(text);}
            if(prnt) {serial.println(text);}
            if(wrt && SDOK == 1) {myFile.println(text);}
            b = b + 10;
            dMem = dMem + 10;
          }
          if( ((BA[b] > 9 && BA[b] < 129) || (BA[b] > 133)) && (dMem < memLoc) ) {
            serial.println("data file alignment error. Store data and write all to new file");
            serial.println("last dMem:"); serial.println(dMem, DEC);
            serial.print(b, DEC); serial.print(" "); serial.print(BA[b], HEX); serial.print(" "); serial.print(BA[b+1], HEX); serial.print(" "); serial.println(BA[b+3], HEX);
            showFlash(4224, 5500); 
        
            delay(500);
            if(wrt && SDOK == 1) {myFile.close(); SDstop();}      //close the file 
            dMem = memLoc + 1000; 
            break;
            return;
          }   
          //serial.print("dMem is now "); serial.println(dMem, DEC);  
      } 
      if(wrt && SDOK == 1) {
        myFile.close();  //close the file 
        //SDstop();
      }     
  }
  serial.println();
}


void extractMemLog(uint8_t prntWrt, uint32_t flashStart) {
//  serial.println("Write log mem...");
//  serial.println();
  
  if(logLoc <= flashStart) {
    serial.println("no log data to write");
    return;
  }

  bool prnt = bitRead(prntWrt, 0);
  bool wrt = bitRead(prntWrt, 1);
  char BA[500];           //define byte array to store (500 bytes)
  uint32_t dMem = flashStart;  //counter for memory position
  File myFile;

    //Check if file on SD card exists. if not create it.
  if(wrt && SDOK == 1) {
    SDstart();
    if(!SD.exists(logFile)) {
      serial.print("Creating new log file on SD card: ");
      serial.println(logFile);
      myFile = SD.open(logFile, FILE_WRITE);
      delay(10);
      myFile.close();
    } 
    //else {
    //  serial.println("file on SD card found");
    //}
    //SDstop();
  }

  if(!SD.exists(logFile)) {
    serial.println("log file not created!!");
  }

  serial.print("transferring data from "); serial.print(flashStart, DEC); serial.print(" to "); serial.println(logLoc, DEC);
  while(dMem < logLoc) {                 // read batches of flash data until end of data is reached.   
     //showFlash(dMem, dMem+500);
     digitalWrite(LED_RFID, LOW);   // Flash LED to indicate progress             
     readFlash(dMem, BA, 500);       // Read in batch of data

     digitalWrite(LED_RFID, HIGH);  // Flash LED to indicate progress 
     flashOff();                    // Make sure flash chip is off 
     //serial.print("Flash batch read in starting at "); serial.println(dMem, DEC);
     uint16_t b = 0; 

          //Write lines to SD card and/or serial
     if(wrt && (SDOK == 1)) {  //open SD card file if needed
         myFile = SD.open(logFile, FILE_WRITE);  //Open for appending new data to file
     } 
     
     while((b < 490) & (dMem < logLoc)) {       //Don't go through end of array, in case of cutting off data lines.
        //serial.print("first byte of line at position "); serial.print(b, DEC); serial.print(" is "); serial.println(BA[b], HEX);
        if(BA[b] != 0xFF) {
          getLogMessage(BA[b]); //Log message gets loaded into logMess
          unixTime.b1 = BA[b+1]; unixTime.b2 = BA[b+2]; unixTime.b3 = BA[b+3]; unixTime.b4 = BA[b+4];
          convertUnix(unixTime.unixLong);  // covert unix time. Time values get stored in array timeIn, bytes 0 through 5.
          static char text[20]; 
          sprintf(text, ", %02d/%02d/%04d %02d:%02d:%02d", timeIn[0], timeIn[1], timeIn[2], timeIn[3], timeIn[4], timeIn[5]);
          if(prnt){
             serial.print(logMess);
             serial.println(text);
          }
          if(wrt && SDOK == 1){
             myFile.print(logMess);
             myFile.println(text);
          }
          b = b + 5;
          dMem = dMem + 5;
        } else {
          break; 
        }
       //serial.print("dMem is now "); serial.println(dMem, DEC);  
      } 
      if(wrt && SDOK == 1) {
        myFile.close();  //close the file 
        //SDstop();
      }     
  }
  serial.println();
  return;
}



//void extractMemLogOld(uint8_t prntWrt) { 
//  //write LOG data to SD card
// 
//  bool prnt = bitRead(prntWrt, 0);
//  bool wrt = bitRead(prntWrt, 1);
//  File dataFile;
//  if(wrt && SDOK == 1) {
//    fName[5] = 'L';                                      // change fName to Log file
//    serial.println("Writing log data to SD card ");  // Message to user 
//  } 
//  if(logLoc > logStart) {     //proceed only if there are data - write nothing if there are no data
//    byte BA[533];             //define byte array to store a full page (528 bytes) plus 12 extra bytes
//    int b = 0;
//    uint32_t dMem = logStart;  
//    if(prnt) {
//      serial.println("Printing Flash Memory - Log...");
//      serial.println();
//    }
//    while(dMem < logLoc) {                 // read in full pages one at a time until last page is reached.
//       digitalWrite(LED_RFID, LOW);                 // Flash LED to indicate progress             
//       uint32_t pAddr = dMem/528;
//       if(wrt && SDOK == 1) {
//          serial.print("Transfering log data from page ");   // progress message 
//          serial.println(pAddr, DEC);
//       } 
//       pAddr = pAddr << 10;   
//       flashOn();
//       SPI.transfer(0x03);                         // opcode for low freq read
//       SPI.transfer((pAddr >> 16) & 0xFF);      // write most significant byte of Flash address
//       SPI.transfer((pAddr >> 8) & 0xFF);       // second address byte
//       SPI.transfer(0);                            // third address byte
//       
//       //SPI.transfer(BA, 528);                  // might work for reading in full page?? needs test.
//       
//       for (int n = 0; n < 532; n++) {           //read in 540 bytes - page crossover should be OK. 
//         BA[n] = SPI.transfer(0);                
//         //serial.print(BA[n]);
//       }    
//       digitalWrite(LED_RFID, HIGH);
//       flashOff();
//
//       if(wrt && SDOK == 1) {
//          SDstart();
//          dataFile = SD.open(fName, FILE_WRITE);
//       }
//       
//       while(b < 528) {
//          if(BA[b] != 0xFF) {
//            getLogMessage(BA[b]); //Log message gets loaded into logMess
//            //serial.println("process log line");
//            unixTime.b1 = BA[b+1]; unixTime.b2 = BA[b+2]; unixTime.b3 = BA[b+3]; unixTime.b4 = BA[b+4];
//            convertUnix(unixTime.unixLong);  // covert unix time. Time values get stored in array timeIn, bytes 0 through 5.
//            static char text[20]; 
//            sprintf(text, ", %02d/%02d/%04d %02d:%02d:%02d", timeIn[0], timeIn[1], timeIn[2], timeIn[3], timeIn[4], timeIn[5]);
//            if(prnt){
//              serial.print(logMess);
//              serial.println(text);
//            }
//            if(wrt && SDOK == 1){
//              dataFile.print(logMess);
//              dataFile.println(text);
//            }
//            b = b + 5;
//          } else {
//            b=1000;
//          }  
//        }
//        if(wrt && SDOK == 1) {
//          dataFile.close();      //close the file 
//          SDstop();
//        }
//        b = b - 528;    //Pick up somewhere in the next page 
//        dMem = dMem + 528;  //Go to next page.
//      //        serial.println();
//      //        serial.print("next page ");
//      //        serial.println(dMem, DEC);
//      //        serial.println();
//      }
//    } else {
//    serial.println("No Log data to write");
//  }
//  fName[5] = 'D'; 
//  serial.println();
//}
//


////////////SD CARD FUNCTIONS////////////////////

//Startup routine for the SD card
bool SDstart() {                 // Startup routine for the SD card
  digitalWrite(SDselect, HIGH);  // Deactivate the SD card if necessary
  digitalWrite(FlashCS, HIGH);   // Deactivate flash chip if necessary
  pinMode(SDon, OUTPUT);         // Make sure the SD power pin is an output
  digitalWrite(SDon, LOW);       // Power to the SD card
  delay(20);
  digitalWrite(SDselect, LOW);   // SD card turned on
  if (!SD.begin(SDselect)) {     // Return a 1 if everyting works
    //serial.println("SD fail");
    return 0;
  } else {
    return 1;
  }
}

//Stop routine for the SD card
void SDstop() {                 // Stop routine for the SD card
  delay(20);                    // delay to prevent write interruption
  SD.end();                     // End SD communication
  digitalWrite(SDselect, HIGH); // SD card turned off
  digitalWrite(SDon, HIGH);     // power off the SD card
}

uint32_t getMemLoc(uint32_t startMem, uint32_t endMem){ //startMem = beginning of first page; endMem = beginning of last page.
    //serial.println("Getting memLoc");
    char ccc[5] = {0,0,0,0,0};
    char ff[533]; //huge array for entire page plus a few at the end. 
    uint32_t mLoc = startMem; //Start with specified memory location
    startMem = startMem + 527;  //look at last byte on first memory page

    while(startMem <= endMem){  //This loop provides the last page address
       readFlash(startMem, ccc, 5);
       if((ccc[0]==0xFF) && (ccc[1]==0xFF) && (ccc[2]==0xFF) && (ccc[3]==0xFF) && (ccc[4]==0xFF)) {
          break;
          } else {
          startMem = startMem + 528;   //Data found at end of page. try the next one
       }  
    }
    //uint32_t endLoc = startMem + 80; //Define uppler limit for search
    startMem = startMem-527; //Go back to beginning of page
    //serial.print("Byte search starting at "); serial.println(startMem, DEC);
    readFlash(startMem, ff, 533);  //read in entire page plus a few bytes
    for(uint16_t i = 0; i < 528; i++) { //
      if(ff[i] == 0xFF) {
        if((ff[i+1]==0xFF) && (ff[i+2]==0xFF) && (ff[i+3]==0xFF) &(ff[i+4]==0xFF)) { //Check for a string of empty bytes.
          //serial.print("end found at "); 
          startMem = startMem+i;
          //serial.println(startMem, DEC);
          return startMem;
        }
      }
    }
    serial.print("Problem finding memory location");
    return 4224;
}

bool writeSDLine(String fName, uint8_t mess, char *BA) {
  bool success = 0;       // valriable to indicate success of operation
  SDstart();                                        // start up the SD card
  File dFile = SD.open(fName, FILE_WRITE);          // Open the file
  if (dFile) {                                      // If the file is opened successfully...
     if(mess !=0) {                                 // write if it is a log file
        getLogMessage(mess); //Log message gets loaded into logMess
        unixTime.b1 = BA[1]; unixTime.b2 = BA[2]; unixTime.b3 = BA[3]; unixTime.b4 = BA[4];
        convertUnix(unixTime.unixLong);  // covert unix time. Time values get stored in array timeIn, bytes 0 through 5.
        static char text[22]; 
        sprintf(text, ", %02d/%02d/%04d %02d:%02d:%02d", timeIn[0], timeIn[1], timeIn[2], timeIn[3], timeIn[4], timeIn[5]);
        text[21] = '\0';
        
        dFile.print(logMess);  // write log message
        dFile.println(text);     // write date/time and new line      
      }
      if(mess == 0) {   
        dFile.println(BA);
      }
      success = 1;                                         // ...note success of operation...
      dFile.close();                                       // ...close the file...
  }
  SDstop();                                              // Disable SD
  //fName[5] = 'D';                                        // Make sure fName is set to the RFID data file
  return success;                                        // Indicates success (1) or failure (0)
}

///////Sleep Function/////////////Sleep Function/////////

void lpSleep() {
  digitalWrite(MOTR, HIGH) ;                         // Must be set high to get low power working - don't know why
  shutDownRFID();                                    // Turn off both RFID circuits
  attachInterrupt(INT1, ISR, FALLING);               // Set up interrupt to detect high to low transition on interrupt pin
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_EIC) |     // Configure EIC to use GCLK1 which uses XOSC32K
                      GCLK_CLKCTRL_GEN_GCLK1 |       // This has to be done after the first call to attachInterrupt()
                      GCLK_CLKCTRL_CLKEN;
  USB->DEVICE.CTRLA.reg &= ~USB_CTRLA_ENABLE;        // disable USB
  SysTick->CTRL  &= ~SysTick_CTRL_ENABLE_Msk;        // disable ms clock

  __WFI();    //Enter sleep mode
  //...Sleep...wait for interrupt
  USB->DEVICE.CTRLA.reg |= USB_CTRLA_ENABLE;         // enable USB
  detachInterrupt(INT1);                             // turn interrupt off
  SysTick->CTRL  |= SysTick_CTRL_ENABLE_Msk;         // Enable clock
  Debug = 0;
}

void ISR() {     // dummy routine - no interrupt activity needed
  byte SLEEP_FLAG = false;
}
