/*
  Data logging sketch for the ETAG RFID Reader
  ETAG READER Version 10.0
  PROCESSOR SAMD21J
  USE BOARD DEFINITION: ETAG RFID V2 D21
  Code by:
   Jay Wilhelm
   Tho Trinh
   Eli Bridge
   Alexander Moreno
   David Mitchell

  Sept 2019

  Licenced in the public domain

  REQUIREMENTS:
  Power supply for the board should come from the USB cable or a 5V battery or DV power source.
  A 3.3V CR1025 coin battery should be installed on the back side of the board to maintain the date
      and time when the board is disconnected from a primary power source.

  LIBRARIES:
  The RV3129 library is accessible on github here:
  https://github.com/OUIDEAS/RV-3129_Arduino_Library.git

  FLASH MEMORY STRUCTURE:
  The onboard flash memory is divided into pages of 528 bytes each. There are probably several thousand pages.
  Page 0 is reserved for RFID tag codes  (Addresses 0x00000000 to 0x000000??
  Page 1 is reserved for parameters and counters (Addresses 0x00000400 to ??
    bytes 0-2 (0x400 - (0x402) - first three are the current backup memory address
    byte 3 - (0x403) Set to 0xAA once the memory address counter is intialized. (important mainly for the very first initialization process with a completely blank Flash memory)
    bytes 4-7 - (0x404-0x407) next four bytes are for the reader ID charaters
    byte 8 - Set to 0xAA once the Reader ID is established - if this byte is anything other than 0xAA then the Reader ID will be set to the default
    byte 9 = tagCount (the number of tags stored on page 0)
    byte 10 = feeder mode
    byte 11-12 = timeCalFreq - how frequently to calibrate clock with GPS (in minutes)    
  Page 2 (0x00000800) and the rest is for backup data  
*/

// ***********INITIALIZE INCLUDE FILES AND I/O PINS*******************
#include "RV3129.h"          // include library for the real time clock - must be installed in libraries folder
#include <Wire.h>            // include the standard wire library - used for I2C communication with the clock
#include <SD.h>              // include the standard SD card library
#include <SPI.h>             // include standard SPI library
#include "Manchester.h"

#define serial SerialUSB       // Designate the USB connection as the primary serial comm port - note lowercase "serial"
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
char deviceID[5] = "RFID";            // User defined name of the device
String deviceIDstr;                   // String verion of device name.
String dataFile;                      // Stores text file name for SD card writing.
String logFile;                       // Stores text file name for SD card writing.
uint32_t fAddress;                    // Address pointer for logging to backup memory

uint32_t pastRFID;                    // stores past RFID number - for use with delayTime
uint32_t pastHHMMSS;                  // stores the past Hour, Minute and Second - for use with delayTime

unsigned int pageAddress;             // page address for flash memory
unsigned int byteAddress;             // byte address for flash memory

byte RFcircuit = 1;                   // Used to determine which RFID circuit is active. 1 = primary circuit, 2 = secondary circuit.
String currentDate;                   // USed to get the current date in mm/dd/yyyy format (we're weird in the US)
String currentTime;                   // Used to get the time
String currentDateTime;               // Full date and time string
String timeString;                    // String for storing the whole date/time line of data
char incomingByte = 0;                // Used for incoming serial data
unsigned int timeIn[12];              // Used for incoming serial data during clock setting
byte menu;                            // Keeps track of whether the menu is active.

// Global variable for tag codes

char RFIDstring[10];                  // Stores the TagID as a character array (10 character string)
byte RFIDtagUser = 0;                 // Stores the first (most significant) byte of a tag ID (user number)
unsigned long RFIDtagNumber = 0;      // Stores bytes 1 through 4 of a tag ID (user number)
byte RFIDtagArray[5];                 // Stores the five individual bytes of a tag ID.


// ********************CONSTANTS (SET UP LOGGING PARAMETERS HERE!!)*******************************
const byte checkTime = 30;                          // How long in milliseconds to check to see if a tag is present (Tag is only partially read during this time -- This is just a quick way of detirmining if a tag is present or not
const unsigned int pollTime1 = 200;                 // How long in milliseconds to try to read a tag if a tag was initially detected (applies to both RF circuits, but that can be changed)
const unsigned int delayTime = 8;                   // Minimim time in seconds between recording the same tag twice in a row (only applies to data logging--other operations are unaffected)
const unsigned long pauseTime = 100;                // CRITICAL - This determines how long in milliseconds to wait between reading attempts. Make this wait time as long as you can and still maintain functionality (more pauseTime = more power saved)
uint16_t pauseCountDown = pauseTime / 31.25;        // Calculate pauseTime for 32 hertz timer
byte pauseRemainder = ((100*pauseTime)%3125)/100;   // Calculate a delay if the pause period must be accurate
//byte pauseRemainder = 0 ;                         // ...or set it to zero if accuracy does not matter

const byte slpH = 11;                            // When to go to sleep at night - hour
const byte slpM = 12;                            // When to go to sleep at night - minute
const byte wakH = 11;                            // When to wake up in the morning - hour
const byte wakM = 13;                            // When to wake up in the morning - minute
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
unsigned int stopCycleCount = 400;     // How many read cycles to maintain serial comminications
bool Debug = 1;                       // Use to stop serial messages once sleep mode is used.






//////SETUP//////////////SETUP//////////////SETUP//////////////SETUP////////

void setup() {  // setup code goes here, it is run once before anything else

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
  
  blinkLED(LED_RFID, 4, 400);     // blink LED to provide a delay for serial comms to come online
  serial.begin(9600);
  serial.println("Start");

  // Set up SD card communication 
  serial.println("Initializing SD card...");       // message to user
  if(SDwriteString("Message", "DUMMY.TXT")) {    // Write a dummy file to the SD card   
      serial.println("SD card OK");                // OK message
      SDremoveFile("DUMMY.TXT");                   // delete the dummy file      
  } else {     
      serial.println("SD card not working");       // error message
  }

  //Check flash memory and initialize if needed
  serial.print("Is flash memory initialized? ");    //It is neccessary to initialize the flash memory on the first startup
  byte byte0 = readFlashByte(0x00000403);           //Read a particular byte from the flash memory
  if (readFlashByte(0x00000403) != 0xAA) {          //If the byte is 0xFF then the flash memory needs to be initialized
    serial.println("NO!");                          //Message
    serial.println("Initializing Flash Memory..."); //Message
    writeFlashByte(0x00000403, 0xAA);               //Write a different byte to this memory location
    char fa[3] = {0, 8, 0};                         //Set initial flash memory address for logging data to page 2, byte address 0
    writeFlashArray(0x400, fa, 3);                  //Set initial flash memory address for logging data to page 2, byte address 0
  } else {
    serial.println("YES");                          //Confirm initialization
  }

  fAddress = getFlashAddr(); 
  if (fAddress < 0x00000800) {         // Check to make sure memory address makes sense
    char fa[3] = {0, 8, 0};            // if not reset flash memory address 
    writeFlashArray(0x400, fa, 3); 
  }

  if (readFlashByte(0x00000408) != 0xAA) {  //If this byte is not 0xAA then set a default deviceID
    serial.println("Setting default device ID - Please update this!!!");
    deviceID[0] = 'R'; deviceID[1] = 'F'; deviceID[2] = '0'; deviceID[3] = '1';
    writeFlashArray(0x00000404, deviceID, 4);  //write to flash memory without updating flash address
  }
  
//  serial.println("pauseTime - pauseCountDown  -  pauseRemainder");
//  serial.print(pauseTime);
//  serial.print("        ");
//  serial.print(pauseCountDown);
//  serial.print("        ");
//  serial.println(pauseRemainder);

  // start clock functions and check time
  Wire.begin();  //Start I2C bus to communicate with clock.
  if (rtc.begin() == false) {  // Try initiation and report if something is wrong
    serial.println("Something wrong with clock");
  } else {
    rtc.set24Hour();
  }

  //////////////MENU////////////MENU////////////MENU////////////MENU////////
  //Display all of the following each time the main menu is called up.
  
  byte menu = 1;
  while (menu == 1) {
    serial.println();
    rtc.updateTime();
    serial.println(showTime());
    
    //Read in parameters and display parameters from the flash memory
    //Get and display the device ID
    readFlashArray(0x404, deviceID, 4);   //Read in device ID
    serial.println("Device ID: " + String(deviceID));          //Display device ID
    //serial.println(deviceID);             //Display device ID

    //Read and display flash memory address
    fAddress = getFlashAddr(); 
    serial.print("Flash memory address: Page ");
    serial.print(fAddress >> 10, DEC);
    serial.print(" - Byte ");
    serial.println(fAddress & 0x3FF, DEC);
    
    //Define the log and data files
    deviceIDstr = String(deviceID);
    logFile = String(deviceIDstr +  "LOG.TXT");
    dataFile = String(deviceIDstr + "DATA.TXT");

    // Ask the user for instruction and display the options
    serial.println("What to do? (input capital letters)");
    serial.println("    C = set clock");
    serial.println("    B = Display backup memory");
    serial.println("    E = Erase (reset) backup memory");
    serial.println("    I = Set device ID");

    //Get input from user or wait for timeout
    incomingByte = getInputByte(10000);
    String printThis = String("Value recieved: ") + incomingByte;
    serial.println(printThis);
    switch (incomingByte) {                                               // execute whatever option the user selected
      default:
        menu = 0;         //Any non-listed entries - set menu to 0 and break from switch function. Move on to logging data
        break;
      case 'C': {                                                         // option to set clock
          inputTime();                                                    // calls function to get time values from user
          break;                                                        //  break out of this option, menu variable still equals 1 so the menu will display again
        }
      case 'I': {
          inputID();   //  calls function to get a new feeder ID
          writeFlashByte(0x00000408, 0xAA);                           //Write a different byte to this memory location
          break;       //  break out of this option, menu variable still equals 1 so the menu will display again
        }
      case 'E': {
          char fa[3] = {0, 8, 0};                          //reset flash memory address to page 2, byte address 0
          writeFlashArray(0x400, fa, 3);                   //reset flash memory address to page 2, byte address 0
          serial.println("BACKUP MEMORY RESET");
          break;       //  break out of this option, menu variable still equals 1 so the menu will display again
        }
      case 'B': {
          dumpMem(0);
          break;       //  break out of this option, menu variable still equals 1 so the menu will display again
        }
    } //end of switch
  } //end of while(menu = 1){

  rtc.updateTime();
  String logStr = "Logging started at " + showTime();
  SDwriteString(logStr, logFile);
  RFcircuit = 1;
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
     String SlpStr =  "Entering sleep mode at " + showTime();  // if it's time to sleep make a log message
     if(Debug) serial.println(SlpStr);                         // print log message
     SDwriteString(SlpStr, logFile);                           // save log message
     sleepAlarm();                                             // sleep using clock alarm for wakeup
     rtc.updateTime();                                         // get time from clock
     SlpStr =  "Wake up from sleep mode at " + showTime();     // log message
     SDwriteString(SlpStr, logFile);                           // save log message
  }

//////Read Tags//////////////Read Tags//////////
//Try to read tags - if a tag is read and it is not a recent repeat, write the data to the SD card and the backup memory.

  if (FastRead(RFcircuit, checkTime, pollTime1) == 1) {
    processTag(RFIDtagArray, RFIDstring, RFIDtagUser, &RFIDtagNumber);                  // Parse tag data into string and hexidecimal formats
    rtc.updateTime();                                                                   // Update time from clock
    String SDsaveString = String(RFIDstring) + ", " + RFcircuit + ", " + showTime();    // Create a data string
    uint32_t HHMMSS = rtc.getHours()*10000 + rtc.getMinutes()*100 + rtc.getSeconds();   // Creat a time value
    if(RFIDtagNumber != pastRFID | (HHMMSS-pastHHMMSS >= delayTime)) {                  // See if the tag read is a recent repeat
      SDwriteString(SDsaveString, dataFile);                                            // If it's a new read, save to the SD card
      char flashData[12] = {RFIDtagArray[0], RFIDtagArray[1], RFIDtagArray[2],          // Create an array representing an entire line of data
          RFIDtagArray[3], RFIDtagArray[4], RFcircuit, rtc.getMonth(), rtc.getDate(), 
          rtc.getYear(), rtc.getHours(), rtc.getMinutes(), rtc.getSeconds()};
      writeFlashArray(fAddress, flashData, 12);                                         // write the array of 12 bytes
      fAddress = advanceFlashAddress(fAddress, 12);                                     // Add 12 to the flash address 
      pastRFID = RFIDtagNumber;                                                         // Update past RFID to check for repeats       
      pastHHMMSS = HHMMSS;                                                              // Update pastHHMMSS to check for repeats
    } else {
      if(Debug) serial.println("Repeat - Data not logged");                             // Message to indicate repeats (no data logged)
    }
    if(Debug){
      serial.println("Tag detected.");
      serial.println(SDsaveString);
      serial.println();
    }
    blipLED(5);
 }

//////////Pause//////////////////Pause//////////
//After each read attempt execute a pause using either a simple delay or low power sleep mode.

  if(cycleCount < stopCycleCount){   // Pause between read attempts with delay or a sleep timer 
    delay(pauseTime);                // Use a simple delay and keep USB communication working
    cycleCount++ ;                   // Advance the counter                         
  }else{
    sleepTimer(pauseCountDown, pauseRemainder);
   }
}


/////////////////////////////////////////////////////////////////////////////
/////FUNCTIONS/////////FUNCTIONS/////////FUNCTIONS/////////FUNCTIONS/////////
/////////////////////////////////////////////////////////////////////////////

//repeated LED blinking function
void blinkLED(uint8_t ledPin, uint8_t repeats, uint16_t duration) { //Flash an LED or toggle a pin
  pinMode(ledPin, OUTPUT);             // make pin an output
  for (int i = 0; i < repeats; i++) {  // loop to flash LED x number of times
    delay(duration);                   // pause for a while
    digitalWrite(ledPin, LOW);         // turn the LED on (LOW turns it on)
    delay(duration);                   // pause again
    digitalWrite(ledPin, HIGH);        // turn the LED off (HIGH turns it off)
  }                                    // end loop
}                                      // End function

//Quick LED blink
void blipLED(uint16_t blip) {
  digitalWrite(LED_RFID, LOW);    // turn the LED on (LOW turns it on)
  delay(blip);                    // leave LED on for a quick flash
  digitalWrite(LED_RFID, HIGH);   // turn the LED off (HIGH turns it off)
}

//Recieve a byte (charaacter) of data from the user- this times out if nothing is entered
char getInputByte(uint32_t timeOut) {                 // Get a single character
  char readChar = '?';                                // Variable for reading in character
  uint32_t sDel = 0;                                  // Counter for timing how long to wait for input
  while (serial.available() == 0 && sDel < timeOut) { // Wait for a user response
    delay(1);                                         // Delay 1 ms
    sDel++;                                           // Add to the delay count
  }
  if (serial.available()) {                           //If there is a response then perform the corresponding operation
    readChar = serial.read();                         //read the entry from the user
  }
  return readChar;                                    //Return the value. It will be "?" if nothing is recieved.
}

//Recieve a string of data from the user- this times out if nothing is entered
String getInputString(uint32_t timeOut) {             // Get a string from the user
  String readStr = "";                                // Define an empty string to work with
  uint32_t sDel = 0;                                  // Counter for timing how long to wait for input
  while (serial.available() == 0 && sDel < timeOut) { // Wait for a user response
    delay(1);                                         // Delay 1 ms
    sDel++;                                           // Add to the delay count
  }
  if (serial.available()) {                           // If there is a response then read in the data
    delay(40);                                        // long delay to let all the data sink into the buffer
    readStr = serial.readString();                    // read the entry from the user
  }
  return readStr;                                     //Return the string. It will be "" if nothing is recieved.
}



//CLOCK FUNCTIONS///////////

//Get date and time strings and cobine into one string
String showTime() { //Get date and time strings and cobine into one string
  currentDateTime = String(rtc.stringDateUSA()) + " " + String(rtc.stringTime()); 
  return currentDateTime;
}

//Function to get user data for setting the clock
void inputTime() {                               // Function to set the clock
  serial.println("Enter mmddyyhhmmss");          // Ask for user input
  String timeStr = getInputString(20000);        // Get a string of data and supply time out value
  if (timeStr.length() == 12) {                  // If the input string is the right length, then process it
    //serial.println(timeStr);                   // Show the string as entered  
    byte mo = timeStr.substring(0, 2).toInt();   //Convert two ascii characters into a single decimal number
    byte da = timeStr.substring(2, 4).toInt();   //Convert two ascii characters into a single decimal number
    byte yr = timeStr.substring(4, 6).toInt();   //Convert two ascii characters into a single decimal number
    byte hh = timeStr.substring(6, 8).toInt();   //Convert two ascii characters into a single decimal number
    byte mm = timeStr.substring(8, 10).toInt();  //Convert two ascii characters into a single decimal number
    byte ss = timeStr.substring(10, 12).toInt(); //Convert two ascii characters into a single decimal number
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
   blipLED(30);                               // blink to indicate wakeup
   rtc.writeRegister(0x02, 0);                // clear clock flags to turn off alarm.
}

// Sleep and wake up using a 32-hertz timer on the real time clock 
void sleepTimer(uint16_t pCount, byte pRemainder){ // Sleep and wake up using a 32-hertz timer on the real time clock 
   rtc.writeRegister(0x02, 0);       // write a zero to the flags register to clear all flags.
   rtc.setTimer(pCount);             // set timer countdown (32-hertz timer) 
   rtc.enableTimerINT(1);            // enable the clock interrupt output
   rtc.setCTRL1Register(B10000010);  // set control register to enable a 32 Hertz timer.
   lpSleep();                        // call sleep funciton (you lose USB communicaiton here)
   //blipLED(20);                    // blink indicator - processor reawakened
   delay(pRemainder);                // additional delay for accuracy
   rtc.enableTimerINT(0);            // disable the clock interrupt output
   rtc.writeRegister(0x02, 0);       // write a zero to clear all interrupt flags.
}


////FLASH MEMORY FUNCTIONS////////////////////

//Enable the flash chip
void flashOn(void) {              // Enable the flash chip
  SPI.begin();                    // Enable SPI communication for Flash Memory
  pinMode(FlashCS, OUTPUT);       // Chip select pin for Flash memory set to output
  digitalWrite(SDselect, HIGH);   // make sure the SD card select is off
  digitalWrite(FlashCS, LOW);     // activate Flash memory
  SPI.setClockDivider(SPI_CLOCK_DIV16); // slow down the SPI for noise and signal quality reasons.
}

//Disable the flash chip
void flashOff(void) {           // Disable the flash chip
  SPI.end();                    // turn off SPI
  digitalWrite(FlashCS, HIGH);  // deactivate Flash memory
}

// Read a single byte from flash memory
byte readFlashByte(unsigned long bAddr) {   // read a single byte from flash memory
  flashOn();                                // activate flash chip
  SPI.transfer(0x03);                       // opcode for low freq read
  SPI.transfer((bAddr >> 16) & 0xFF);       // first of three address bytes
  SPI.transfer((bAddr >> 8) & 0xFF);        // second address byte
  SPI.transfer(bAddr & 0xFF);               // third address byte
  byte fByte = SPI.transfer(0);             // finally, read the byte
  flashOff();                               // deactivate flash chip
  return fByte;                             // return the byte that was read
}

void writeFlashByte(uint32_t wAddr, byte wByte) { // write a single byte from flash memory
  flashOn();                                      // activate flash chip
  SPI.transfer(0x58);                             // opcode for read modify write
  SPI.transfer((wAddr >> 16) & 0xFF);             // first of three address bytes
  SPI.transfer((wAddr >> 8) & 0xFF);              // second address byte
  SPI.transfer(wAddr & 0xFF);                     // third address byte
  SPI.transfer(wByte);                            // finally, write the byte
  flashOff();                                     // deactivate flash chip
  delay(20);                                      // delay to allow processing
} 

// Get the address counter for the flash memory
unsigned long getFlashAddr() {      // get the address counter for the flash memory from page 1 address 
  char gAddr[3] = {0, 0, 0};        // create a character array for holding data
  readFlashArray(0x400, gAddr, 3);  // read from flash
  unsigned long fAddr1 = (gAddr[0]<<16) + (gAddr[1]<<8) + gAddr[2]; // Combine three bytes into a long
  return fAddr1;                    // return the address.
}

//Write a data array to a specified memory address on the flash chip
unsigned long writeFlashArray(unsigned long wAddr, char *cArr, byte nchar) {
  flashOn();                           // activate flash chip
  SPI.transfer(0x58);                  // opcode for read modify write
  SPI.transfer((wAddr >> 16) & 0xFF);  // first of three address bytes
  SPI.transfer((wAddr >> 8) & 0xFF);   // second address byte
  SPI.transfer(wAddr & 0xFF);          // third address byte
  for (int n = 0; n < nchar; n++) {    // loop throught the bytes
    SPI.transfer(cArr[n]);             // finally, write the byte
    }
  flashOff();                          // Shut down 
  delay(20);                           // This delay allows writing to happen - maybe it can be removed if the chip is not read immediately after       
}

void readFlashArray(unsigned long addr, char *carr, byte nchar) {  // read a single byte from flash memory
  flashOn();                              // activate flash chip
  SPI.transfer(0x03);                     // opcode for low freq read
  SPI.transfer((addr >> 16) & 0xFF);      // first of three address bytes
  SPI.transfer((addr >> 8) & 0xFF);       // second address byte
  SPI.transfer(addr & 0xFF);              // third address byte
  for (int n = 0; n < nchar; n++) {       // loop throught the bytes
      carr[n] = SPI.transfer(0);          // finally, write the byte
  }
  flashOff();                             // deactivate flash chip
}

unsigned long advanceFlashAddress(unsigned long fAddr, byte steps) {    // update flash address - should only be used when logging RFID data 
  unsigned int bAddress = fAddr & 0x03FF;         // and with 00000011 11111111 to isolate byte address
  bAddress = bAddress + steps;                    // number of new bytes added (e.g. RFID (5) + RFcircuit (1) + date/time (6) = 12)
  if (bAddress > 500) {                           // If we are near a page break - beyond byte address 500...
    fAddr = (fAddr & 0xFFFFC00) + 0x0400;         // ...set byte address to zero and add 1 to the page address
  } else {                                        // otherwise... 
    fAddr = (fAddr & 0xFFFFC00) + bAddress;       // ...just add to the byte address
  }
  char fArray[3] = {((fAddr>>16)&0xFF), ((fAddr>>8)&0xFF), (fAddr&0xFF)};  //Format address as array
  writeFlashArray(0x400, fArray, 3);              //Write the array
  return fAddr;                                   //Return new address value
}

// Input the device ID and write it to flash memory
void inputID() {                                         // Function to input and set feeder ID
  serial.println("Enter four alphanumeric characters");  // Ask for user input
  String IDStr = getInputString(10000);                  // Get input from user (specify timeout)
  if (IDStr.length() == 4) {                             // if the string is the right length...
    deviceID[0] = IDStr.charAt(0);                       // Parse the bytes in the string into a char array
    deviceID[1] = IDStr.charAt(1);
    deviceID[2] = IDStr.charAt(2);
    deviceID[3] = IDStr.charAt(3);
    writeFlashArray(0x404, deviceID, 4);                 // Write the array to flash
  } else {
    serial.println("Invalid ID entered");                // error message if the string is the wrong lenth
  }
}

//Display backup memory
void dumpMem(byte doAll) {                        // Display backup memory
  unsigned long curAddr = 0x00000800;             // first address for stored data
  unsigned long fAddressEnd = getFlashAddr();     // last address for stored data
  char BA[12];                                    // byte array for holding data
  static char dataLine[40];                       // make an array for writing data lines
  serial.println("Displaying backup memory.");    // Message
  if(doAll) fAddressEnd = ((528 << 10) + 512);    // If we are dumping EVERYTHING, calculate the last possible flash address
  serial.print("last flash memory address: ");    // Message
  serial.println(fAddressEnd, BIN);               // Message
  while (curAddr < fAddressEnd) {                 // Escape long memory dump by entering a character
    if (serial.available()) {                     // If a charcter is entered....
      serial.println("User exit");                // ...print a message...
      break;                                      // ...and exit the loop    
    }
    readFlashArray(curAddr, BA, 12);              // read a line of data (12 bytes)    
    //convert the array to a string  
    sprintf(dataLine, "%02X%02X%02X%02X%02X, %d, %02d/%02d/%02d %02d:%02d:%02d",
         BA[0], BA[1], BA[2], BA[3], BA[4], BA[5], BA[6], BA[7], BA[8], BA[9], BA[10], BA[11]);
    serial.println(dataLine);                      // print the string
    curAddr = curAddr + 12;                        // add 12 to the byte address     
    if((curAddr & 0x000003FF) > 500) {             // Check to see if we are near the end of a memory page
      curAddr = (curAddr & 0xFFFFC00) + 0x0400;   // if so advance a page set byte address to zero and add 1 to the page address
    } 
  }
}

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

// Remove a file on the SD card
void SDremoveFile(String killFile) {   // Remove a file on the SD card 
  //serial.println("killing file");    // Message
  SDstart();                           // Enable SD
  SD.remove(killFile);                 // Delete the file.
  SDstop();                            // Disable SD
}

// Write an entire string of data to a file on the SD card
bool SDwriteString(String writeStr, String writeFile) {  // Write an entire string 
  SDstart();                                             // Enable SD
  bool success = 0;                                      // valriable to indicate success of operation
  File dFile = SD.open(writeFile, FILE_WRITE);           // Initialize the SD card and open the file "datalog.txt" or create it if it is not there.
  if (dFile) {                                           // If the file is opened successfully...
    dFile.println(writeStr);                             // ...write the string...
    dFile.close();                                       // ...close the file...
    success = 1;                                         // ...note success of operation...
    //serial.println("SD Write OK");                     // success message
  } //else {                                           
    //serial.println("SD Write fail");                   // fail message
  //}
  SDstop();                                              // Disable SD
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
