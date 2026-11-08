#include "sd_writer.h"
#include "TimeLib.h"


SDWriter sd_writer;

/*
  Initialising SDWriter, and file opening functions.

  File functions return the following error codes
  Error codes:
    0 - Everything works as intended
    1 - SD card reader failed to open
    2 - User tries to either open an already opened file, or
        write to nonexisting file
*/

/*
  SdFat calls this when a file is CREATED (all three stamps - create, access, write -
  are set from it) and again on every sync() */
static void sdDateTimeCallback(uint16_t *date, uint16_t *time){
  time_t t = now();

  if (year(t) < 1980){
    *date = FS_DATE(1980, 1, 1);
    *time = FS_TIME(0, 0, 0);
    return;
  }

  *date = FS_DATE(year(t), month(t), day(t));
  *time = FS_TIME(hour(t), minute(t), second(t));
}

uint32_t SDWriter::countFilesInDirectory(const char * dirName){
  /*
    Count files in a directory. Each entry must be closed before openNext(), or it
    stops after the first file. A local handle, so an open LogFile is untouched.
  */
  uint32_t fileCount = 0;
  File directory;
  if (!directory.open(dirName, O_RDONLY)) return 0;

  File entry;
  while (entry.openNext(&directory, O_RDONLY)) {
    if (!entry.isHidden() && !entry.isDir()) {
      fileCount++;
    }
    entry.close();
    IWatchdog.reload();  // a card with hundreds of readings must not trip the watchdog
  }
  directory.close();
  return fileCount;
}

bool SDWriter::begin(void){
  /*
    Start up the SD writer. Returns 0 if everything works as intended. 
  */
  if (debug_serial)
    Serial.println("Initializing SD card!");

  /*
    The SD card sits on SPI1 (PA5/PA6/PA7) together with the IMU. Both chip
    selects are driven high before the bus is started, so neither slave
    answers while the other is being addressed. setMOSI/setMISO/setSCLK must
    be called before SPI.begin().
  */
  pinMode(SPI_CS_IMU_PIN, OUTPUT);
  digitalWrite(SPI_CS_IMU_PIN, HIGH);
  pinMode(SPI_CS_SD_PIN, OUTPUT);
  digitalWrite(SPI_CS_SD_PIN, HIGH);

  SPI.setMOSI(SPI_MOSI_PIN);
  SPI.setMISO(SPI_MISO_PIN);
  SPI.setSCLK(SPI_SCK_PIN);
  SPI.begin();

  // To enable timestamped files
  FsDateTime::setCallback(sdDateTimeCallback);

  SD_fail = !SD.begin(SdSpiConfig(SPI_CS_SD_PIN, SHARED_SPI, SD_SCK_MHZ(SD_SPI_MHZ), &SPI));

  if (SD_fail && debug_serial){
    // sdErrorCode/sdErrorData say whether the sd-card never answered CMD0 (wiring,
    // CS, no sd-card) or failed later in the init handshake (sd-card/speed issue).
    Serial.print("SD begin failed, error 0x");
    Serial.print(SD.sdErrorCode(), HEX);
    Serial.print(" data 0x");
    Serial.println(SD.sdErrorData(), HEX);
  }

  if (!SD_fail && (WIO_MODE == BUOY_MODE)){
    active = true;

    // We create the necessary directories if not already present
    if (!SD.exists("readings")){
      SD.mkdir("readings");
    } else if (SD.exists("readings")) {
      //If readings directory is present, we count 
      // the files in said directory to avoid writing to 
      // the same file twice
      if (debug_serial){
        Serial.println("Counting files");
      }
      
      logCount = countFilesInDirectory(readings_dir);
      //Count amount of files in reading directory to ensure readability
      if (debug_serial){
        Serial.print("Num files: ");
        Serial.println(logCount);
        Serial.println("\n");
      }
    } 
    if (!SD.exists("messages")){
      SD.mkdir("messages");
    }

  } else if (!SD_fail && (WIO_MODE == BST_MODE)){
    if (!SD.exists("transmissions")){
      SD.mkdir("transmissions");
    } else {
      // We only need to count if the directory is already present
      transmissionsCount = countFilesInDirectory("transmissions/");
    }
    if (!SD.exists("debugFiles")){
      SD.mkdir("debugFiles");
      SD.mkdir("debugFiles/NotecardResets");
      SD.mkdir("debugFiles/BSTHeartbeats");
      SD.mkdir("debugFiles/NotehubSyncs");
      SD.mkdir("debugFiles/buoyComms");
      SD.mkdir("debugFiles/buoyRescue");
    } else {
      debugInfoLogCount[DEBUG_MODE_NOTECARD_RESET] = countFilesInDirectory("debugFiles/NotecardResets/");
      debugInfoLogCount[DEBUG_MODE_BST_HEARTBEAT]  = countFilesInDirectory("debugFiles/BSTHeartbeats/");
      debugInfoLogCount[DEBUG_MODE_NOTEHUB_SYNC]   = countFilesInDirectory("debugFiles/NotehubSyncs/");            
      debugInfoLogCount[DEBUG_MODE_BUOY_COMM]      = countFilesInDirectory("debugFiles/buoyComms/");    
      debugInfoLogCount[DEBUG_MODE_BUOY_RESCUE]    = countFilesInDirectory("debugFiles/buoyRescue/");          
    }
    //Count amount of files in reading directory to ensure readability
  }

  delay(100);
  return SD_fail;
}

int8_t SDWriter::startDebugging(int MODE){
  if (debug_SD){
    if (!SD_fail && !debugOpened){
      debugOpened = true;
      char debugFileName[128];
      if (MODE == DEBUG_MODE_NOTECARD_RESET){
        sprintf(debugFileName, "debugFiles/NotecardResets/log%09d.txt", debugInfoLogCount[DEBUG_MODE_NOTECARD_RESET]++);
      } else if (MODE == DEBUG_MODE_BST_HEARTBEAT){
        sprintf(debugFileName, "debugFiles/BSTHeartbeats/log%09d.txt", debugInfoLogCount[DEBUG_MODE_BST_HEARTBEAT]++);
      } else if (MODE == DEBUG_MODE_NOTEHUB_SYNC){
        sprintf(debugFileName, "debugFiles/NotehubSyncs/log%09d.txt", debugInfoLogCount[DEBUG_MODE_NOTEHUB_SYNC]++);
      } else if (MODE == DEBUG_MODE_BUOY_COMM){
        sprintf(debugFileName, "debugFiles/buoyComms/log%09d.txt", debugInfoLogCount[DEBUG_MODE_BUOY_COMM]++);
      } else if (MODE == DEBUG_MODE_BUOY_RESCUE){
        sprintf(debugFileName, "debugFiles/buoyRescue/log%09d.txt", debugInfoLogCount[DEBUG_MODE_BUOY_RESCUE]++);
      } else {
        //Unsupported mode
        return 3;
      }

      debugFile = SD.open(debugFileName, FILE_WRITE);

    } else if (SD_fail){
      return 1;
    } else {
      return 2;
    }
  }
  
  return 0;
}


// We avoid using Arduino strings for most operations
int8_t SDWriter::startLogging(String filename){
  int8_t state = startLogging(filename.c_str());
  return state;
}

int8_t SDWriter::startLogging(const char * filename){
  /*
    We check if the SD writer is in the correct mode to start new operations
  */
  if ((mode == ORB_SD_INACTIVE) && (!SD_fail)){
    mode   = ORB_SD_WRITE_MODE;
    LogFile = SD.open(filename, FILE_WRITE);

    // Only reading logs bump logCount, which names the next reading file and seeds reading_ID.
    if (strncmp(filename, readings_dir, sizeof(readings_dir) - 1) == 0){
      logCount++;
    }
    return 0;

  } else if (SD_fail) {
    return 1; 
  } else {
    return 2;
  }

}

int8_t SDWriter::startReading(const char * filename){
  /*
    If the SD writer is inactive, start reading operation. Otherwise pass with error code
  */
  if ((mode == ORB_SD_INACTIVE) && (!SD_fail)){
    mode   = ORB_SD_READ_MODE;
    LogFile = SD.open(filename, FILE_READ);
    Serial.println("FILE OPENED");
    numLines = 0;
    return 0;
  } else if (SD_fail) {
    return 1; 
  } else {
    return 2;
  }
}

int8_t SDWriter::startReading(String filename){
  int8_t state = startReading(filename.c_str());
  return state;
}


// File writing functions
int8_t SDWriter::logString(const char * line){
  if ((mode == ORB_SD_WRITE_MODE) && (!SD_fail)){
    if (debug_serial){
      Serial.println("Writing the following line to file:");
      Serial.println(line);
    }
    LogFile.println(line);
    LogFile.sync();
    return 0;
  } else if (SD_fail) {
    return 1;
  } else {
    return 2;
  }
}

int8_t SDWriter::logString(String line){
  int8_t state = logString(line.c_str());
  return state;
}

int8_t SDWriter::logString(const byte * line, const uint8_t messageSize){
  char _line[messageSize];
  memcpy(_line, line, messageSize);
  int8_t state = logString(_line);
  return state;
}

// RSSI/SNR only works on received signals
int8_t SDWriter::logSignalInfo(uint32_t RSSI, uint32_t SNR){
  char msg[32];
  sprintf(msg, "RSSI: %ld SNR: %ld\n", RSSI, SNR);
  int8_t ecode = logString(msg);
  return ecode;
}

// The 'b' separator is included to indicate a new byte as to ensure unambigous log files
int8_t SDWriter::logByteArray(byte* array, int length){
  if ((mode == ORB_SD_WRITE_MODE) && (!SD_fail)){
    if (debug_serial)
      Serial.println("Writing the following byte array to file:");
    for (int i = 0; i < length; i++){
      if (debug_serial){
        Serial.print(array[i]);
        Serial.print('b');
      }
      LogFile.print(array[i]);
      LogFile.print('b');
    }
    if (debug_serial)
      Serial.println();
    
    LogFile.println();
    LogFile.sync();
    IWatchdog.reload();
    // LogFile.write(array, length);
    return 0;
  } else if (SD_fail) {
    return 1;
  } else {
    return 2;
  }
}

// File reading function

int8_t SDWriter::readFile(void){
  /*
    Prepare the open logfile for line-by-line reading.
    The file is no longer copied into RAM: this counts the lines into numLines and
    rewinds, and the readers then fetch one line at a time with readLine().
  */
  if ((mode == ORB_SD_READ_MODE) && (!SD_fail)){
    rewindRead();
    fileLine tmp;
    while (readLine(tmp) == 0){
      numLines++;
    }
    rewindRead();
    IWatchdog.reload();
    return 0;
  } else {
    return 1;
  }
}


int8_t SDWriter::readLine(fileLine & out){
  /*
    Read the next line of the open logfile into out, without the line ending.
    A line too long for out.line is truncated and the rest of it skipped, so the
    next call starts on the following line.
    Returns 0 when a line was read, 1 at end of file, on a read error, or when no
    file is open for reading.
  */
  out = {"",0};
  if ((mode != ORB_SD_READ_MODE) || SD_fail){
    return 1;
  }
  // fgets drops '\r', stops after '\n' and returns 0 at end of file, -1 on error
  int n = LogFile.fgets(out.line, sizeof(out.line));
  if (n <= 0){
    out.line[0] = '\0';
    return 1;
  }
  if (out.line[n - 1] == '\n'){
    out.line[--n] = '\0';
  } else {
    int c;
    while (((c = LogFile.read()) >= 0) && (c != '\n')){}
  }
  out.length = n;
  return 0;
}


void SDWriter::rewindRead(void){
  if ((mode == ORB_SD_READ_MODE) && (!SD_fail)){
    LogFile.rewind();
  }
}


int8_t SDWriter::printFileToSerial(void){
  rewindRead();
  fileLine tmp;
  for (uint16_t line_num = 0; (line_num < numLines) && (readLine(tmp) == 0); line_num++ ){
    Serial.println(tmp.line);
    delay(500);
    IWatchdog.reload();
  }
  return 0;
}

// Functions to stop writing/reading a file. Important to call before opening a new file. 

int8_t SDWriter::closeLog(void){
  if ((mode == ORB_SD_WRITE_MODE) && (!SD_fail)){
    LogFile.close();
    mode = ORB_SD_INACTIVE;
    numLines = 0;
    return 0;
  } else if (SD_fail) {
    return 1;
  } else {
    return 2; 
  }
}


int8_t SDWriter::closeRead(void){
  if ((mode == ORB_SD_READ_MODE) && (!SD_fail)){
    LogFile.close();
    numLines = 0;
    mode = ORB_SD_INACTIVE;
    return 0;
  } else if (SD_fail) {
    return 1;
  } else {
    return 2;
  }
}



void SDWriter::closeDebug() {
    if (debugFile) {
        debugFile.close();
        debugOpened = false;
    }
}

// Boot down to preserve power
int8_t SDWriter::shutdown(void){
  if (active){
    SD.end();
    active = false;
  }
  return 0;
}


// File cleaning
int8_t SDWriter::deleteFile(const char * filename){
  if (SD.exists(filename)){
    SD.remove(filename);
    return 0;
  } else {
    return 1;
  }
}


/*
debugSerialPrint is an alias for Serial print 
which also dumps to debug file if open
Many, likely redundant overloads
*/

void SDWriter::debugSerialPrint(const char * line){
  if (debug_serial){
    Serial.print(line);
  }
  if (debug_SD && debugOpened){
    debugFile.print(line);
    debugFile.sync();
  }
}

void SDWriter::debugSerialPrintln(const char * line){
  if (debug_serial){
    Serial.println(line);
  }
  if (debug_SD && debugOpened){
    debugFile.println(line);
    debugFile.sync();
  }
}


void SDWriter::debugSerialPrint(float number){
  if (debug_serial){
    Serial.print(number);
  }
  if (debug_SD && debugOpened){
    debugFile.print(number);
    debugFile.sync();
  }
}

void SDWriter::debugSerialPrintln(float number){
  if (debug_serial){
    Serial.println(number);
  }
  if (debug_SD && debugOpened){
    debugFile.println(number);
    debugFile.sync();
  }
}


void SDWriter::debugSerialPrint(float number, int digits){
  if (debug_serial){
    Serial.print(number, digits);
  }
  if (debug_SD && debugOpened){
    debugFile.print(number, digits);
    debugFile.sync();
  }
}


void SDWriter::debugSerialPrintln(float number, int digits){
  if (debug_serial){
    Serial.println(number, digits);
  }
  if (debug_SD && debugOpened){
    debugFile.println(number, digits);
    debugFile.sync();
  }
}


int8_t SDWriter::debugByteArray(byte* array, int length){
  if (debugOpened && !SD_fail){
    debugSerialPrintln("Writing the following byte array to file:");
    for (int i = 0; i < length; i++){
      if (debug_serial)
        Serial.print(array[i]);
      debugFile.print('b');
      debugFile.print(array[i]);
    }
    if (debug_serial)
      Serial.println();
    debugFile.println();
    debugFile.sync();
    return 0;
  } else if (SD_fail) {
    return 1;
  } else {
    return 2;
  }
}