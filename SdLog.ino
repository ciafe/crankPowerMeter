
#include "FS.h"                // SD Card ESP32
#include "SD_MMC.h"            // SD Card ESP32

//#define SD_DEBUG_INFO

/* File name in the SD (max. 7 characters, one is reserved for the counter) */
#define SD_FILE_NAME    "/Log0000.csv"
      
#define FILE_COLUMNS    "Time;degsec;cadence;angle;degreesRev;force;powerRev;cadenceAvg;powerAvg;numberRev"


File    sd_file;
boolean sd_card_present = false;
boolean valid_file_created = false;

boolean Logging_Init_SD(void)
{
    if(!SD_MMC.begin()){
      #ifdef SD_DEBUG_INFO
      Serial.println("Card Mount Failed");
      #endif
      sd_card_present = false;
    }
    else
    {
      uint8_t cardType = SD_MMC.cardType();

      if(cardType == CARD_NONE)
      {
          #ifdef SD_DEBUG_INFO
          Serial.println("No SD card attached");
          #endif
          sd_card_present = false;
      }
      else
      {
          #ifdef SD_DEBUG_INFO
         Serial.println("SD card found!");
         #endif
         sd_card_present = true;
      }
    }
    return(sd_card_present);
}

boolean Logging_CreateNewLoggingFile(void)
{
    char file_name_string[] = SD_FILE_NAME;
    unsigned char valid_file_created = 0;
    unsigned char data_temp;
    boolean result = false;

    if (sd_card_present)
    {
      #ifdef SD_DEBUG_INFO
      Serial.print("File: ");
      Serial.println(file_name_string);
      #endif
      /* Look for a valid name, not existing in the SD, and create it */
      for (int i=0; i < 9; i++)
      {
          file_name_string[7] = i + '0';
          if (!SD_MMC.exists(file_name_string))
          {
              sd_file = SD_MMC.open(file_name_string, FILE_WRITE);
              #ifdef SD_DEBUG_INFO
              Serial.print("File ");
              Serial.print(file_name_string);
              Serial.println(" created and opened.");
              #endif
              valid_file_created = true;
              break;
          }
      }
      /* If file has been created, add initial data */
      if (valid_file_created == true)
      {
          if (sd_file != NULL)
          {
              #ifdef SD_DEBUG_INFO
              Serial.println("Write header info");
              #endif
              sd_file.println(FILE_COLUMNS);    /* Write columns information */
              sd_file.flush();                  /* Write data to the SD file */
              result = true;
          }
          else
          {
              #ifdef SD_DEBUG_INFO
              Serial.println("Problem detected with the opened file.");
              #endif
              result = false;
          }
      }
      else
      {
          #ifdef SD_DEBUG_INFO
          Serial.println("ERROR! All files already exist in the SD.");
          Serial.println("Remove any of them an restart measurement.");
          #endif
          result = false;
      }
    }
    return(result);
}

void Logging_StoreMeasurementData(void)
{

    #ifdef SD_DEBUG_INFO
    Serial.println("New data stored to SD");
    #endif

    if (true)//(valid_file_created == true)
    {
      /* write the date and time of the slot */
      sd_file.print(lastUpdateSensor);
      sd_file.print(";");
      sd_file.print(dps, 3);
      sd_file.print(";");
      sd_file.print(cadence, 3);
      sd_file.print(";");
      sd_file.print(crankAngle, 3);
      sd_file.print(";");
      sd_file.print(revElapsedDegrees);
      sd_file.print(";");
      sd_file.print(force);
      sd_file.print(";");
      sd_file.print(revPower, 2);
      sd_file.print(";");
      sd_file.print(avgCadence, 3);
      sd_file.print(";");
      sd_file.print(avgPower, 2);
      sd_file.print(";");
      sd_file.print(totalCrankRevs);
      sd_file.println(";");
  
      /* Assure all data is written to the SD file */
      sd_file.flush();   
    }
    else{
      #ifdef SD_DEBUG_INFO
      Serial.println("Storage skipped...");
      #endif
    }
}

void Logging_CloseSDFile(void)
{
    if (true)//(valid_file_created == true)
    {
      /* Write final test information */
      sd_file.close();                  /* Write data to the SD file and close */
  
      #ifdef SD_DEBUG_INFO
      Serial.println("SD file closed.");
      #endif
    }
}
