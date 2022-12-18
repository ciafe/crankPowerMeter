

 
#include "ble.h"

#ifndef DISABLE_BLE
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLEAdvertising.h>
#include <BLE2902.h>

const int16_t minPowLev = 0; // power level range settings, no app cares about this
const int16_t maxPowLev = 500;
const int16_t minPowInc = 1;
const int16_t maxResLev = 40;// resistance level range settings, no app cares about this
uint16_t speedOut = 100;
int16_t powerOut = 100;
int16_t powerOut_filter[BLE_POWER_FILTER_SAMPLES];
int16_t powerOut_filter_pos = 0;

#define fitnessMachineService BLEUUID((uint16_t)0x1826) // fitness machine service uuid, as defined in gatt specifications

// required characteristics
BLECharacteristic fitnessMachineFeatureCharacteristics(BLEUUID((uint16_t)0x2ACC), BLECharacteristic::PROPERTY_READ);
BLECharacteristic indoorBikeDataCharacteristic(BLEUUID((uint16_t)0x2AD2), BLECharacteristic::PROPERTY_NOTIFY);
//BLECharacteristic resistanceLevelRangeCharacteristic(BLEUUID((uint16_t)0x2AD6), BLECharacteristic::PROPERTY_READ);
//BLECharacteristic powerLevelRangeCharacteristic(BLEUUID((uint16_t)0x2AD8), BLECharacteristic::PROPERTY_READ);
//BLECharacteristic fitnessMachineControlPointCharacteristic(BLEUUID((uint16_t)0x2AD9), BLECharacteristic::PROPERTY_INDICATE | BLECharacteristic::PROPERTY_WRITE);
BLECharacteristic fitnessMachineStatusCharacteristic(BLEUUID((uint16_t)0x2ADA), BLECharacteristic::PROPERTY_NOTIFY);
BLEAdvertisementData advertisementData = BLEAdvertisementData();


// no app cares about these flags, but they are there just in case
const uint16_t indoorBikeDataCharacteristicDef = 0b0000000001000100; // flags for indoor bike data characteristics - power and cadence
const uint32_t fitnessMachineFeaturesCharacteristicsDef = 0b00000000000000000100000010000010; // flags for Fitness Machine Features Field - cadence, resistance level and inclination level
const uint32_t targetSettingFeaturesCharacteristicsDef = 0b00000000000000000010000000001100;  // flags for Target Setting Features Field - power and resistance level, Indoor Bike Simulation Parameters 

uint8_t indoorBikeDataCharacteristicData[8] = {        // values for setup - little endian order
  (uint8_t)(indoorBikeDataCharacteristicDef & 0xff),  
  (uint8_t)(indoorBikeDataCharacteristicDef >> 8), 
  (uint8_t)(speedOut & 0xff),
  (uint8_t)(speedOut >> 8), 
  (uint8_t)(speedOut & 0xff), 
  (uint8_t)(speedOut >> 8), 
  0x64, 
  0 
};
                                     
uint8_t fitnessMachineFeatureCharacteristicsData[8] = {  // values for setup - little endian order
   (uint8_t)(fitnessMachineFeaturesCharacteristicsDef & 0xff),
  (uint8_t)(fitnessMachineFeaturesCharacteristicsDef >> 8),
  (uint8_t)(fitnessMachineFeaturesCharacteristicsDef >> 16),
  (uint8_t)(fitnessMachineFeaturesCharacteristicsDef >> 24),
  (uint8_t)(targetSettingFeaturesCharacteristicsDef & 0xff),
  (uint8_t)(targetSettingFeaturesCharacteristicsDef >> 8),
  (uint8_t)(targetSettingFeaturesCharacteristicsDef >> 16),
  (uint8_t)(targetSettingFeaturesCharacteristicsDef >> 24) 
};

#endif

int16_t cadenceIn;
int16_t powerIn;

void ble_Setup(void) {

  #ifndef DISABLE_BLE
    BLEDevice::init(BLE_DEV_NAME);
    
    InitBLEServer();
    #endif

    for(uint8_t iloop = 0; iloop < BLE_POWER_FILTER_SAMPLES; iloop++)
    {
      powerOut_filter[iloop] = 0;
    }
    powerOut_filter_pos = 0;

}


void ble_startAdvertising(void) {

}

/*
 * Set up the power service
 */
void ble_setupPwr(void) {

}

/*
 * This service exists only to publish logs over BLE.
 */
void ble_setupLogger(void) {
#ifdef BLE_LOGGING

#endif
}

boolean ble_isConnected(void)
{
  return true;
}

/*
 * Publish the instantaneous power measurement.
 */
void ble_PublishPower(int16_t instantPwr, uint16_t cadence, uint32_t crankRevs, long millisLast) {
#ifndef DISABLE_BLE
  speedOut = (uint16_t)crankRevs;
  /* Perform average according config */
  powerOut_filter[powerOut_filter_pos] = instantPwr;
  powerOut_filter_pos = (powerOut_filter_pos + 1) % BLE_POWER_FILTER_SAMPLES;
  powerIn = 0;
  for(uint8_t iloop = 0; iloop < BLE_POWER_FILTER_SAMPLES; iloop++)
  {
    powerIn += powerOut_filter[iloop];
  }
  powerIn = powerIn/BLE_POWER_FILTER_SAMPLES;
  cadenceIn = cadence;
  

  //speedOut = (cadenceIn * 2.75 * 2.08 * 60*gears[gearIndex]) / 10;            // calculated speed, required by the specification
  indoorBikeDataCharacteristicData[2] = (uint8_t)(speedOut & 0xff);
  indoorBikeDataCharacteristicData[3] = (uint8_t)(speedOut >> 8);             // speed value with little endian order
  indoorBikeDataCharacteristicData[4] = (uint8_t)((cadenceIn * 2) & 0xff);        
  indoorBikeDataCharacteristicData[5] = (uint8_t)((cadenceIn * 2) >> 8);          // cadence value
  indoorBikeDataCharacteristicData[6] = (uint8_t)(constrain(powerIn, 0, 4000) & 0xff);
  indoorBikeDataCharacteristicData[7] = (uint8_t)(constrain(powerIn, 0, 4000) >> 8);    // power value, constrained to avoid negative values, although the specification allows for a sint16
    
  indoorBikeDataCharacteristic.setValue(indoorBikeDataCharacteristicData, 8);       // values sent
  indoorBikeDataCharacteristic.notify();                          // device notified
  #endif
}
/*
 * Publish the battery status measurement.
 */
void ble_PublishBatt(uint8_t battPercent) {
  //Serial.println("BLE DUMMY PUBLISH BATT");
}

#ifndef DISABLE_BLE
void InitBLEServer() {
  BLEServer *pServer = BLEDevice::createServer();

  const std::string fitnessData = { 0b00000001, 0b00100000, 0b00000000 };  // advertising data on "Service Data AD Type" - byte of flags (little endian) and two for Fitness Machine Type (little endian)
                                         // indoor bike supported
  advertisementData.setServiceData(fitnessMachineService, fitnessData);  // already includdes Service Data AD Type ID and Fitness Machine Service UUID
                                         // with fitnessData 6 bytes
  BLEService *pFitness = pServer->createService(fitnessMachineService);

  // added characteristics and descriptors - 2902 required for characteristics that notify or indicate
  
  pFitness->addCharacteristic(&indoorBikeDataCharacteristic);
  indoorBikeDataCharacteristic.addDescriptor(new BLE2902());
  pFitness->addCharacteristic(&fitnessMachineFeatureCharacteristics);

  BLE2902* descr = new BLE2902();
  descr->setIndications(1); // default indications on
  
  pFitness->addCharacteristic(&fitnessMachineStatusCharacteristic);
  fitnessMachineStatusCharacteristic.addDescriptor(new BLE2902());

  advertisementData.setFlags(ESP_BLE_ADV_FLAG_BREDR_NOT_SPT+ESP_BLE_ADV_FLAG_GEN_DISC); // set BLE EDR not supported and general discoverable flags, necessary for RGT
  pServer->getAdvertising()->addServiceUUID(fitnessMachineService);
  pServer->getAdvertising()->setAdvertisementData(advertisementData);
  pFitness->start();
  pServer->getAdvertising()->start();
  
  fitnessMachineFeatureCharacteristics.setValue(fitnessMachineFeatureCharacteristicsData, 8); // flags
}
#endif
