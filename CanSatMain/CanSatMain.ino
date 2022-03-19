#include <STM32FreeRTOS.h>
#include <SparkFunMPL3115A2.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> 
#include <SD.h>
#include <Wire.h>
#include <ArduCAM.h>
#include <SPI.h>
#include "memorysaver.h"


//TO DO: Handle Lora, save data to SD, implement light sensors and check orientation, see chute deployment (and confirmation)

//Constants
#define MIN_SAFE_ALTITUDE 100
#define ITER_COHERENCE_CONST 10
#define SHUTE_DEPLOY_ALTITUDE 10

//Components
MPL3115A2 MPL;
SFE_UBLOX_GNSS GPS;

//State Variables
float currentAltitude;
int isFalling=0;
float currentTemperature;
float currentPressure;
long currentLatitude;
long currentLongitude;
int shuteDeployed=0;

//useful variables
SemaphoreHandle_t MPLsemaphore;
int lastGpsIter=0;
int iterCoh=ITER_COHERENCE_CONST;
int currentAltitudePrevIter;

int ShuteDeploy(){}; // No idea yet on how to deploy the parachute
int backupShuteDeploy(){}; // No idea yet on how to deploy the parachute



void TaskCameraHandle( void *pvParameters __attribute__((unused)) ){
    Camloop();
};



void TaskSafetyTriggers( void *pvParameters __attribute__((unused)) ){

    if (xSemaphoreTake(MPLsemaphore, ((TickType_t) 5)) == pdTRUE ){
        currentAltitude=MPL.readAltitude();
        xSemaphoreGive(MPLsemaphore);
    }

    if(isFalling && currentAltitude<MIN_SAFE_ALTITUDE && !(shuteDeployed)){
        backupShuteDeploy();
    }

    if (currentAltitude<currentAltitudePrevIter){
        iterCoh--;
        if (!(iterCoh)){
            isFalling=1;
        }
    }
    else{
        iterCoh=ITER_COHERENCE_CONST;
    }


    if (isFalling && currentAltitude<SHUTE_DEPLOY_ALTITUDE){
        iterCoh--;
        if (!(iterCoh)){
            ShuteDeploy();
        }
    }
    
}



void TaskSensorsHandle( void *pvParameters __attribute__((unused)) ){

    if (xSemaphoreTake(MPLsemaphore, ((TickType_t) 5)) == pdTRUE ){
        currentAltitude=MPL.readAltitude();
        currentTemperature = MPL.readTemp();
        xSemaphoreGive(MPLsemaphore);
    }

    if(millis()-lastGpsIter>1000){
        lastGpsIter=millis();
        currentLatitude = GPS.getLatitude();
        currentLongitude = GPS.getLongitude();
    }

    Serial.println(currentAltitude);
    
    Serial.println(currentLatitude);
    
    Serial.println(currentLongitude);
    
    Serial.println(currentTemperature);
    
}



void setup() {

    Wire.begin();       ///< Setting up I2c

    Serial.begin(9600);
    
    //< Setting up the MPL3115A2 sensor

    if (MPL.begin() == false){
    Serial.println("MPL not detected");
    delay(1000);
    }

    MPL.setModeAltimeter();
    MPL.setOversampleRate(7);
    MPL.enableEventFlags();
    
    currentAltitudePrevIter = MPL.readAltitude();
    currentAltitude = currentAltitudePrevIter;

    if (MPLsemaphore == NULL ){   //may be overkill to make a semaphore for this but since i dont know how the particutal library functions are made better safe than sorry
        MPLsemaphore = xSemaphoreCreateMutex();
        if (MPLsemaphore){
        xSemaphoreGive(MPLsemaphore);
        }
    }


    //< Setting up the GPS

    if (GPS.begin() == false){
        Serial.println(F("GPS not detected"));
        delay(1000);
    }

    GPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
    GPS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR

    Camsetup();

  while (!Serial);

  xTaskCreate(
    TaskSafetyTriggers
    ,  (const portCHAR *)"SafetyTriggers"
    ,  128
    ,  NULL
    ,  3
    ,  NULL );

  xTaskCreate(
    TaskSensorsHandle
    ,  (const portCHAR *)"HandleSensors"
    ,  128
    ,  NULL
    ,  2
    ,  NULL );


  xTaskCreate(
    TaskCameraHandle
    ,  (const portCHAR *)"CameraHandle"
    ,  128
    ,  NULL
    ,  1
    ,  NULL );

  vTaskStartScheduler();
  Serial.println("No more RAM bro");
  while(1);
}



void loop(){}