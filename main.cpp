/* mbed MicroGcontroller Library
 *  * Copyright (c) 2006-2013 ARM Limited
 *   *
 *    * Licensed under the Apache License, Version 2.0 (the "License");
 *     * you may not use this file except in compliance with the License.
 *      * You may obtain a copy of the License at
 *       *
 *        *     http://www.apache.org/licenses/LICENSE-2.0
 *         *
 *          * Unless required by applicable law or agreed to in writing, software
 *           * distributed under the License is distributed on an "AS IS" BASIS,
 *            * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *             * See the License for the specific language governing permissions and
 *              * limitations under the License.
 *               */

#include "mbed.h"
#include "ble/BLE.h"
#include "ButtonService.h"
#include "tofSensor.h"

#define MAX_DISTANCE 3000
#define DOOR_THRESHOLD 150
#define TIMESTAMP_ARRAY_SIZE 100
#define I2C_RESETTIME 2000 //in milliseconds


DigitalOut  led1(LED1);
DigitalOut  led2(LED2);
InterruptIn button(BUTTON1);
InterruptIn pin_16(p16, PullDown);
Serial pc(USBTX, USBRX, 115200);

uint16_t customServiceUUID = 0xA000;
uint16_t readCharUUID = 0xA005;

const static char     DEVICE_NAME[] = "Button";
static const uint16_t uuid16_list[] = {customServiceUUID};
bool isConnected = false;

uint16_t array_timestamps[TIMESTAMP_ARRAY_SIZE];
uint16_t timestamps_index = 0;

enum {
    RELEASED = 0,
    PRESSED,
    IDLE
};

static uint8_t buttonState = IDLE;

static uint8_t counter = 0;

static uint16_t tof_triggered_counter = 512;

static ButtonService *buttonServicePtr;

static uint8_t READ_FLAG = 0;
void buttonPressedCallback(void)
{
    /* Note that the buttonPressedCallback() executes in interrupt context, so it is safer to access
 *      * BLE device API from the main thread. */
    buttonState = PRESSED;
    counter++;
    led2 = !led2;
}

void buttonReleasedCallback(void)
{
    /* Note that the buttonReleasedCallback() executes in interrupt context, so it is safer to access
 *      * BLE device API from the main thread. */
    buttonState = RELEASED;
	led2 = !led2;
}

void disconnectionCallback(const Gap::DisconnectionCallbackParams_t *params)
{
    BLE::Instance().gap().startAdvertising();
    isConnected = false;
}

void connectionCallback(const Gap::ConnectionCallbackParams_t *params)
{
    isConnected = true;
}

void periodicCallback(void)
{
    led1 = !led1; /* Do blinky on LED1 to indicate system aliveness. */
    READ_FLAG = 1;
}

/*
 * Set Up characteristics
 *
 * */
static uint8_t readValue[2] = {0};
ReadOnlyArrayGattCharacteristic<uint8_t, sizeof(readValue)> readChar(readCharUUID, readValue, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY);
GattCharacteristic *characteristics[] = {&readChar};
GattService customService(customServiceUUID, characteristics, sizeof(characteristics) / sizeof(GattCharacteristic *));

void updateDataToCharacteristic(BLE &ble, uint16_t data){
     
    uint8_t data_[2];
    printf("%u:%u",data,data>>8);
    data_[0] = data>>8;
    data_[1] = (data<<8)>>8;
    
    ble.gattServer().write(readChar.getValueHandle(), data_, sizeof(uint8_t)*2);
}

/**
 *  * This function is called when the ble initialization process has failled
 *   */
void onBleInitError(BLE &ble, ble_error_t error)
{
    /* Initialization error handling should go here */
}

/**
 *  * Callback triggered when the ble initialization process has finished
 *   */
void bleInitComplete(BLE::InitializationCompleteCallbackContext *params)
{
    BLE&        ble   = params->ble;
    ble_error_t error = params->error;

    if (error != BLE_ERROR_NONE) {
        /* In case of error, forward the error handling to onBleInitError */
        onBleInitError(ble, error);
        return;
    }

    /* Ensure that it is the default instance of BLE */
    if(ble.getInstanceID() != BLE::DEFAULT_INSTANCE) {
        return;
    }

    ble.gap().onDisconnection(disconnectionCallback);
    ble.gap().onConnection(connectionCallback);
    /* Setup primary service */
//    buttonServicePtr = new ButtonService(ble, false /* initial value for button pressed */);
    ble.addService(customService);
    /* setup advertising */
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED | GapAdvertisingData::LE_GENERAL_DISCOVERABLE);
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LIST_16BIT_SERVICE_IDS, (uint8_t *)uuid16_list, sizeof(uuid16_list));
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LOCAL_NAME, (uint8_t *)DEVICE_NAME, sizeof(DEVICE_NAME));
    ble.gap().setAdvertisingType(GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED);
    ble.gap().setAdvertisingInterval(1000); /* 1000ms. */
    ble.gap().startAdvertising();

}



void appendCurrentTimeToList(void){
    if ( isConnected ) {
        printf("Sent 1 to phone");
        buttonServicePtr -> updateButtonState( timestamps_index+1 );
        timestamps_index = 0;
    } else {    
        uint16_t minutes = time(NULL)/60;   
        printf("Minutes since start = %u\n\r", (unsigned int)minutes);  
        array_timestamps[timestamps_index] = minutes;   
        timestamps_index++; 
        timestamps_index = timestamps_index % TIMESTAMP_ARRAY_SIZE;
    }
}

time_t prevSensor1Time = 0;
time_t prevSensor2Time = 0;
bool detectedSensor1 = false;
bool detectedSensor2 = false;


#define EXIT_ROOM -1
#define NONE_SENSE 0
#define ENTER_ROOM 1

#define DETECTED_THRESHOLD 5
int8_t aggregateMeasurements(uint32_t measurement1, uint32_t measurement2){
    
    time_t currentTime = time(NULL);
    int8_t status = NONE_SENSE;
    // Logic for Sensor 1 detecting something
    
    if (measurement1 <= MAX_DISTANCE && measurement1 > DOOR_THRESHOLD) { 
        
        detectedSensor1 = true;

    } else { 
        
        if (detectedSensor1) { 
        
            printf("Sensor 1 Detected! \n\r"); 
            detectedSensor1 = false;
            if(currentTime - prevSensor2Time < DETECTED_THRESHOLD){
                status += ENTER_ROOM;
                printf("Someone Entered Room? \n\r");
            }
            prevSensor1Time = currentTime;
        } 
    } 
    // Logic for Sensor 2 detecting something
    if (measurement2 <= MAX_DISTANCE && measurement2 > DOOR_THRESHOLD) {
        
        detectedSensor2 = true;
    
    } else {
        if (detectedSensor2) {
    
            printf("Sensor 2 Detected! \n\r");
            detectedSensor2 = false;
    
            if(currentTime - prevSensor1Time < DETECTED_THRESHOLD){
                status += EXIT_ROOM;
                printf("Someone Exited Room? \n\r");
            }
            prevSensor2Time = currentTime;
        }
    }

    return status; // If someone entered and exited at the same time return NON_SENSE
}


int main(void)
{
    led1 = 1;
    uint32_t measurement, measurement2;
    int8_t personPassingThrough = 0; 
    init_sensor(RANGE_SENSOR1);
    init_sensor(RANGE_SENSOR2); 
    Ticker ticker;
    set_time(0);
    ticker.attach(periodicCallback, 0.5);
    
    BLE &ble = BLE::Instance();
    ble.init(bleInitComplete);

    /* SpinWait for initialization to complete. This is necessary because the
 *      * BLE object is used in the main loop below. */
    while (ble.hasInitialized()  == false) { /* spin loop */ }
    
    uint32_t num_readings = 0;

    while (true) {
        
        if( READ_FLAG == 1){
            READ_FLAG=0;
                    
            measurement  = take_measurement(RANGE_SENSOR1);
            measurement2 = take_measurement(RANGE_SENSOR2);

            
            //if( num_readings %10 == 0){

                printf("Nreads: %d, 1: %lu, 2:%lu\n\r",num_readings , measurement, measurement2);
            //}
            if( measurement2 ==-1 || measurement > 500000000 || measurement ==-1 || measurement2==500000000 || measurement==1 ||measurement2==1){
                printf("Number of Reads: %d\n\r", num_readings);
                printf("Resetting Channel");
                reset_i2c_chn();
                wait_ms(I2C_RESETTIME);
                init_sensor(RANGE_SENSOR1);
                init_sensor(RANGE_SENSOR2);
                wait_ms(I2C_RESETTIME);
                continue; 
            }
            num_readings++;
            
            personPassingThrough = aggregateMeasurements(measurement, measurement2);
            if( personPassingThrough == EXIT_ROOM ){
                //buttonServicePtr -> updateButtonState( ++timestamps_index );
                updateDataToCharacteristic(ble, ++tof_triggered_counter);
                
                printf("Someone Exited Room\n\r");
            }else if( personPassingThrough == ENTER_ROOM ){
                //buttonServicePtr -> updateButtonState( ++timestamps_index );
                updateDataToCharacteristic(ble, ++tof_triggered_counter);
                printf("Someone Entered Room\n\r");
            }
        }
        ble.waitForEvent();

    }
}
