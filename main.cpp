/* mbed Microcontroller Library
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
DigitalOut  led1(LED1);
DigitalOut  led2(LED2);
InterruptIn button(BUTTON1);
InterruptIn pin_16(p16, PullDown);
Serial pc(USBTX, USBRX, 115200);

const static char     DEVICE_NAME[] = "Button";
static const uint16_t uuid16_list[] = {ButtonService::BUTTON_SERVICE_UUID};
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
    buttonServicePtr = new ButtonService(ble, false /* initial value for button pressed */);

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

int main(void)
{
    bool wasPersonDetected = false;
    led1 = 1;
    uint32_t measurement;
    
    init_sensor(RANGE_SENSOR1);
    
    Ticker ticker;
    set_time(0);
    ticker.attach(periodicCallback, 0.5);
    
    BLE &ble = BLE::Instance();
    ble.init(bleInitComplete);

    /* SpinWait for initialization to complete. This is necessary because the
 *      * BLE object is used in the main loop below. */
    while (ble.hasInitialized()  == false) { /* spin loop */ }
    
    while (true) {
        
        if( READ_FLAG == 1){
            READ_FLAG=0;

            measurement = take_measurement(RANGE_SENSOR1);

            printf("%lu\n\r", measurement);
            if(measurement <= MAX_DISTANCE && measurement > DOOR_THRESHOLD) {
                wasPersonDetected = true;
                //if flag is set, detected someone
                
                //buttonServicePtr->updateButtonState(measurement/10); 

            } else {
                if(wasPersonDetected) {
                    printf("PersonDetected\n\r!"); 
                    appendCurrentTimeToList();
                    wasPersonDetected = false;
                }
                //Set flag for door or no data
            }

        }
        ble.waitForEvent();

    }
}
