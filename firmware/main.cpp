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
#include "ble/Gap.h"
#include "tofSensor.h"


#define TIMESTAMP_ARRAY_SIZE 100 
#define I2C_RESETTIME 50 //in milliseconds
#define BYTESSENT 2

#define TIMEDIVIDER 2 // Store time is Seconds/2

#define EXIT_ROOM -1
#define NONE_SENSE 0
#define ENTER_ROOM 1

#define MAX_DISTANCE 700 // Maximum distance detection threshold

#define DOOR_THRESHOLD 10 // minimum distance detection threshold

#define DETECTED_THRESHOLD 1 // Time threshold between detection in seconds

#define IGNORE_PRINTF

#ifdef IGNORE_PRINTF
#define printf(fmt, ...) (0)
#endif

DigitalOut  led1(LED1);
DigitalOut  led2(LED2);
DigitalOut  led4(LED4);
DigitalOut  led3(LED3);
//Serial pc(USBTX, USBRX, 115200);
InterruptIn btn1(p17, PullDown);
RawSerial serial(USBTX, USBRX, 115200);
uint16_t customServiceUUID = 0xA000;
uint16_t buttonCharUUID = 0xA001;
uint16_t ledCharUUID = 0xA002;
uint16_t readCharUUID = 0xA005;

const static char     DEVICE_NAME[] = "DONTCONNECT!";
static const uint16_t uuid16_list[] = {customServiceUUID};
bool isConnected = false;

uint16_t array_timestamps[TIMESTAMP_ARRAY_SIZE];

uint16_t timestamps_index = 0;

uint16_t endOfTx = 65535;

Ticker ticker;

static uint16_t tof_triggered_counter = 513;

void appendCurrentTimeToList(int status);
static uint8_t READ_FLAG = 0;
static uint8_t READY_TO_SENT = 0;

void button1Callback(void){
    printf("Button1 pressed");
   BLE::Instance().gap().startAdvertising();
}

void periodicCallback(void)

{
    if(isConnected)
    {
        //READ_FLAG=0;
        led4 = !led4;
        //return;
    }
    led1 = !led1; /* Do blinky on LED1 to indicate system aliveness. */
    READ_FLAG = 1;
}


void disconnectionCallback(const Gap::DisconnectionCallbackParams_t *params)
{
    led4 = 1 ;
    isConnected=false;
    READY_TO_SENT=0;
    printf("Disconnection Callback\n\r");
    BLE::Instance().gap().startAdvertising();
    //appendCurrentTimeToList(ENTER_ROOM);
}

void connectionCallback(const Gap::ConnectionCallbackParams_t *params)
{
    printf("Conenction Callback\n\r");
    isConnected = true;
    //updateDataToCharacteristic(endOfTx);
}
void timeoutCallback( Gap::TimeoutSource_t source)
{
    led4 = 1;
    isConnected = false;
    printf("TimeoutCallback \n\r");
    BLE::Instance().gap().startAdvertising();
}

/*
 * Set Up characteristics
 *
 * */
// Tof Characteristics
static uint8_t readValue[BYTESSENT] = {0};
ReadOnlyArrayGattCharacteristic<uint8_t, sizeof(readValue)> tofChar(readCharUUID, readValue, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY);
//// Led Characteristics
static bool initValueForLEDChar=false;
WriteOnlyGattCharacteristic<bool> ledChar(ledCharUUID, &initValueForLEDChar);
//// Button Characteristics
//static bool initValueForButtonChar=true;
//ReadOnlyGattCharacteristic<bool> buttonChar(buttonCharUUID, &initValueForButtonChar);

GattCharacteristic *characteristics[] = {&tofChar, &ledChar};
GattService customService(customServiceUUID, characteristics, sizeof(characteristics) / sizeof(GattCharacteristic *));

void writeCharCallback(const GattWriteCallbackParams *params){

    if(READY_TO_SENT==0){
        READY_TO_SENT=1;
    printf("wrote to led, ready to sen!\n\r");
    }else{
        printf("wrote to led, finish sending!\n\r");
        READY_TO_SENT=0;
        //BLE &ble = BLE::Instance();
        //ble.disconnect(Gap::REMOTE_USER_TERMINATED_CONNECTION);
    }
}


void updateDataToCharacteristic (uint16_t data){
    //uint16_t curr_seconds = time(NULL);    
    uint8_t data_[BYTESSENT];
    
    data_[0] = (data)>>8;
    data_[1] = ((data)<<8)>>8;
//    data_[0] = *data>>24;
//    data_[1] = (*data<<8)>>16;
//    data_[2] = (*data<<16)>>8;
//    data_[3] = (*data<<24)>>24;
    printf("SENT%u, %u\n\r",data_[0],data_[1]);
    BLE &ble = BLE::Instance();
    ble.gattServer().write(tofChar.getValueHandle(), data_, sizeof(uint8_t)*BYTESSENT);
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
    ble.gap().onTimeout(timeoutCallback);
    ble.onDataWritten(writeCharCallback);
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

void appendCurrentTimeToList(int status){
    uint16_t seconds = time(NULL);   
    printf("Seconds since start = %u\n\r", (unsigned int)seconds);  
    seconds = seconds/TIMEDIVIDER;
    if( status == ENTER_ROOM ){ 
        seconds = seconds | 0x8000; // SET MSB to 1 for Entering room
    }else if ( status == EXIT_ROOM ){
        seconds = seconds & ~0x8000; // SET MSB to 0 for Exiting room
    }

    array_timestamps[timestamps_index] = seconds;   
    timestamps_index++; 
    timestamps_index = timestamps_index % TIMESTAMP_ARRAY_SIZE;
}

time_t prevSensor1Time = 0;
time_t prevSensor2Time = 0;
bool detectedSensor1 = false;
bool detectedSensor2 = false;


int8_t aggregateMeasurements(uint32_t measurement1, uint32_t measurement2){
    
    time_t currentTime = time(NULL);
    int8_t status = NONE_SENSE;
    // Logic for Sensor 1 detecting something
    
    if (measurement1 <= MAX_DISTANCE && measurement1 > DOOR_THRESHOLD) { 
        //printf("Sensor1 Triggered\n\r"); 
        detectedSensor1 = true;

    } else { 
        
        if (detectedSensor1) { 
        
          //  printf("Sensor 1 Detected! \n\r"); 
            detectedSensor1 = false;
            if(currentTime - prevSensor2Time < DETECTED_THRESHOLD){
                status += ENTER_ROOM;
            //    printf("Someone Entered Room? \n\r");
            }
            prevSensor1Time = currentTime;
        } 
    } 
    // Logic for Sensor 2 detecting something
    if (measurement2 <= MAX_DISTANCE && measurement2 > DOOR_THRESHOLD) {
       // printf("Sensor2 triggered\n\r");
        detectedSensor2 = true;
    
    } else {
        if (detectedSensor2) {
    
         //   printf("Sensor 2 Detected! \n\r");
            detectedSensor2 = false;
    
            if(currentTime - prevSensor1Time < DETECTED_THRESHOLD){
                status += EXIT_ROOM;
           //     printf("Someone Exited Room? \n\r");
            }
            prevSensor2Time = currentTime;
        }
    }

    return status; // If someone entered and exited at the same time return NON_SENSE
}

void reset_tof_i2c(void){

    reset_i2c_chn();
    wait_ms(I2C_RESETTIME);
    init_sensor(RANGE_SENSOR1);
    init_sensor(RANGE_SENSOR2);
    wait_ms(I2C_RESETTIME);

}


int main(void)
{
    led1 = 1;
    led4 = 1;
    led2 = 1;
    led3 = 1;
    uint32_t measurement, measurement2;
    int8_t personPassingThrough = 0; 
    init_sensor(RANGE_SENSOR1);
    init_sensor(RANGE_SENSOR2); 
    set_time(0);
    ticker.attach(periodicCallback, 0.1);
    btn1.rise(button1Callback); 
    BLE &ble = BLE::Instance();
    ble.init(bleInitComplete);

    /* SpinWait for initialization to complete. This is necessary because the
 *      * BLE object is used in the main loop below. */
    while (ble.hasInitialized()  == false) { /* spin loop */ }
    
    uint32_t num_readings = 0;
//    for(int i=0;i<10;i++){
//        wait_ms(2000);
//        if(i%2==0){
//            appendCurrentTimeToList(EXIT_ROOM);
//        }else{
//            appendCurrentTimeToList(ENTER_ROOM);
//        }
//    }
    while (true) {
        ble.processEvents();
        if( READ_FLAG == 1){
            READ_FLAG=0;
                    
            measurement  = take_measurement(RANGE_SENSOR1);
            measurement2 = take_measurement(RANGE_SENSOR2);

            
            //if( num_readings %10 == 0){

            //printf("Nreads: %d, 1: %lu, 2:%lu\n\r",num_readings , measurement, measurement2);
            //}
            if( measurement2 ==-1 ||  measurement ==-1){
                led2 = 1;   
                printf("Number of Reads: %d\n\r", num_readings);
                printf("Resetting Channel");
                reset_tof_i2c();
                READ_FLAG=0;
                continue; 
            }
            led2 = 0;
            num_readings++;
            
            personPassingThrough = aggregateMeasurements(measurement, measurement2);
            if( personPassingThrough == EXIT_ROOM ){
                led3 = !led3;
                //buttonServicePtr -> updateButtonState( ++timestamps_index );
                //updateDataToCharacteristic(ble, &++tof_triggered_counter);
                appendCurrentTimeToList(EXIT_ROOM); 
                printf("Someone Exited Room\n\r");
            }else if( personPassingThrough == ENTER_ROOM ){
                led3 = !led3;
                //buttonServicePtr -> updateButtonState( ++timestamps_index );
                //updateDataToCharacteristic(ble, &++tof_triggered_counter);
                appendCurrentTimeToList(ENTER_ROOM);
                printf("Someone Entered Room\n\r");
          }
        }
        if(isConnected){
            uint16_t endOfTx = 65535;
            led1 = 1;
            if(READY_TO_SENT){
        //        printf("ReadyToSend!");
                if(timestamps_index!=0){        
                    printf("Sending timestamps to phone\n\r");
                    uint16_t curr_secs = time(NULL);
                    printf("Current Time: %u\n\r", curr_secs);
                    updateDataToCharacteristic(curr_secs);
                    for(int i=0; i<timestamps_index; i++){
                        printf("%d: %u (s)\n\r", i, array_timestamps[i]);
                        updateDataToCharacteristic(array_timestamps[i]);
                        if(i==timestamps_index-1){
                            updateDataToCharacteristic(endOfTx);
                        }
                    }
                    timestamps_index=0;
//                    ble.disconnect(Gap::REMOTE_USER_TERMINATED_CONNECTION);
                }else{
                 //    updateDataToCharacteristic(endOfTx);
                 //   READY_TO_SENT=false;
                 //updateDataToCharacteristic(endOfTx);
                 //  ble.disconnect(Gap::REMOTE_USER_TERMINATED_CONNECTION);
                 //  isConnected = false; 
                }
//                for(int i=0;i<3;i++){
//                    wait_ms(1000);
//                    if(i%2==0){
//                        appendCurrentTimeToList(EXIT_ROOM);
//                    }else{
//                        appendCurrentTimeToList(ENTER_ROOM);
//                    }
//                }
//
            }
        }
    }
        
}
