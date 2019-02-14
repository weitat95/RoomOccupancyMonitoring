#include <events/mbed_events.h>
#include <mbed.h>
#include "ble/BLE.h"
#include "ble/Gap.h"
#include "ble/services/HeartRateService.h"
#include "main.h"

DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
DigitalOut led4(LED4);
Serial PC(USBTX, USBRX);


const static char     DEVICE_NAME[] = "Not a Spy Drone";

static EventQueue eventQueue(/* event count */ 16 * EVENTS_EVENT_SIZE);

void onBleInitError(BLE &ble, ble_error_t error)
{
    (void)ble;
    (void)error;
   /* Initialization error handling should go here */
   PC.printf("Bluetooth initialisation failed");
   led1=0;
}

// Initiialization routine goes here
void bleInitComplete(BLE::InitializationCompleteCallbackContext *params)
{

    BLE&        ble   = params->ble;
    ble_error_t error = params->error;

    // healthy error checking
    if (error != BLE_ERROR_NONE) {
        onBleInitError(ble, error);
        return;
    }

    if (ble.getInstanceID() != BLE::DEFAULT_INSTANCE) {
        return;
    }


/* Setup advertising. */
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::GENERIC_HEART_RATE_SENSOR);
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LOCAL_NAME,
       (uint8_t*)DEVICE_NAME, sizeof(DEVICE_NAME));
    ble.gap().setAdvertisingType(GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED);
    ble.gap().setAdvertisingInterval(1000); /* 1000ms */
    ble.gap().startAdvertising();
}

// events processing handler
void scheduleBleEventsProcessing(BLE::OnEventsToProcessCallbackContext* context) {
    eventQueue.call(callback(&(context->ble), &BLE::processEvents));
}

int main()
{
    // create BLE instance
    BLE &ble = BLE::Instance();
    ble.onEventsToProcess(scheduleBleEventsProcessing);
    ble.init(bleInitComplete);

    // link to events processing handler
    // initialise the BLE device
    led1=1;
    led2=1;
    led3=0;
    led4=1;
    PC.baud(115200);
    PC.printf("Initialising Buttons");
    init_buttons();
    init_leds();

    while(0) {
      PC.printf("Nice\n");
      led4=0;

      wait_ms(500);

      led4=1;

      wait_ms(500);

      // If button 1 is pressed turn on LED 1
      if((NRF_GPIO->DIR & (1 << 17))>> 17) {
        led1=1;
        break;
      }
    }

    eventQueue.dispatch_forever(); // blocking

    return 0;
}
