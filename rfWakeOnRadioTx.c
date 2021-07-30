/*
 *TIRE PRESSURE MONITORING SYSTEM
 *Monitor Side
 *Date   : 23.07.2021
 *Author : MOUAIAD SALAAS
 *this project copy right return to INOVAR R&D SOLUTIONS limited company
 */


//NOTES:
/*(CC1310)
 *1.OUR USED PINS ARE:
 *                        IOID_20   :   FXTH Activate PIN ,
 *
 *                        IOID_XX   :   UART Tx PIN ,
 *                        IOID_XX   :   UART Rx PIN ,
 *
 */

/*-------------------------------------------------------0.Preparing Section------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------------------------------------------*/

/************************************************************ Includes ***********************************************************************/
/* Standard C Libraries */
#include <stdlib.h>
#include <stdint.h>
#include <stddef.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/Assert.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>

/* TI-RTOS Header files */
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/PIN.h>
/* Driverlib Header files */
#include DeviceFamily_constructPath(driverlib/rf_prop_mailbox.h)

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/UART.h>

#include <ti/display/Display.h>
#define delayMs(a)                Task_sleep(a*1000 / Clock_tickPeriod);

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/cpu.h)

/* Board Header files */
#include "Board.h"
/* Application Header files */
#include "RFQueue.h"
/* RF settings */
#include "smartrf_settings/smartrf_settings.h"
#include "smartrf_settings/smartrf_settings2.h"

/* Sytstem Tasks and clocks Structs */
Task_Struct task0Struct,
            task1Struct,
            task2Struct,
            task3Struct,
            task4Struct;

Clock_Struct clk2Struct;
Clock_Struct clk0Struct, clk1Struct;

#define THREADSTACKSIZE         1024

Char task0Stack[THREADSTACKSIZE],
     task1Stack[THREADSTACKSIZE],
     task2Stack[THREADSTACKSIZE],
     task3Stack[THREADSTACKSIZE],
     task4Stack[THREADSTACKSIZE];




char str[400] = {0};

char sends[150];
Clock_Handle clk2Handle;
Clock_Handle clk2Handle;
UART_Handle uart;
uint32_t Rx_data_index = 0;
char Rx_data[100];
unsigned char buf[128] = {0};
uint32_t GlobalCounter = 0;
bool tick = 0;
/******************************************************************************/
/***** Defines *****/
/* Wake-on-Radio configuration */
#define WOR_WAKEUPS_PER_SECOND 2

/* TX number of random payload bytes */
#define PAYLOAD_LENGTH 30

/* WOR Example configuration defines */
#define WOR_PREAMBLE_TIME_RAT_TICKS(x) \
    ((uint32_t)(4000000*(1.0f/(x))))

/* TX task stack size and priority */
#define TX_TASK_STACK_SIZE 1024
#define TX_TASK_PRIORITY   2


/***** Prototypes *****/
static void WORTask(UArg arg0, UArg arg1);
static void initializeTxAdvCmdFromTxCmd(rfc_CMD_PROP_TX_ADV_t* RF_cmdPropTxAdv, rfc_CMD_PROP_TX_t* RF_cmdPropTx);


/***** Variable declarations *****/

/* TX packet payload (length +1 to fit length byte) and sequence number */
static uint8_t packet[PAYLOAD_LENGTH +1];
static uint16_t seqNumber;

/* RF driver objects and handles */
RF_Params rfParams;

static RF_Object rfObject;
static RF_Handle rfHandle;
/******************************************************************************/
/***** Defines *****/

/* Packet RX Configuration */
#define DATA_ENTRY_HEADER_SIZE 8  /* Constant header size of a Generic Data Entry */
#define MAX_LENGTH             30 /* Max length byte the radio will accept */
#define NUM_DATA_ENTRIES       2  /* NOTE: Only two data entries supported at the moment */
#define NUM_APPENDED_BYTES     2  /* The Data Entries data field will contain:
                                   * 1 Header byte (RF_cmdPropRx2.rxConf.bIncludeHdr = 0x1)
                                   * Max 30 payload bytes
                                   * 1 status byte (RF_cmdPropRx2.rxConf.bAppendStatus = 0x1) */



/***** Prototypes *****/
static void callback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e);

/***** Variable declarations *****/

/* Buffer which contains all Data Entries for receiving data.
 * Pragmas are needed to make sure this buffer is 4 byte aligned (requirement from the RF Core) */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_ALIGN (rxDataEntryBuffer, 4);
static uint8_t
rxDataEntryBuffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,
                                                  MAX_LENGTH,
                                                  NUM_APPENDED_BYTES)];
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment = 4
static uint8_t
rxDataEntryBuffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,
                                                  MAX_LENGTH,
                                                  NUM_APPENDED_BYTES)];
#elif defined(__GNUC__)
static uint8_t
rxDataEntryBuffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,
                                                  MAX_LENGTH,
                                                  NUM_APPENDED_BYTES)]
                                                  __attribute__((aligned(4)));
#else
#error This compiler is not supported.
#endif

/* Receive dataQueue for RF Core to fill in data */
static dataQueue_t dataQueue;
static rfc_dataEntryGeneral_t* currentDataEntry;
static uint8_t packetLength;
static uint8_t* packetDataPointer;
uint8_t packet_counter = 0;


static uint8_t packet[MAX_LENGTH + NUM_APPENDED_BYTES - 1]; /* The length byte is stored in a separate variable */


/******************************************************************************/

/* Pin driver objects and handles */
static PIN_Handle ledPinHandle;
static PIN_Handle buttonPinHandle;
static PIN_State ledPinState;
static PIN_State buttonPinState;

/* TX Semaphore */
static Semaphore_Struct txSemaphore;
static Semaphore_Handle txSemaphoreHandle;

/* Advanced TX command for sending long preamble */
static rfc_CMD_PROP_TX_ADV_t RF_cmdPropTxAdv;

/*
 * Application LED pin configuration table:
 *   - All LEDs board LEDs are off.
 */
PIN_Config pinTable[] =
{
    Board_PIN_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_PIN_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,   
	PIN_TERMINATE
};

/*
 * Application button pin configuration table:
 *   - Buttons interrupts are configured to trigger on falling edge.
 */
PIN_Config buttonPinTable[] = {
    Board_PIN_BUTTON0 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
    PIN_TERMINATE
};

/*additional variables*/
bool wor_init_requerded_flag = false;       //flag to rise if wor need to init
bool wor_sending_flag = false;              //flag to rise if wor is sending id after wake on

bool spi_rf_data_closed = true;             //flag to rise if spi data rf channel closed
bool wor_init_rf_closed = true;             //flag to rise if wor init rf channel closed
bool wor_id_rf_closed   = true;             //flag to rise if wor id send rf channel closed

bool spi_rf_processing_flag   = false;      //flag to rise if spi data rf sending starts
bool wor_rf_processing_flag   = false;      //flag to rise if wor id   rf sending starts
bool reset_flag = false;
bool start_lesitning = true;
bool start_wor = false;
bool flagrise = false;
bool rx_For_wor = false;

bool listen_process = false;

int counter_for_packet = 0;
int counter_to_reset = 0;

char bufUart[99];
uint8_t bufIndex = 0;
uint8_t rcv_ch_0xff = 0;
uint8_t mysensor_id = 0;
 char rcv_ch;

void SendCommand( const char * data );
Void clk0Fxn(UArg arg0)
{
    GlobalCounter++;
    tick=true;
}

///*UART write function*/
//void GPRS_SendAtCommand( const char * data ){
//    //GPRS_BufferClear();
//   UART_write(uart, (unsigned char *)data, strlen(data));
//}


void uartReadCallback(UART_Handle handle, void *buf, size_t indexcount){


    strncpy(&rcv_ch, (char *)buf, indexcount);


    if(bufIndex == 0 && rcv_ch != 0x70)
    {
        return;
    }

    bufUart[bufIndex++] = rcv_ch;

    if( rcv_ch == 0xFF ){

        rcv_ch_0xff++;
    }

    if(rcv_ch_0xff == 3)
    {
        if (bufUart[1] == 'B' && bufUart[2] == 'B')
        {
            bufIndex = 0;
            rcv_ch_0xff = 0;
            flagrise = true;
            start_lesitning = false;
            start_wor = true;
            rx_For_wor = true;
            Semaphore_post(txSemaphoreHandle);

        }else{
            bufIndex = 0;
            rcv_ch_0xff = 0;
            flagrise = false;
        }
    }

}


void uartThread(UArg arg0, UArg arg1)
{
    char rcvData;
    UART_Params uartParams;

    /* Create a UART with data processing off. */
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_TEXT;
    uartParams.readDataMode = UART_DATA_TEXT;
    uartParams.readMode = UART_MODE_CALLBACK;
    uartParams.readCallback = &uartReadCallback;
    uartParams.readTimeout = 1;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.baudRate = 115200;
    uart = UART_open(Board_UART0, &uartParams);

    if (uart == NULL) {
        /* UART_open() failed */
        while (1);
    }

    /* Loop forever echoing */
    while (1) {

        sprintf(sends,"get va0.txt\xFF\xFF\xFF");
        SendCommand(sends);

        UART_read(uart, &rcvData, 1);
        delayMs(10);
    }
}

void clockThread(UArg arg0, UArg arg1)
{
    /* Construct BIOS Objects */
    Clock_Params clkParams;

    /* Call driver init functions */
    Clock_Params_init(&clkParams);
    clkParams.period = 1000;     //1 seconde
    clkParams.startFlag = TRUE;
    //startFlag = true; //starts immediatley
    //startFlag = false; //starts after timeout.

    /* Construct a periodic Clock Instance */
    Clock_construct(&clk0Struct, (Clock_FuncPtr)clk0Fxn,
                    1, &clkParams);


    clk2Handle = Clock_handle(&clk0Struct);

    Clock_start(clk2Handle);
    //Clock_stop(clk2Handle);

    while (1) {

        delayMs(10);
    }

}
/*UART write function*/
void SendCommand( const char * data ){
    //GPRS_BufferClear();
   UART_write(uart, (unsigned char *)data, strlen(data));
}

void setData( char * data){
    SendCommand(data);
}

void *mainThread(void *arg0)
{

    /* Loop forever echoing */
    while (1) {
        //GPRS_SendAtCommand("OMAR\r\n");
        delayMs(10);
        /*UART write function*/
//        sprintf(sends,"t0.txt=\"%d\"\xFF\xFF\xFF",1);
//        SPRINTF(SENDS,"T0.TXT=\"KEREM\"\XFF\XFF\XFF");
//        SENDCOMMAND(SENDS);
//        DELAYMS(10);

    }
}




/***** Function definitions *****/
/* Pin interrupt Callback function board buttons configured in the pinTable. */
void buttonCallbackFunction(PIN_Handle handle, PIN_Id pinId) {

    /* Simple debounce logic, only toggle if the button is still pushed (low) */
    CPUdelay((uint32_t)((48000000/3)*0.050f));
    if (!PIN_getInputValue(pinId)) {
        /* Post TX semaphore to TX task */
        start_lesitning = false;
        start_wor = true;
        rx_For_wor = true;
        Semaphore_post(txSemaphoreHandle);
    }
}

/* TX task function. Executed in Task context by TI-RTOS when the scheduler starts. */
static void WORTask(UArg arg0, UArg arg1)
{



    /* Enter main TX loop */
    while(1)
    {
        delayMs(10);
        Semaphore_pend(txSemaphoreHandle, BIOS_WAIT_FOREVER);
        delayMs(100);
        if(start_wor == true && start_lesitning == false){
            if(listen_process == false){

                /* Wait for a button press */
                         if(wor_init_rf_closed == false || spi_rf_data_closed == false){
                             RF_close(rfHandle);
                             wor_init_rf_closed = true;
                             spi_rf_data_closed = true;
                         }
                         /* Initialize the radio */
                         RF_Params_init(&rfParams);
                         wor_init_rf_closed = false;
                         /* Initialize TX_ADV command from TX command */
                         initializeTxAdvCmdFromTxCmd(&RF_cmdPropTxAdv, &RF_cmdPropTx);

                         /* Set application specific fields */
                         RF_cmdPropTxAdv.pktLen = PAYLOAD_LENGTH +1; /* +1 for length byte */
                         RF_cmdPropTxAdv.pPkt = packet;
                         RF_cmdPropTxAdv.preTrigger.triggerType = TRIG_REL_START;
                         RF_cmdPropTxAdv.preTime = WOR_PREAMBLE_TIME_RAT_TICKS(WOR_WAKEUPS_PER_SECOND);
                         RF_cmdPropTxAdv.pNextOp = (rfc_radioOp_t *)&RF_cmdPropRx2;

                         /* Request access to the radio */
                     #if defined(DeviceFamily_CC26X0R2)
                         rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioSetup, &rfParams);
                     #else
                         rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup, &rfParams);
                     #endif// DeviceFamily_CC26X0R2

                         /* Set the frequency */
                         RF_runCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, 0);

                         /* Create packet with incrementing sequence number and random payload */
                         packet[0] = PAYLOAD_LENGTH;
                         packet[1] = (uint8_t)(seqNumber >> 8);
                         packet[2] = (uint8_t)(seqNumber++);
                         uint8_t i;
                         for (i = 3; i < PAYLOAD_LENGTH +1; i++)
                         {
                             packet[i] = rand();
                         }

                         /* Send packet */
                         RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropTxAdv, RF_PriorityNormal, NULL, 0);

                         /* Update display */

                         /* Toggle LED */
                         PIN_setOutputValue(ledPinHandle, Board_PIN_LED1, !PIN_getOutputValue(Board_PIN_LED1));

                         start_lesitning = true;
                         start_wor = false;
            }

        }
    }
}

/* Copy the basic RX configuration from CMD_PROP_RX to CMD_PROP_RX_SNIFF command. */
static void initializeTxAdvCmdFromTxCmd(rfc_CMD_PROP_TX_ADV_t* RF_cmdPropTxAdv, rfc_CMD_PROP_TX_t* RF_cmdPropTx)
{
    #define RADIO_OP_HEADER_SIZE 14

    /* Copy general radio operation header from TX commmand to TX_ADV */
    memcpy(RF_cmdPropTxAdv, RF_cmdPropTx, RADIO_OP_HEADER_SIZE);

    /* Set command to CMD_PROP_TX_ADV */
    RF_cmdPropTxAdv->commandNo = CMD_PROP_TX_ADV;

    /* Copy over relevant parameters */
    RF_cmdPropTxAdv->pktConf.bFsOff = RF_cmdPropTx->pktConf.bFsOff;
    RF_cmdPropTxAdv->pktConf.bUseCrc = RF_cmdPropTx->pktConf.bUseCrc;
    RF_cmdPropTxAdv->syncWord = RF_cmdPropTx->syncWord;
}


void *RXTask(void *arg0)
{


    while(1){
        delayMs(10);
        if(start_lesitning == true && start_wor == false){
            listen_process = true;
                if(wor_init_rf_closed == false || spi_rf_data_closed == false){
                    RF_close(rfHandle);
                    wor_init_rf_closed = true;
                    spi_rf_data_closed = true;
                }
                RF_Params_init(&rfParams);
                spi_rf_data_closed = false;

                        if( RFQueue_defineQueue(&dataQueue,
                                                rxDataEntryBuffer,
                                                sizeof(rxDataEntryBuffer),
                                                NUM_DATA_ENTRIES,
                                                MAX_LENGTH + NUM_APPENDED_BYTES))
                        {
                            /* Failed to allocate space for all data entries */
                            while(1);
                        }

                        /* Modify CMD_PROP_RX command for application needs */
                        /* Set the Data Entity queue for received data */
                        RF_cmdPropRx2.pQueue = &dataQueue;
                        /* Discard ignored packets from Rx queue */
                        RF_cmdPropRx2.rxConf.bAutoFlushIgnored = 1;
                        /* Discard packets with CRC error from Rx queue */
                        RF_cmdPropRx2.rxConf.bAutoFlushCrcErr = 1;
                        /* Implement packet length filtering to avoid PROP_ERROR_RXBUF */
                        RF_cmdPropRx2.maxPktLen = MAX_LENGTH;
                        RF_cmdPropRx2.pktConf.bRepeatOk = 0;
                        RF_cmdPropRx2.pktConf.bRepeatNok = 0;
                        RF_cmdPropRx2.startTrigger.triggerType = TRIG_NOW;
                        RF_cmdPropRx2.endTrigger.triggerType = TRIG_REL_START;
                        RF_cmdPropRx2.endTime = 100000 * 4;
                        RF_cmdPropRx2.pNextOp = (rfc_radioOp_t *)&RF_cmdPropTxAdv;

                        /* Request access to the radio */
                    #if defined(DeviceFamily_CC26X0R2)
                        rfHandle = RF_open(&rfObject, &RF_prop2, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup2, &rfParams);
                    #else
                        rfHandle = RF_open(&rfObject, &RF_prop2, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup2, &rfParams);
                    #endif// DeviceFamily_CC26X0R2

                        /* Set the frequency */
                        RF_postCmd(rfHandle, (RF_Op*)&RF_cmdFs2, RF_PriorityNormal, NULL, 0);

                        /* Enter RX mode and stay forever in RX */
                        RF_EventMask terminationReason = RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropRx2,
                                                                   RF_PriorityNormal, &callback,
                                                                   RF_EventRxEntryDone);

                        uint32_t cmdStatus = ((volatile RF_Op*)&RF_cmdPropRx2)->status;
                        switch(cmdStatus)
                        {
                            case PROP_DONE_OK:
                                // Packet received with CRC OK
                                if(rx_For_wor == true){
                                    mysensor_id = packet[4];
                                    rx_For_wor = false;
                                }
                                break;
                            case PROP_DONE_RXERR:
                                // Packet received with CRC error
                                break;
                            case PROP_DONE_RXTIMEOUT:
                                // Observed end trigger while in sync search
                                delayMs(10);
                                break;
                            case PROP_DONE_BREAK:
                                // Observed end trigger while receiving packet when the command is
                                // configured with endType set to 1
                                break;
                            case PROP_DONE_ENDED:
                                // Received packet after having observed the end trigger; if the
                                // command is configured with endType set to 0, the end trigger
                                // will not terminate an ongoing reception
                                break;
                            case PROP_DONE_STOPPED:
                                // received CMD_STOP after command started and, if sync found,
                                // packet is received
                                break;
                            case PROP_DONE_ABORT:
                                // Received CMD_ABORT after command started
                                break;
                            case PROP_ERROR_RXBUF:
                                // No RX buffer large enough for the received data available at
                                // the start of a packet
                                break;
                            case PROP_ERROR_RXFULL:
                                // Out of RX buffer space during reception in a partial read
                                break;
                            case PROP_ERROR_PAR:
                                // Observed illegal parameter
                                break;
                            case PROP_ERROR_NO_SETUP:
                                // Command sent without setting up the radio in a supported
                                // mode using CMD_PROP_RADIO_SETUP or CMD_RADIO_SETUP
                                break;
                            case PROP_ERROR_NO_FS:
                                // Command sent without the synthesizer being programmed
                                break;
                            case PROP_ERROR_RXOVF:
                                // RX overflow observed during operation
                                break;
                            default:
                                // Uncaught error event - these could come from the
                                // pool of states defined in rf_mailbox.h
                                while(1);
                        }

                        listen_process = false;

        }


    }
}

void callback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e)
{
    if (e & RF_EventRxEntryDone)
    {
        /* Toggle pin to indicate RX */
        PIN_setOutputValue(ledPinHandle, Board_PIN_LED2,
                           !PIN_getOutputValue(Board_PIN_LED2));

        /* Get current unhandled data entry */
        currentDataEntry = RFQueue_getDataEntry();

        /* Handle the packet data, located at &currentDataEntry->data:
         * - Length is the first byte with the current configuration
         * - Data starts from the second byte */
        packetLength      = *(uint8_t*)(&currentDataEntry->data);
        packetDataPointer = (uint8_t*)(&currentDataEntry->data + 1);

        /* Copy the payload + the status byte to the packet variable */
        memcpy(packet, packetDataPointer, (packetLength + 1));
        packet_counter++;

        RFQueue_nextEntry();
    }
}


/*
 *  ======== main ========
 */

int main(void){
    Task_Params taskParams;

    Board_initGeneral();
    GPIO_init();
    UART_init();

    ledPinHandle = PIN_open(&ledPinState, pinTable);
    Assert_isTrue(ledPinHandle != NULL, NULL);

    /* Open Button pins */
    buttonPinHandle = PIN_open(&buttonPinState, buttonPinTable);
    Assert_isTrue(buttonPinHandle != NULL, NULL);

    /* Setup callback for button pins */
    PIN_Status status = PIN_registerIntCb(buttonPinHandle, &buttonCallbackFunction);
    Assert_isTrue((status == PIN_SUCCESS), NULL);

    Semaphore_construct(&txSemaphore, 0, NULL);
    txSemaphoreHandle = Semaphore_handle(&txSemaphore);

    Task_Params_init(&taskParams);
    taskParams.stackSize = THREADSTACKSIZE;
    taskParams.stack = &task0Stack;
    taskParams.instance->name = "WORTask";
    Task_construct(&task0Struct, (Task_FuncPtr)WORTask, &taskParams, NULL);

    Task_Params_init(&taskParams);
    taskParams.stackSize = THREADSTACKSIZE;
    taskParams.stack = &task1Stack;
    taskParams.instance->name = "RXTask";
    Task_construct(&task1Struct, (Task_FuncPtr)RXTask, &taskParams, NULL);


    Task_Params_init(&taskParams);
    taskParams.stackSize = THREADSTACKSIZE;
    taskParams.stack = &task2Stack;
    taskParams.instance->name = "mainThread";
    Task_construct(&task2Struct, (Task_FuncPtr)mainThread, &taskParams, NULL);

    Task_Params_init(&taskParams);
    taskParams.stackSize = THREADSTACKSIZE;
    taskParams.stack = &task3Stack;
    taskParams.instance->name = "uartThread";
    Task_construct(&task3Struct, (Task_FuncPtr)uartThread, &taskParams, NULL);

    Task_Params_init(&taskParams);
    taskParams.stackSize = THREADSTACKSIZE;
    taskParams.stack = &task4Stack;
    taskParams.instance->name = "clockThread";
    Task_construct(&task4Struct, (Task_FuncPtr)clockThread, &taskParams, NULL);


    BIOS_start();

    return (0);
}
