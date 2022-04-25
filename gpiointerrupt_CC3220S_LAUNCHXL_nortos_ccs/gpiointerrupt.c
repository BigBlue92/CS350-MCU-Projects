/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== gpiointerrupt.c ========
 */
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>

/* Driver configuration */
#include "ti_drivers_config.h"

/* For the I2C driver, copy the following code into CCS */
#include <ti/drivers/I2C.h>

/* Include the header file: ti/drivers/Timer.h */
#include <ti/drivers/Timer.h>

/* For the UART driver, copy the following code into CCS: */
#include <ti/drivers/UART.h>
#define DISPLAY(x) UART_write(uart, &output, x);

/* I2C Global Variables */
static const struct {
    uint8_t resultReg;
    uint8_t address;
    char *id;}
sensors[3] = {    { 0x48, 0x0000, "11X" },
                  { 0x49, 0x0000, "116" },
                  { 0x41, 0x0001, "006" }};
uint8_t txBuffer[1];
uint8_t rxBuffer[2];
I2C_Transaction i2cTransaction;

/* UART Global Variables */
char output[64];
int bytesToSend;

/* UART Driver Handles - Global variables */
UART_Handle uart;

// Timer Driver Handles - Global variables
Timer_Handle timer0;
volatile unsigned char TimerFlag = 0;

// I2C Driver Handles - Global variables
I2C_Handle i2c;

// Other Global Variables
int16_t temperature;
bool toggleHeat;
int seconds;

// Make sure you call initUART() before calling this function.
void initI2C(void)
{
    int8_t i, found;
    I2C_Params i2cParams;
    DISPLAY(snprintf(output, 64, "Initializing I2C Driver - "));

    // Init the driver
    I2C_init();

    // Configure the driver
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;

    // Open the driver
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);
    if (i2c == NULL)
    {
        DISPLAY(snprintf(output, 64, "Failed\n\r"))
        while (1)
            ;
    }

    DISPLAY(snprintf(output, 32, "Passed\n\r"));

    // Boards were shipped with different sensors.
    // Welcome to the world of embedded systems.
    // Try to determine which sensor we have.
    // Scan through the possible sensor addresses

    /* Common I2C transaction setup */
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;
    found = false;

    for (i = 0; i < 3; ++i)
    {
        i2cTransaction.slaveAddress = sensors[i].address;
        txBuffer[0] = sensors[i].resultReg;
        DISPLAY(snprintf(output, 64, "Is this %s? ", sensors[i].id));

        if (I2C_transfer(i2c, &i2cTransaction))
        {
            DISPLAY(snprintf(output, 64, "Found\n\r"))
            found = true;
            break;
        }
        DISPLAY(snprintf(output, 64, "No\n\r"));
    }

    if (found)
    {
        DISPLAY(snprintf(output, 64, "Detected TMP%s I2C address: %x\  n\r",
                         sensors[i].id, i2cTransaction.slaveAddress))
    }
    else
    {
        DISPLAY(snprintf(output, 64,
                         "Temperature sensor not found, contact professor\n\r"))
    }
}

int16_t readTemp(void)
{
    i2cTransaction.readCount = 2;

    if (I2C_transfer(i2c, &i2cTransaction))
    {
        /*
         * Extract degrees C from the received data;
         * see TMP sensor datasheet
         */
        temperature = (rxBuffer[0] << 8) | (rxBuffer[1]);
        temperature *= 0.0078125;

        /*
         * If the MSB is set '1', then we have a 2's complement
         * negative value which needs to be sign extended
         */
        if (rxBuffer[0] & 0x80)
        {
            temperature |= 0xF000;
        }
    }
    else
    {
        DISPLAY(snprintf(output, 64,
                         "Error reading temperature sensor (%d)\n\r",
                         i2cTransaction.status));
        DISPLAY(snprintf(output, 64,
                "Please power cycle your board by unplugging USB and plugging back in.\n\r"));
    }
    return temperature;
}

void initUART(void)
{
    UART_Params uartParams;

    // Init the driver
    UART_init();

    // Configure the driver
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.baudRate = 115200;

    // Open the driver
    uart = UART_open(CONFIG_UART_0, &uartParams);
    if (uart == NULL)
    {
        /* UART_open() failed */
        while (1);
    }
}

void timerCallback(Timer_Handle myHandle, int_fast16_t status)
{
    TimerFlag = 1;
}

void initTimer(void)
{
    Timer_Params params;

    // Init the driver
    Timer_init();

    // Configure the driver
    Timer_Params_init(&params);
    params.period = 100000;
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;

    // Open the driver
    timer0 = Timer_open(CONFIG_TIMER_0, &params);
    if (timer0 == NULL)
    {
        /* Failed to initialized timer */
        while (1)
        {
        }
    }
    if (Timer_start(timer0) == Timer_STATUS_ERROR)
    {
        /* Failed to start timer */
        while (1)
        {
        }
    }
}

// Task Scheduler States
enum DisplayState {DS_Start, DS_Display};
enum TempState {TS_Start, TS_TempUp, TS_TempDown};
enum ButtonState {BS_Start, BS_TempUp, BS_TempDown};

// Time variables
const unsigned long taskGCD = 100;
const unsigned buttonTime = 200;
const unsigned tempTime = 500;
const unsigned displayTime = 1000;

// Task scheduler
typedef struct taskScheduler {
    unsigned long period;
    unsigned long elapsed;
    int state;
    int (*TickFunct)(int);
} taskScheduler;

// Task variables
taskScheduler taskCommands[3];
const unsigned char taskNum = 3;

// Function to check button state
int checkButton(int state){

    switch (state)
    {
        case BS_TempUp:
            temperature++;
            break;
        case BS_TempDown:
            temperature--;
            break;
        default:
            break;
    }

    return BS_Start;
}

// Function returns true or false for temperature
bool checkTemp(int16_t setpoint, int16_t temp) {

    if (setpoint > temp)
    {
        taskCommands[0].state = TS_TempUp;
        return true;
    }
    else if (setpoint < temp)
    {
        taskCommands[0].state = TS_TempDown;
        return false;
    }
    else {
        taskCommands[0].state = TS_Start;
        return false;
    }
}

// Print function
int printDisplay(int state, int16_t setpoint, bool heat, int seconds) {

    DISPLAY(snprintf(output, 64, "<%02d,%02d,%d,%04d>\n\r", temperature, setpoint, heat, seconds))
    return DS_Start;

}

// LED toggle function; Turns LED on when heat is active
void toggleLED(bool toggleHeat) {

    if(toggleHeat) {
        GPIO_write(CONFIG_GPIO_LED_0, 1);
    }
    else {
        GPIO_write(CONFIG_GPIO_LED_0, 0);
    }
}

// Check temp function
int checkTemperature(int state, int16_t setpoint) {

    toggleHeat = checkTemp(setpoint, temperature);
    toggleLED(toggleHeat);

    return state;
}



/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_0.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn0(uint_least8_t index)
{
    // Trigger left button flag
    taskCommands[0].state = BS_TempUp;
}


/*
 *  ======== gpioButtonFxn1 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_1.
 *  This may not be used for all boards.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn1(uint_least8_t index)
{
    // Trigger right button flag
    taskCommands[0].state = BS_TempDown;
}


/*
 *  ======== mainThread ========
 */
void* mainThread(void *arg0)
{
    /* Local variables */
    int seconds = 0;
    size_t size = 2;
    toggleHeat = false;

    // local timing variables
    unsigned long BT_ElapsedTime = 200;
    unsigned long TP_ElapsedTime = 500;
    unsigned long DP_ElapsedTime = 1000;

    char setpoint[2];
    const char echoPrompt[] = "Enter set point: \r\n";

    /* Set tasks */
    unsigned char i = 0;
    taskCommands[i].state = BS_Start;
    taskCommands[i].period = buttonTime;
    taskCommands[i].elapsed = taskCommands[i].period;
    taskCommands[i].TickFunct = &checkButton;

    ++i;

    taskCommands[i].state = TS_Start;
    taskCommands[i].period = tempTime;
    taskCommands[i].elapsed = taskCommands[i].period;
    taskCommands[i].TickFunct = &checkTemperature;

    ++i;

    taskCommands[i].period = displayTime;
    taskCommands[i].elapsed = taskCommands[i].period;
    taskCommands[i].TickFunct = &printDisplay;

    /* Call driver init functions */
    GPIO_init();
    initUART();
    initI2C();
    initTimer();

    // Read current temp
    readTemp();

    /* Configure the LED and button pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0,
                   GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);

    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);

    /*
     *  If more than one input pin is available for your device, interrupts
     *  will be enabled on CONFIG_GPIO_BUTTON1.
     */
    if (CONFIG_GPIO_BUTTON_0 != CONFIG_GPIO_BUTTON_1)
    {
        /* Configure BUTTON1 pin */
        GPIO_setConfig(CONFIG_GPIO_BUTTON_1,
                       GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

        /* Install Button callback */
        GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);
        GPIO_enableInt(CONFIG_GPIO_BUTTON_1);
    }

    GPIO_write(CONFIG_GPIO_LED_0, 0);
    UART_write(uart, echoPrompt, sizeof(echoPrompt));

    // Create initial temp setting
    UART_read(uart, &setpoint, size);
    int16_t setpoint_int = atoi(setpoint);

    while (1)
    {

        // Every 200ms check the button flags
        if (BT_ElapsedTime >= 200)
        {
            checkButton(taskCommands[0].state);
            BT_ElapsedTime = 0;
        }

        // every 500ms read the temperature and update the LED
        if (TP_ElapsedTime >= 500)
        {
            checkTemperature(taskCommands[1].state, setpoint_int);
            TP_ElapsedTime = 0;
        }

        // Every second output the following to the UART
        // "<%02d,%02d,%d,%04d>, temperature, set point, heat, seconds"
        if (DP_ElapsedTime >= 1000)
        {
            printDisplay(taskCommands[2].state, setpoint_int, toggleHeat, seconds);
            DP_ElapsedTime = 0;
        }

        // wait for timer period and lower flag raised by timer
        while (!TimerFlag)
        {
        }
        TimerFlag = 0;

        // calculate time
        BT_ElapsedTime += taskGCD;
        TP_ElapsedTime += taskGCD;
        DP_ElapsedTime += taskGCD;
        seconds += 1;
    }

}
