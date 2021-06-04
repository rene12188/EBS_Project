/*
 * Toggles LED on pin 1 on port N
 * Turns off LED on pin 0 on port N if button (USR_SW1 on port J pin 0) is pressed
 *
 * Already set build options of this project (no need to change anything):
 * Stack size set to 4096 byte
 * Heap size disabled - No malloc() available
 * No code optimization - Easier debugging
 * Strict floating point interrupt behavior
 * Minimal printf() and friends support - No floating point - reduces footprint
 * Hardware floating point unit activated
 */

#define TARGET_IS_TM4C129_RA2 /* Tells rom.h the version of the silicon */
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"

/* Controller is initially clocked with 16 MHz (via PIOSC) */
/* !!! Changing this macro does not change clock speed !!! */
#define F_CPU (16000000)

volatile uint32_t Blinkspeed = 1;
uint32_t OneSecond = 615385*2;

uint32_t I2CReceive(uint32_t slave_addr, uint8_t reg)
    {
        //specify that we are writing (a register address) to the
        //slave device
        I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, false);

        //specify register to be read
        I2CMasterDataPut(I2C0_BASE, reg);

        //send control byte and register address byte to slave device
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);

        //wait for MCU to finish transaction
        while(I2CMasterBusy(I2C0_BASE));

        //specify that we are going to read from slave device
        I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, true);

        //send control byte and read from the register we
        //specified
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);

        //wait for MCU to finish transaction
        while(I2CMasterBusy(I2C0_BASE));

        //return data pulled from the specified register
        return I2CMasterDataGet(I2C0_BASE);
    }


int main(void)
{
    uint32_t ui32Loop;
    uint32_t ui32temp = 0;
    setvbuf(stdout, NULL, _IONBF, 0);

    /* Activate GPIO ports */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    //enable I2C module 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

    //reset module
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);

    //enable GPIO peripheral that contains I2C 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Configure the pin muxing for I2C0 functions on port B2 and B3.
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);

    // Select the I2C function for these pins.
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockFreqSet(SYSCTL_OSC_INT | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_320,     F_CPU), false);

    //clear I2C FIFOs
    HWREG(I2C0_BASE + I2C_O_FIFOCTL) = 80008000;



    //Output
    //RGB LED Green Pin
    //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 8);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3 );

    //RGB LED Red Pin
    //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 4);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2 );

    //RGB LED Blue Pin
    //GPIOPinWrite( GPIO_PORTG_BASE, GPIO_PIN_0 ,1);
    GPIOPinTypeGPIOOutput(GPIO_PORTG_BASE, GPIO_PIN_0 );

    while(1) {

        for(ui32Loop = 0; ui32Loop < OneSecond; ui32Loop++)
                             __nop(); /* "No operation", prohibits possible loop elimination by the compiler. */
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 8);
       ui32temp = I2CReceive(0x40, 0x01);
       printf("Button 1: %d\n", ui32temp);
       for(ui32Loop = 0; ui32Loop < OneSecond; ui32Loop++)
                            __nop(); /* "No operation", prohibits possible loop elimination by the compiler. */
       GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);
       for(ui32Loop = 0; ui32Loop < OneSecond; ui32Loop++)
                                __nop(); /* "No operation", prohibits possible loop elimination by the compiler. */
    }


   }


