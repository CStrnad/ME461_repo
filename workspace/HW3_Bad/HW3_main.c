//  CJS - Search for my initials CJS
//#############################################################################
// FILE:   HW3_main.c
//
// TITLE:  Homework #3 Code
//#############################################################################

// Included Files
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"
#include "F28379dSerial.h"
#include "LEDPatterns.h"
#include "song.h"
#include "dsp.h"
#include "fpu32/fpu_rfft.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
// The Launchpad's CPU Frequency set to 200 you should not change this value
#define LAUNCHPAD_CPU_FREQUENCY 200


// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
extern uint32_t numRXA;
uint16_t UARTPrint = 0;
uint16_t LEDdisplaynum = 0;

//I2C declarations
void I2CB_Init(void);
int16_t WriteDAN777RCServo(uint16_t RC1, uint16_t RC2);
int16_t ReadDAN777ADC(uint16_t *ADC1, uint16_t *ADC2);
int16_t WriteBQ32000(uint16_t second,uint16_t minute,uint16_t hour,uint16_t day,uint16_t date,uint16_t month,uint16_t year);
int16_t ReadBQ32000(uint16_t *second,uint16_t *minute,uint16_t *hour,uint16_t *day,uint16_t *date,uint16_t *month,uint16_t *year);
int16_t I2C_CheckIfTX(uint16_t timeout);
int16_t I2C_CheckIfRX(uint16_t timeout);

uint16_t RunI2C = 0; // Flag variable to indicate when to run I2C commands
int16_t I2C_OK = 0;
int32_t num_WriteDAN777_Errors = 0;
int32_t num_ReadDAN777_Errors = 0;

uint16_t bqVal1 = 0;
uint16_t bqVal2 = 0;
uint16_t bqVal3 = 0;
uint16_t bqVal4 = 0;
uint16_t bqVal5 = 0;
uint16_t bqVal6 = 0;
uint16_t bqVal7 = 0;
int32_t num_ReadBQ32000_Errors = 0;

uint16_t ReadTwo16BitValuesFromDAN777(uint16_t *Rvalue1,uint16_t *Rvalue2);
uint16_t WriteTwo16BitValuesToDAN777(uint16_t Cmd16bit_1, uint16_t Cmd16bit_2);
uint16_t ReadTwo16BitValuesFromBQ32000(uint16_t *Rvalue1,uint16_t *Rvalue2,uint16_t *Rvalue3,uint16_t *Rvalue4,uint16_t *Rvalue5,uint16_t *Rvalue6,uint16_t *Rvalue7);

float servoAngle1 = 0.0;
float servoAngle2 = 90.0;
uint16_t servoPos1 = 3200;
uint16_t servoPos2 = 5200;
uint16_t servoDirection = 0;
uint16_t adcVal1 = 0;
uint16_t adcVal2 = 0;
uint16_t movementSpeed = 10;



void main(void)
{
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xD_SysCtrl.c file.
    InitSysCtrl();

    InitGpio();
	
	// Blue LED on LaunchPad
    GPIO_SetupPinMux(31, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO31 = 1;

	// Red LED on LaunchPad
    GPIO_SetupPinMux(34, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(34, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO34 = 1;

	// LED1 and PWM Pin
    GPIO_SetupPinMux(22, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(22, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;
	
	// LED2
    GPIO_SetupPinMux(94, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(94, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1;

	// LED3
    GPIO_SetupPinMux(95, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(95, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO95 = 1;

	// LED4
    GPIO_SetupPinMux(97, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(97, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO97 = 1;

	// LED5
    GPIO_SetupPinMux(111, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(111, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO111 = 1;

	// LED6
    GPIO_SetupPinMux(130, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(130, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO130 = 1;

	// LED7	
    GPIO_SetupPinMux(131, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(131, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO131 = 1;

	// LED8
    GPIO_SetupPinMux(25, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(25, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;

	// LED9
    GPIO_SetupPinMux(26, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(26, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;

	// LED10
    GPIO_SetupPinMux(27, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(27, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;

	// LED11	
    GPIO_SetupPinMux(60, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(60, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;

	// LED12	
    GPIO_SetupPinMux(61, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(61, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;

	// LED13
    GPIO_SetupPinMux(157, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(157, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO157 = 1;

	// LED14
    GPIO_SetupPinMux(158, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(158, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO158 = 1;
	
	// LED15
    GPIO_SetupPinMux(159, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(159, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO159 = 1;

	// LED16
    GPIO_SetupPinMux(160, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(160, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPFCLEAR.bit.GPIO160 = 1;

    //WIZNET Reset
    GPIO_SetupPinMux(0, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(0, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO0 = 1;

    //ESP8266 Reset
    GPIO_SetupPinMux(1, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(1, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO1 = 1;

	//SPIRAM  CS  Chip Select
    GPIO_SetupPinMux(19, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(19, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;

    //DRV8874 #1 DIR  Direction
    GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO29 = 1;

    //DRV8874 #2 DIR  Direction
    GPIO_SetupPinMux(32, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(32, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO32 = 1;

    //DAN28027  CS  Chip Select
    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO9 = 1;
	
    //MPU9250  CS  Chip Select
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
	
	//WIZNET  CS  Chip Select
    GPIO_SetupPinMux(125, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(125, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDSET.bit.GPIO125 = 1;
	
    //PushButton 1
    GPIO_SetupPinMux(4, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(4, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 2
    GPIO_SetupPinMux(5, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(5, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 3
    GPIO_SetupPinMux(6, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(6, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 4
    GPIO_SetupPinMux(7, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(7, GPIO_INPUT, GPIO_PULLUP);
	
	//Joy Stick Pushbutton
    GPIO_SetupPinMux(8, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(8, GPIO_INPUT, GPIO_PULLUP);


    // Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    DINT;

    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F2837xD_PieCtrl.c file.
    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2837xD_DefaultIsr.c.
    // This function is found in F2837xD_PieVect.c.
    InitPieVectTable();

    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this project
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    PieVectTable.TIMER1_INT = &cpu_timer1_isr;
    PieVectTable.TIMER2_INT = &cpu_timer2_isr;
    PieVectTable.SCIA_RX_INT = &RXAINT_recv_ready;
    PieVectTable.SCIB_RX_INT = &RXBINT_recv_ready;
    PieVectTable.SCIC_RX_INT = &RXCINT_recv_ready;
    PieVectTable.SCID_RX_INT = &RXDINT_recv_ready;
    PieVectTable.SCIA_TX_INT = &TXAINT_data_sent;
    PieVectTable.SCIB_TX_INT = &TXBINT_data_sent;
    PieVectTable.SCIC_TX_INT = &TXCINT_data_sent;
    PieVectTable.SCID_TX_INT = &TXDINT_data_sent;

    PieVectTable.EMIF_ERROR_INT = &SWI_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers


    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every given period:
    // 200MHz CPU Freq,                       Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, LAUNCHPAD_CPU_FREQUENCY, 10000);
    ConfigCpuTimer(&CpuTimer1, LAUNCHPAD_CPU_FREQUENCY, 20000);
    ConfigCpuTimer(&CpuTimer2, LAUNCHPAD_CPU_FREQUENCY, 40000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

	init_serialSCIA(&SerialA,115200);

	I2CB_Init(); //CJS - Calling function to initialize I2C.

    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.  
    IER |= M_INT1;
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
	// Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;
	
	init_serialSCIB(&SerialB,115200);
	init_serialSCIC(&SerialC,115200);
	init_serialSCID(&SerialD,115200);
    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    I2CB_Init(); //CJS - Calling function to initialize I2C.
    
    // IDLE loop. Just sit and loop forever (optional):
//    while(1)
//    {
//        if (UARTPrint == 1 ) {
//			serial_printf(&SerialA,"Num Timer2:%ld Num SerialRX: %ld\r\n",CpuTimer2.InterruptCount,numRXA);
//            UARTPrint = 0;
//        }
//      }

        //CJS - Reading and writing via I2C
//        int16_t WriteDAN777RCServo(uint16_t RC1, uint16_t RC2);
//        int16_t ReadDAN777ADC(uint16_t *ADC1, uint16_t *ADC2);

        /*  Goals:
         *  - Call the Write and Read functions.
         *  - In one of the timer ISR functions, set RunI2C flag var to "1" every 20ms.
         *  - Print the two ADC readings every 100 ms (Code in timer ISR function.
         *  - Command the two RC servos to continuously swing between 90 deg and -90 deg
         *  - Finally, during checkoff, show SDA and SCL sigs and servo PWM signals on Oscilloscope
         */
    while(1)
    {
        if (UARTPrint == 1 ) {
            serial_printf(&SerialA,"servoPos1: %d, servoPos2: %d, adcVal1: %d, adcVal2: %d\r\n", servoPos1, servoPos2, adcVal1, adcVal2);
            UARTPrint = 0;
        }

        if (RunI2C == 1)  {
            //CJS - Increment/Decrement Servo positions.
            if(servoDirection == 1 && servoPos1 < 5200) {
                servoPos1 += movementSpeed;
                servoPos2 -= movementSpeed;
            }
            if(servoPos1 == 5200) servoDirection = 0;

            if(servoDirection == 0 && servoPos1 > 3200) {
                servoPos1 -= movementSpeed;
                servoPos2 += movementSpeed;
            }
            if(servoPos1 <= 3200) servoDirection = 1;

            RunI2C = 0;
            // Write to DAN777
            I2C_OK = WriteTwo16BitValuesToDAN777(servoPos1, servoPos2);
            num_WriteDAN777_Errors = 0;
            while(I2C_OK != 0) {
                num_WriteDAN777_Errors++;
                if (num_WriteDAN777_Errors > 2) {
                    serial_printf(&SerialA,"WriteTwo16BitValuesToDAN777 Error: %d\r\n",I2C_OK);
                    I2C_OK = 0;
                } else {
                    I2CB_Init();
                    DELAY_US(100000);
                    I2C_OK = WriteTwo16BitValuesToDAN777(servoPos1, servoPos2);
                }
            }

            // Read DAN777
            I2C_OK = ReadTwo16BitValuesFromDAN777(&adcVal1, &adcVal2);
            num_ReadDAN777_Errors = 0;
            while(I2C_OK != 0) {
                num_ReadDAN777_Errors++;
                if (num_ReadDAN777_Errors > 2) {
                    serial_printf(&SerialA,"ReadTwo16BitValuesFromDAN777 Error: %d\r\n",I2C_OK);
                    I2C_OK = 0;
                } else {
                    I2CB_Init();
                    DELAY_US(100000);
                    I2C_OK = ReadTwo16BitValuesFromDAN777(&adcVal1, &adcVal2);
                }
            }




            // Read BQ32000
            I2C_OK = ReadTwo16BitValuesFromBQ32000(&&bqVal1, $bqVal2, $bqVal3, $bqVal4, $bqVal5, $bqVal6, $bqVal7);
            num_ReadBQ32000_Errors = 0;
            while(I2C_OK != 0) {
                num_ReadBQ32000_Errors++;
                if (num_ReadBQ32000_Errors > 2) {
                    serial_printf(&SerialA,"ReadTwo16BitValuesFromBQ320000 Error: %d\r\n",I2C_OK);
                    I2C_OK = 0;
                } else {
                    I2CB_Init();
                    DELAY_US(100000);
                    I2C_OK = ReadTwo16BitValuesFromBQ320000(&&bqVal1, $bqVal2, $bqVal3, $bqVal4, $bqVal5, $bqVal6, $bqVal7);
                }
            }


//            // Write to DAN777
//            I2C_OK = WriteTwo16BitValuesToDAN777(servoPos1, servoPos2);
//            num_WriteDAN777_Errors = 0;
//            while(I2C_OK != 0) {
//                num_WriteDAN777_Errors++;
//                if (num_WriteDAN777_Errors > 2) {
//                    serial_printf(&SerialA,"WriteTwo16BitValuesToDAN777 Error: %d\r\n",I2C_OK);
//                    I2C_OK = 0;
//                } else {
//                    I2CB_Init();
//                    DELAY_US(100000);
//                    I2C_OK = WriteTwo16BitValuesToDAN777(servoPos1, servoPos2);
//                }
//            }
//
//            // Read DAN777
//            I2C_OK = ReadTwo16BitValuesFromDAN777(&adcVal1, &adcVal2);
//            num_ReadDAN777_Errors = 0;
//            while(I2C_OK != 0) {
//                num_ReadDAN777_Errors++;
//                if (num_ReadDAN777_Errors > 2) {
//                    serial_printf(&SerialA,"ReadTwo16BitValuesFromDAN777 Error: %d\r\n",I2C_OK);
//                    I2C_OK = 0;
//                } else {
//                    I2CB_Init();
//                    DELAY_US(100000);
//                    I2C_OK = ReadTwo16BitValuesFromDAN777(&adcVal1, &adcVal2);
//                }
//            }

        }
    }
}


// SWI_isr,  Using this interrupt as a Software started interrupt
__interrupt void SWI_isr(void) {

    // These three lines of code allow SWI_isr, to be interrupted by other interrupt functions
	// making it lower priority than all other Hardware interrupts.  
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
    asm("       NOP");                    // Wait one cycle
    EINT;                                 // Clear INTM to enable interrupts
	
	
	
    // Insert SWI ISR Code here.......
	
	
    numSWIcalls++;
    
    DINT;

}

// cpu_timer0_isr - CPU Timer0 ISR
__interrupt void cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;

    numTimer0calls++;

//    if ((numTimer0calls%50) == 0) {
//        PieCtrlRegs.PIEIFR12.bit.INTx9 = 1;  // Manually cause the interrupt for the SWI
//    }

    if ((numTimer0calls%25) == 0) {
        displayLEDletter(LEDdisplaynum);
        LEDdisplaynum++;
        if (LEDdisplaynum == 0xFFFF) {  // prevent roll over exception
            LEDdisplaynum = 0;
        }
    }

    if ((numTimer0calls%50) == 0) {
		// Blink LaunchPad Red LED
		GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
    }


    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void)
{
    CpuTimer1.InterruptCount++;

    //CJS - Coding a good portion of Exercise 1 here.
    /*  Goals:
             *  - Call the Write and Read functions in MAIN LOOP.
             *  - In one of the timer ISR functions, set RunI2C flag var to "1" every 20ms.
             *  - Print the two ADC readings every 100 ms (Code in timer ISR function.
             *  - Command the two RC servos to continuously swing between 90 deg and -90 deg
             *  - Finally, during checkoff, show SDA and SCL sigs and servo PWM signals on Oscilloscope
             */

    // Set RunI2C flag to 1 every 20ms.
    RunI2C = 1;

    // Get two ADC readings every 100ms to be printed in UARTPrint
        // Create variables to hold the ADC readings.
        // Convert to useful data. Need two variables for converted data too.

    // Command the two RC servos to swing between 90 and -90 degrees.
        //Create variables for swing and direction




    if ((CpuTimer1.InterruptCount % 5) == 0) UARTPrint = 1;    //CJS - Timer 1 period is 20ms, so print every 100ms by % 5.
}

// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void)
{
	// Blink LaunchPad Blue LED
    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;

    CpuTimer2.InterruptCount++;
	
//	if ((CpuTimer2.InterruptCount % 10) == 0) {
//		UARTPrint = 1;
//	}
}

/* Functions to check if I2C is ready to transfer or receive.
 * Here we utilize EPWM7 to keep track of time. If I2C is
 * not ready, wait 100ms then try again. */
int16_t I2C_CheckIfTX(uint16_t timeout) {
    int16_t Xrdy = 0;
    EPwm7Regs.TBCTR = 0x0; // Clear counter
    EPwm7Regs.TBCTL.bit.CTRMODE = 0; // unfreeze, and enter up count mode
    while(!Xrdy) {
        if (EPwm7Regs.TBCTR > timeout) {  // if it has been 100ms
            EPwm7Regs.TBCTL.bit.CTRMODE = 3; // freeze counter
            return -1;
        }
        Xrdy = I2cbRegs.I2CSTR.bit.XRDY;
    }
    return 0;
}


int16_t I2C_CheckIfRX(uint16_t timeout) {
    int16_t Rrdy = 0;
    EPwm7Regs.TBCTR = 0x0; // Clear counter
    EPwm7Regs.TBCTL.bit.CTRMODE = 0; //unfreeze, and enter up count mode
    while(!Rrdy) {
        if (EPwm7Regs.TBCTR > timeout) {  // if we have been in this function for 100ms
            EPwm7Regs.TBCTL.bit.CTRMODE = 3; // freeze counter
            return -1;
        }
        Rrdy = I2cbRegs.I2CSTR.bit.RRDY;
    }
    return 0;
}
void I2CB_Init(void) {
    // Setting up EPWM 7 to use as a timer for error handling
    EALLOW;
    EPwm7Regs.ETSEL.bit.SOCAEN = 0;     // Disable SOC on A group
    EPwm7Regs.TBCTL.bit.CTRMODE = 3;    // Freeze counter
    EPwm7Regs.TBCTR = 0x0;              // Clear counter
    EPwm7Regs.TBPHS.bit.TBPHS = 0x0000; // Phase is 0
    EPwm7Regs.TBCTL.bit.PHSEN = 0;      // Disable phase loading
    EPwm7Regs.TBCTL.bit.CLKDIV = 7;     // divide by 1  50Mhz Clock
    EPwm7Regs.TBPRD = 0xFFFF;           // PRD not used for timer
    // Notice here we are not using the PWM signal, so CMPA/CMPB are not set
    EDIS;

    /* If an I2C device is holding the SDA Line Low you need to tell the device
     * to reinitialize by clocking 9 clocks to the device to reset it. */
    GpioDataRegs.GPBSET.bit.GPIO41 = 1; // Here make sure SCL clk is high
    GPIO_SetupPinMux(41, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(41, GPIO_OUTPUT, GPIO_PUSHPULL);

    // Set SDA as GPIO input pin for now to check if SDA is being held low.
    GPIO_SetupPinMux(40, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(40, GPIO_INPUT, GPIO_PULLUP);

    /* Check if SDA is low.  If it is manually set the SCL pin high and low to
     * create nine clock periods. For more reading see the I2C specification linked
     * in this homework document and search for "nine clock pulses" */
    if (GpioDataRegs.GPBDAT.bit.GPIO40 == 0) {  // If SDA low
        // Pulse CLK 9 Times if SDA pin Low
        for (int i = 0; i<9; i++) {
            GpioDataRegs.GPBSET.bit.GPIO41 = 1;
            DELAY_US(30);
            GpioDataRegs.GPBCLEAR.bit.GPIO41 = 1;
            DELAY_US(30);
        }
    }

    // Now setup GPIO40 as SDAB and GPIO41 and SCLB
    EALLOW;
    /* Enable internal pull-up for the selected I2C pins */
    GpioCtrlRegs.GPBPUD.bit.GPIO40 = 1;
    GpioCtrlRegs.GPBPUD.bit.GPIO41 = 1;

    /* Set qualification for the selected I2C pins */
    GpioCtrlRegs.GPBQSEL1.bit.GPIO40 = 3;
    GpioCtrlRegs.GPBQSEL1.bit.GPIO41 = 3;

    /* Configure which of the possible GPIO pins will be I2C_B pins using GPIO regs*/
    GpioCtrlRegs.GPBGMUX1.bit.GPIO40 = 1;
    GpioCtrlRegs.GPBMUX1.bit.GPIO40 = 2;

    GpioCtrlRegs.GPBGMUX1.bit.GPIO41 = 1;
    GpioCtrlRegs.GPBMUX1.bit.GPIO41 = 2;
    EDIS;

    // At breakpoint, allow I2C to continue operating
    I2cbRegs.I2CMDR.bit.FREE = 1;

    // Initialize I2C
    I2cbRegs.I2CMDR.bit.IRS = 0;

    // 200MHz / 20 = 10MHz
    I2cbRegs.I2CPSC.all = 19;

    // 10MHz/40 = 250KHz
    I2cbRegs.I2CCLKL = 15*3;  //psc > 2 so d = 5  See Usersguide
    I2cbRegs.I2CCLKH = 15*3;  //psc > 2 so d = 5  See Usersguide

    I2cbRegs.I2CIER.all = 0x00;

    I2cbRegs.I2CMDR.bit.IRS = 1;
    DELAY_US(2000);
}

// Write 2 16-bit commands (LSB then MSB) to I2C Slave DAN777 starting at DAN777's register 4
uint16_t WriteTwo16BitValuesToDAN777(uint16_t Cmd16bit_1, uint16_t Cmd16bit_2) {
    uint16_t Cmd1LSB = 0;
    uint16_t Cmd1MSB = 0;
    uint16_t Cmd2LSB = 0;
    uint16_t Cmd2MSB = 0;
    int16_t I2C_Xready = 0;
    Cmd1LSB = Cmd16bit_1 & 0xFF;        //Bottom 8 bits of command
    Cmd1MSB = (Cmd16bit_1 >> 8) & 0xFF; //Top 8 bits of command
    Cmd2LSB = Cmd16bit_2 & 0xFF;        //Bottom 8 bits of command
    Cmd2MSB = (Cmd16bit_2 >> 8) & 0xFF; //Top 8 bits of command

    // Allow time for I2C to finish up previous commands.
    DELAY_US(200);

    if (I2cbRegs.I2CSTR.bit.BB == 1) { // Check if I2C busy. If it is, it's better
        return 2;                      // to exit and try again next sample.
    }                                  // This should not happen too often.

    I2C_Xready = I2C_CheckIfTX(39062); // Poll until I2C is ready to transmit
    if (I2C_Xready == -1) {
        return 4;
    }

    I2cbRegs.I2CSAR.all = 0x25;   // Set I2C address to that of DAN777's //CJS Set Slave address to 0x25 instead of 0x3A
    I2cbRegs.I2CCNT     = 5;      // Number of values to send plus start register: 4 + 1
    I2cbRegs.I2CDXR.all = 4;      // First need to transfer the register value to start writing data
    I2cbRegs.I2CMDR.all = 0x6E20; // I2C in master mode (MST), I2C is in transmit mode (TRX) with start and stop

    I2C_Xready = I2C_CheckIfTX(39062);   // Poll until I2C ready to transmit
    if (I2C_Xready == -1) {
        return 4;
    }
    I2cbRegs.I2CDXR.all = Cmd1LSB;       // Write Command 1 LSB
    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }

    I2C_Xready = I2C_CheckIfTX(39062);   // Poll until I2C ready to transmit
    if (I2C_Xready == -1) {
        return 4;
    }
    I2cbRegs.I2CDXR.all = Cmd1MSB;       // Write Command 1 MSB
    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }

    I2C_Xready = I2C_CheckIfTX(39062);   // Poll until I2C ready to transmit
    if (I2C_Xready == -1) {
        return 4;
    }
    I2cbRegs.I2CDXR.all = Cmd2LSB;       // Write Command 2 LSB
    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }

    I2C_Xready = I2C_CheckIfTX(39062);   // Poll until I2C ready to transmit
    if (I2C_Xready == -1) {
        return 4;
    }
    I2cbRegs.I2CDXR.all = Cmd2MSB;       // Write Command 2 MSB

    // Since I2CCNT = 0 at this point, a stop condition will be issued

    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }
    return 0;
}


/* Read Two 16 Bit values from I2C Slave DAN777 starting at DAN777's register 10.
 * Notice the Rvalue1 and Rvalue2 passed as pointers (passed by reference). So pass
 * address of the uint16_t variable when using this function. For example:
 *     uint16_t Rval1 = 0;
 *     uint16_t Rval2 = 0;
 *     err = ReadTwo16BitValuesFromDAN777(&Rval1,&Rval2);
 * This allows Rval1 and Rval2 to be changed inside the function and return the
 * values read inside the function. */
uint16_t ReadTwo16BitValuesFromDAN777(uint16_t *Rvalue1,uint16_t *Rvalue2) {
    uint16_t Val1LSB = 0;
    uint16_t Val1MSB = 0;
    uint16_t Val2LSB = 0;
    uint16_t Val2MSB = 0;
    int16_t I2C_Xready = 0;
    int16_t I2C_Rready = 0;

    // Allow time for I2C to finish up previous commands.
    DELAY_US(200);

    if (I2cbRegs.I2CSTR.bit.BB == 1) { // Check if I2C busy. If it is, it's better
        return 2;                      // to exit and try again next sample.
    }                                  // This should not happen too often.

    I2C_Xready = I2C_CheckIfTX(39062); // Poll until I2C ready to transmit
    if (I2C_Xready == -1) {
        return 4;
    }

    I2cbRegs.I2CSAR.all = 0x25;   // I2C address of DAN777 //CJS Set Slave address to 0x25 instead of 0x3A
    I2cbRegs.I2CCNT     = 1;      // Just sending address to start reading from
    I2cbRegs.I2CDXR.all = 10;     // Start reading at this register location
    I2cbRegs.I2CMDR.all = 0x6620; // I2C in master mode (MST), I2C is in transmit mode (TRX) with start

    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }

    I2C_Xready = I2C_CheckIfTX(39062); // Poll until I2C ready to transmit
    if (I2C_Xready == -1) {
        return 4;
    }

    // Reissuing another start command to begin reading the values we want
    I2cbRegs.I2CSAR.all = 0x25;   // I2C address of DAN777 //CJS Set Slave address to 0x25 instead of 0x3A
    I2cbRegs.I2CCNT     = 4;      // Receive count
    I2cbRegs.I2CMDR.all = 0x6C20; // I2C in master mode (MST), TRX=0 (receive mode) with start & stop

    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }

    I2C_Rready = I2C_CheckIfRX(39062);   //Poll until I2C has received 8-bit value
    if (I2C_Rready == -1) {
        return -1;
    }
    Val1LSB = I2cbRegs.I2CDRR.all;       // Read DAN777
    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }

    I2C_Rready = I2C_CheckIfRX(39062);   //Poll until I2C has received 8-bit value
    if (I2C_Rready == -1) {
        return -1;
    }
    Val1MSB = I2cbRegs.I2CDRR.all;       // Read DAN777
    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }

    I2C_Rready = I2C_CheckIfRX(39062);   //Poll until I2C has received 8-bit value
    if (I2C_Rready == -1) {
        return -1;
    }
    Val2LSB = I2cbRegs.I2CDRR.all;       // Read DAN777
    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }

    I2C_Rready = I2C_CheckIfRX(39062);   //Poll until I2C has received 8-bit value
    if (I2C_Rready == -1) {
        return -1;
    }
    Val2MSB = I2cbRegs.I2CDRR.all;       // Read DAN777
    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }

    // Since I2CCNT = 0 at this point, a stop condition will be issued

    *Rvalue1 = (Val1MSB << 8) | (Val1LSB & 0xFF);
    *Rvalue2 = (Val2MSB << 8) | (Val2LSB & 0xFF);
    return 0;
}


/* Read Two 16 Bit values from I2C Slave DAN777 starting at DAN777's register 10.
 * Notice the Rvalue1 and Rvalue2 passed as pointers (passed by reference). So pass
 * address of the uint16_t variable when using this function. For example:
 *     uint16_t Rval1 = 0;
 *     uint16_t Rval2 = 0;
 *     err = ReadTwo16BitValuesFromDAN777(&Rval1,&Rval2);
 * This allows Rval1 and Rval2 to be changed inside the function and return the
 * values read inside the function. */
uint16_t ReadTwo16BitValuesFromBQ32000(uint16_t *Rvalue1,uint16_t *Rvalue2,uint16_t *Rvalue3,uint16_t *Rvalue4,uint16_t *Rvalue5,uint16_t *Rvalue6,uint16_t *Rvalue7) {
    uint16_t Val1LSB = 0; //Seconds
    uint16_t Val1MSB = 0;
    uint16_t Val2LSB = 0; //Minutes
    uint16_t Val2MSB = 0;
    uint16_t Val3LSB = 0; //Hour
    uint16_t Val3MSB = 0;
    uint16_t Val4LSB = 0; //Day (of the week)
    uint16_t Val4MSB = 0;
    uint16_t Val5LSB = 0; //Date (Numeric)
    uint16_t Val5MSB = 0;
    uint16_t Val6LSB = 0; //Month
    uint16_t Val6MSB = 0;
    uint16_t Val7LSB = 0; //Year
    uint16_t Val7MSB = 0;

    int16_t I2C_Xready = 0;
    int16_t I2C_Rready = 0;
    //CJS - Need to get day, month, date, year, hour, minute, second.
    //CJS - Registers:   3    5      4     6     2      1       0
    //CJS - So get 7*2, 14 values (MSB and LSB) then combine them and get data from them.


    // Allow time for I2C to finish up previous commands.
    DELAY_US(200);

    if (I2cbRegs.I2CSTR.bit.BB == 1) { // Check if I2C busy. If it is, it's better
        return 2;                      // to exit and try again next sample.
    }                                  // This should not happen too often.

    I2C_Xready = I2C_CheckIfTX(39062); // Poll until I2C ready to transmit
    if (I2C_Xready == -1) {
        return 4;
    }

    I2cbRegs.I2CSAR.all = 0x68;   // I2C address of BQ32000 //CJS Set Slave address to 0x68 instead of 0x3A
    I2cbRegs.I2CCNT     = 1;      // Just sending address to start reading from
    I2cbRegs.I2CDXR.all = 10;     // Start reading at this register location
    I2cbRegs.I2CMDR.all = 0x6620; // I2C in master mode (MST), I2C is in transmit mode (TRX) with start

    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }

    I2C_Xready = I2C_CheckIfTX(39062); // Poll until I2C ready to transmit
    if (I2C_Xready == -1) {
        return 4;
    }

    // Reissuing another start command to begin reading the values we want
    I2cbRegs.I2CSAR.all = 0x68;   // I2C address of BQ32000 //CJS Set Slave address to 0x68 instead of 0x3A
    I2cbRegs.I2CCNT     = 14;      // Receive count
    I2cbRegs.I2CMDR.all = 0x6C20; // I2C in master mode (MST), TRX=0 (receive mode) with start & stop

    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }

    //Val 1
    I2C_Rready = I2C_CheckIfRX(39062);   //Poll until I2C has received 8-bit value
    if (I2C_Rready == -1) {
        return -1;
    }
    Val1LSB = I2cbRegs.I2CDRR.all;       // Read DAN777
    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }

    I2C_Rready = I2C_CheckIfRX(39062);   //Poll until I2C has received 8-bit value
    if (I2C_Rready == -1) {
        return -1;
    }
    Val1MSB = I2cbRegs.I2CDRR.all;       // Read DAN777
    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }

    //Val 2
    I2C_Rready = I2C_CheckIfRX(39062);   //Poll until I2C has received 8-bit value
    if (I2C_Rready == -1) {
        return -1;
    }
    Val2LSB = I2cbRegs.I2CDRR.all;       // Read DAN777
    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }

    I2C_Rready = I2C_CheckIfRX(39062);   //Poll until I2C has received 8-bit value
    if (I2C_Rready == -1) {
        return -1;
    }
    Val2MSB = I2cbRegs.I2CDRR.all;       // Read DAN777
    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }

    //Val 3
    I2C_Rready = I2C_CheckIfRX(39062);   //Poll until I2C has received 8-bit value
    if (I2C_Rready == -1) {
        return -1;
    }
    Val3LSB = I2cbRegs.I2CDRR.all;       // Read DAN777
    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }

    I2C_Rready = I2C_CheckIfRX(39062);   //Poll until I2C has received 8-bit value
    if (I2C_Rready == -1) {
        return -1;
    }
    Val3MSB = I2cbRegs.I2CDRR.all;       // Read DAN777
    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }

    //Val 4
    I2C_Rready = I2C_CheckIfRX(39062);   //Poll until I2C has received 8-bit value
    if (I2C_Rready == -1) {
        return -1;
    }
    Val4LSB = I2cbRegs.I2CDRR.all;       // Read DAN777
    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }

    I2C_Rready = I2C_CheckIfRX(39062);   //Poll until I2C has received 8-bit value
    if (I2C_Rready == -1) {
        return -1;
    }
    Val4MSB = I2cbRegs.I2CDRR.all;       // Read DAN777
    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }

    //Val 5
    I2C_Rready = I2C_CheckIfRX(39062);   //Poll until I2C has received 8-bit value
    if (I2C_Rready == -1) {
        return -1;
    }
    Val5LSB = I2cbRegs.I2CDRR.all;       // Read DAN777
    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }

    I2C_Rready = I2C_CheckIfRX(39062);   //Poll until I2C has received 8-bit value
    if (I2C_Rready == -1) {
        return -1;
    }
    Val5MSB = I2cbRegs.I2CDRR.all;       // Read DAN777
    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }

    //Val 6
    I2C_Rready = I2C_CheckIfRX(39062);   //Poll until I2C has received 8-bit value
    if (I2C_Rready == -1) {
        return -1;
    }
    Val6LSB = I2cbRegs.I2CDRR.all;       // Read DAN777
    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }

    I2C_Rready = I2C_CheckIfRX(39062);   //Poll until I2C has received 8-bit value
    if (I2C_Rready == -1) {
        return -1;
    }
    Val6MSB = I2cbRegs.I2CDRR.all;       // Read DAN777
    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }


    //Val 7
    I2C_Rready = I2C_CheckIfRX(39062);   //Poll until I2C has received 8-bit value
    if (I2C_Rready == -1) {
        return -1;
    }
    Val7LSB = I2cbRegs.I2CDRR.all;       // Read DAN777
    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }

    I2C_Rready = I2C_CheckIfRX(39062);   //Poll until I2C has received 8-bit value
    if (I2C_Rready == -1) {
        return -1;
    }
    Val7MSB = I2cbRegs.I2CDRR.all;       // Read DAN777
    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }


    // Since I2CCNT = 0 at this point, a stop condition will be issued

    *Rvalue1 = (Val1MSB << 8) | (Val1LSB & 0xFF);
    *Rvalue2 = (Val2MSB << 8) | (Val2LSB & 0xFF);
    *Rvalue3 = (Val3MSB << 8) | (Val3LSB & 0xFF);
    *Rvalue4 = (Val4MSB << 8) | (Val4LSB & 0xFF);
    *Rvalue5 = (Val5MSB << 8) | (Val5LSB & 0xFF);
    *Rvalue6 = (Val6MSB << 8) | (Val6LSB & 0xFF);
    *Rvalue7 = (Val7MSB << 8) | (Val7LSB & 0xFF);
    return 0;
}
