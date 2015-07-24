/********************************************************************
 FileName:      main.c
 Dependencies:  See INCLUDES section
 Processor:     PIC18, PIC24, dsPIC, and PIC32 USB Microcontrollers
 Hardware:      This demo is natively intended to be used on Microchip USB demo
                boards supported by the MCHPFSUSB stack.  See release notes for
                support matrix.  This demo can be modified for use on other hardware
                platforms.
 Complier:      Microchip C18 (for PIC18), XC16 (for PIC24/dsPIC), XC32 (for PIC32)
 Company:       Microchip Technology, Inc.

 Software License Agreement:

 The software supplied herewith by Microchip Technology Incorporated
 (the "Company") for its PIC(R) Microcontroller is intended and
 supplied to you, the Company's customer, for use solely and
 exclusively on Microchip PIC Microcontroller products. The
 software is owned by the Company and/or its supplier, and is
 protected under applicable copyright laws. All rights are reserved.
 Any use in violation of the foregoing restrictions may subject the
 user to criminal sanctions under applicable laws, as well as to
 civil liability for the breach of the terms and conditions of this
 license.

 THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.

********************************************************************
 File Description:

 Change History:
  Rev   Description
  ----  ----------------------------------------------------------
  2.5   Initial release of this demo.  Source code
  		was derived from the MCHPFSUSB v2.4
  		"USB Device - WinUSB - Generic Driver Demo"
  		firmware project.

  2.6   Added support for PIC32MX795F512L & PIC24FJ256DA210

  2.6a  Added support for PIC24FJ256GB210

  2.7   No change

  2.7b  Improvements to USBCBSendResume(), to make it easier to use.
  2.9f  Adding new part support
  2.9h  Updated to support MS OS Descriptor for plug and play Win 8 experience
********************************************************************/

/** INCLUDES *******************************************************/
#include "USB/usb.h"
#include "USB/usb_function_generic.h"

#include "HardwareProfile.h"
#include <inttypes.h>
/** CONFIGURATION **************************************************/
#if defined(PIC32MX220F032D_INJECTOR)
    #pragma config UPLLEN   = ON        // USB PLL Enabled
    #pragma config FPLLMUL  = MUL_20        // PLL Multiplier
    #pragma config UPLLIDIV = DIV_2         // USB PLL Input Divider
    #pragma config FPLLIDIV = DIV_2         // PLL Input Divider
    #pragma config FPLLODIV = DIV_2         // PLL Output Divider
    #pragma config FPBDIV   = DIV_1         // Peripheral Clock divisor
    #pragma config FWDTEN   = OFF           // Watchdog Timer
    #pragma config WDTPS    = PS1           // Watchdog Timer Postscale
    #pragma config FCKSM    = CSDCMD        // Clock Switching & Fail Safe Clock Monitor
    #pragma config OSCIOFNC = OFF           // CLKO Enable
    #pragma config POSCMOD  = HS            // Primary Oscillator
    #pragma config IESO     = ON           // Internal/External Switch-over
    #pragma config FSOSCEN  = OFF           // Secondary Oscillator Enable (KLO was off)
    #pragma config FNOSC    = PRIPLL        // Oscillator Selection
    #pragma config CP       = OFF           // Code Protect
    #pragma config BWP      = OFF           // Boot Flash Write Protect
    #pragma config PWP      = OFF           // Program Flash Write Protect
    #pragma config ICESEL   = ICS_PGx3      // ICE/ICD Comm Channel Select

    #pragma config JTAGEN   = OFF      // ICE/ICD Comm Channel Select
    #pragma config FUSBIDIO = OFF      // ICE/ICD Comm Channel Select
    #pragma config FVBUSONIO = OFF      // ICE/ICD Comm Channel Select

#define SYS_FREQ (40000000L)
#define PB_DIV         		1
#define PRESCALE       		8
#define TOGGLES_PER_SEC_1ms	2000
#define TOGGLES_PER_SEC_100us	20000

#define T1_TICK       		(SYS_FREQ/PB_DIV/PRESCALE/TOGGLES_PER_SEC_1ms)

#define T2_TICK       		(SYS_FREQ/PB_DIV/PRESCALE/TOGGLES_PER_SEC_100us)

#else
    #error No hardware board defined, see "HardwareProfile.h"
#endif



/** VARIABLES ******************************************************/
//NOTE:  The below endpoint buffers need to be located in a section of
//system SRAM that is accessible by the USB module.  The USB module on all
//currently existing Microchip USB microcontrollers use a dedicated DMA
//interface for reading/writing USB data into/out of main system SRAM.

//On some USB PIC microcontrollers, all of the microcontroller SRAM is dual
//access, and therefore all of it can be accessed by either the USB 
//module or the microcontroller core.  On other devices, only a certain 
//portion of the SRAM is accessible by the USB module. Therefore, on some 
//devices, it is important to place USB data buffers in certain sections of
//SRAM, while on other devices, the buffers can be placed anywhere.

//USB_VOLATILE BYTE [250];	//User buffer for receiving OUT packets sent from the host
//USB_VOLATILE BYTE EP1OUTEvenBufferEP1OUTOddBuffer[64];	//User buffer for receiving OUT packets sent from the host
//USB_VOLATILE BYTE EP2OUTEvenBuffer[64];	//User buffer for receiving OUT packets sent from the host
//USB_VOLATILE BYTE EP2OUTOddBuffer[64];	//User buffer for receiving OUT packets sent from the host
//USB_VOLATILE BYTE EP3OUTEvenBuffer[64];	//User buffer for receiving OUT packets sent from the host
//USB_VOLATILE BYTE EP3OUTOddBuffer[64];	//User buffer for receiving OUT packets sent from the host
USB_VOLATILE BYTE EP1INEvenBuffer[64];
USB_VOLATILE BYTE EP1INOddBuffer[64];

//The below variables are only accessed by the CPU and can be placed anywhere in RAM.
USB_HANDLE EP1OUTEvenHandle;
USB_HANDLE EP2OUTEvenHandle;
USB_HANDLE EP3OUTEvenHandle;
USB_HANDLE EP1OUTOddHandle;
USB_HANDLE EP2OUTOddHandle;
USB_HANDLE EP3OUTOddHandle;
USB_HANDLE EP1INEvenHandle;
USB_HANDLE EP1INOddHandle;

BOOL EP1INEvenNeedsServicingNext;
BOOL EP1OUTEvenNeedsServicingNext;	//TRUE means even need servicing next, FALSE means odd needs servicing next
BOOL EP2OUTEvenNeedsServicingNext;	//TRUE means even need servicing next, FALSE means odd needs servicing next
BOOL EP3OUTEvenNeedsServicingNext;	//TRUE means even need servicing next, FALSE means odd needs servicing next
WORD led_count;	//Counter for blinking the LEDs on the demo board

/** PRIVATE PROTOTYPES *********************************************/
static void InitializeSystem(void);
void USBDeviceTasks(void);
void YourHighPriorityISRCode(void);
void YourLowPriorityISRCode(void);
void USBCBSendResume(void);
void UserInit(void);
void ProcessIO(void);
void BlinkUSBStatus(void);
void ADC_init(void);

/** DECLARATIONS ***************************************************/

/******************************************************************************
 * Function:        void ISR Yimer(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Main program entry point.
 *
 * Note:            None
 *******************************************************************/
void __ISR(_TIMER_1_VECTOR, ipl2) Timer1Handler(void)
{
    // clear the interrupt flag
    mT1ClearIntFlag();

 //   mPORTAToggleBits(BIT_10);
}

void __ISR(_TIMER_2_VECTOR, ipl2) Timer2Handler(void)
{
    // clear the interrupt flag
    mT2ClearIntFlag();

//    mPORTAToggleBits(BIT_10);
}

/******************************************************************************
 * Function:        void main(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Main program entry point.
 *
 * Note:            None
 *******************************************************************/

int main(void)
{   
    InitializeSystem();

    #if defined(USB_INTERRUPT)
        USBDeviceAttach();
    #endif

    while(1)
    {
        #if defined(USB_POLLING)
		// Check bus status and service USB interrupts.
        USBDeviceTasks(); // Interrupt or polling method.  If using polling, must call
        				  // this function periodically.  This function will take care
        				  // of processing and responding to SETUP transactions 
        				  // (such as during the enumeration process when you first
        				  // plug in).  USB hosts require that USB devices should accept
        				  // and process SETUP packets in a timely fashion.  Therefore,
        				  // when using polling, this function should be called 
        				  // regularly (such as once every 1.8ms or faster** [see 
        				  // inline code comments in usb_device.c for explanation when
        				  // "or faster" applies])  In most cases, the USBDeviceTasks() 
        				  // function does not take very long to execute (ex: <100 
        				  // instruction cycles) before it returns.
        #endif
    				  

		// Application-specific tasks.
		// Application related code may be added here, or in the ProcessIO() function.
        ProcessIO();        
    }//end while
}//end main


/********************************************************************
 * Function:        static void InitializeSystem(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        InitializeSystem is a centralize initialization
 *                  routine. All required USB initialization routines
 *                  are called from here.
 *
 *                  User application initialization routine should
 *                  also be called from here.                  
 *
 * Note:            None
 *******************************************************************/
static void InitializeSystem(void)
{
    #if defined(__C32__)
//        AD1PCFG.w = 0xFFFF;
    #endif

    #if defined(USE_USB_BUS_SENSE_IO)
    tris_usb_bus_sense = INPUT_PIN; // See HardwareProfile.h
    #endif
    
//	If the host PC sends a GetStatus (device) request, the firmware must respond
//	and let the host know if the USB peripheral device is currently bus powered
//	or self powered.  See chapter 9 in the official USB specifications for details
//	regarding this request.  If the peripheral device is capable of being both
//	self and bus powered, it should not return a hard coded value for this request.
//	Instead, firmware should check if it is currently self or bus powered, and
//	respond accordingly.  If the hardware has been configured like demonstrated
//	on the PICDEM FS USB Demo Board, an I/O pin can be polled to determine the
//	currently selected power source.  On the PICDEM FS USB Demo Board, "RA2" 
//	is used for	this purpose.  If using this feature, make sure "USE_SELF_POWER_SENSE_IO"
//	has been defined in HardwareProfile - (platform).h, and that an appropriate I/O pin 
//  has been mapped	to it.
    #if defined(USE_SELF_POWER_SENSE_IO)
    tris_self_power = INPUT_PIN;	// See HardwareProfile.h
    #endif

    OpenTimer1(T1_ON | T1_SOURCE_INT | T1_PS_1_8, T1_TICK);

    // set up the timer interrupt with a priority of 2
    ConfigIntTimer1(T1_INT_ON | T1_INT_PRIOR_2);

//    OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_8, T2_TICK);
//    // set up the timer interrupt with a priority of 2
//    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);

    // enable multi-vector interrupts
    INTEnableSystemMultiVectoredInt();

    // configure output
    mPORTASetPinsDigitalOut(BIT_10);
    mPORTAToggleBits(BIT_10);

    EP1OUTEvenHandle = 0;
    EP2OUTEvenHandle = 0;
    EP3OUTEvenHandle = 0;

    EP1INEvenHandle = 0;
    EP1INOddHandle = 0;

    EP1OUTOddHandle = 0;
    EP2OUTOddHandle = 0;
    EP3OUTOddHandle = 0;

    ADC_init();
    UserInit();			//Application related initialization.  See user.c
    USBDeviceInit();	//usb_device.c.  Initializes USB module SFRs and firmware
    					//variables to known states.
}//end InitializeSystem



void UserInit(void)
{
    led_count=0;
    mInitAllLEDs();
    mInitAllSwitches();
}//end UserInit


/******************************************************************************
 * Function:        void ProcessIO(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is a place holder for other user routines.
 *                  It is a mixture of both USB and non-USB tasks.
 *
 * Note:            None
 *****************************************************************************/
volatile uint16_t bla;
void ProcessIO(void)
{
    //Blink the LEDs according to the USB device status.
    //http://www.microchip.com/forums/m438690-print.aspx
    BlinkUSBStatus();

    if(USBGetDeviceState() == CONFIGURED_STATE) {
        if (EP1INEvenNeedsServicingNext == TRUE)
        {
            if (!USBHandleBusy(EP1INEvenHandle)) //Check if the endpoint has received any data from the host.
            {
                //Re-arm the OUT endpoint for the next packet:IN_TO_HOST 1
                EP1INEvenHandle = USBTransferOnePacket(1, IN_TO_HOST, (BYTE*) & EP1INEvenBuffer, 64);
                EP1INEvenNeedsServicingNext = FALSE;
            }
        }
        else //else EP1OUTOdd needs servicing next
        {
            if (!USBHandleBusy(EP1INOddHandle)) //Check if the endpoint has received any data fromthe host.
            {
                //Re-arm the OUT endpoint for the next packet:IN_TO_HOST 1
                EP1INOddHandle = USBTransferOnePacket(1, IN_TO_HOST, (BYTE*) & EP1INOddBuffer, 64);
                EP1INEvenNeedsServicingNext = TRUE;
            }
        }
    }
}//end ProcessIO

// some local data
volatile int DmaIntFlag = 0; // flag used in interrupts

volatile unsigned short dmaBuff[16 + 1]; // we'll store the received data here

void ADC_DMA_init() {
    DmaChannel chn = DMA_CHANNEL1; // DMA channel to use for our example

    // configure the channel
    DmaChnOpen(chn, DMA_CHN_PRI2, DMA_OPEN_AUTO);

    // set the events: we want the UART2 rx interrupt to start our transfer
    // also we want to enable the pattern match: transfer stops upon detection of CR
    DmaChnSetEventControl(chn, DMA_EV_START_IRQ_EN | DMA_EV_START_IRQ(_ADC_IRQ));

    // set the transfer source and dest addresses, source and dest sizes and the cell size
    DmaChnSetTxfer(chn, (void*) &ADC1BUF0, (void*)dmaBuff, 2, 32, 2);

    DmaChnSetEvEnableFlags(chn, DMA_EV_BLOCK_DONE); // enable the transfer done interrupt: pattern match or all the characters transferred

    // enable system wide multi vectored interrupts
    INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);
    INTEnableInterrupts();

    INTSetVectorPriority(INT_VECTOR_DMA(chn), INT_PRIORITY_LEVEL_5); // set INT controller priority
    INTSetVectorSubPriority(INT_VECTOR_DMA(chn), INT_SUB_PRIORITY_LEVEL_3); // set INT controller sub-priority

    INTEnable(INT_SOURCE_DMA(chn), INT_ENABLED); // enable the chn interrupt in the INT controller

    // enable the chn
    DmaChnEnable(chn);
}

void ADC_DMA_process() {
    while (!DmaIntFlag); // just block here. In a real application you can do some other stuff while the DMA transfer is taking place
    DmaIntFlag = 0; // clear the interrupt flag
}

// handler for the DMA channel 1 interrupt

void __ISR(_DMA1_VECTOR, IPL5SOFT) DmaHandler1(void) {
    int evFlags; // event flags when getting the interrupt

    INTClearFlag(INT_SOURCE_DMA(DMA_CHANNEL1)); // release the interrupt in the INT controller, we're servicing int

    evFlags = DmaChnGetEvFlags(DMA_CHANNEL1); // get the event flags

    if (evFlags & DMA_EV_BLOCK_DONE) { // just a sanity check. we enabled just the DMA_EV_BLOCK_DONE transfer done interrupt
        DmaIntFlag = 1;
        DmaChnClrEvFlags(DMA_CHANNEL1, DMA_EV_BLOCK_DONE);
    }
}

void ADC_init(void)
{
    // configure and enable the ADC
    CloseADC10(); // ensure the ADC is off before setting the configuration

//    AD1CHS = 0x07060000;
    AD1CON1bits.FORM = 0;   // Integer 16-bit (DOUT = 0000 0000 0000 0000 0000 00dd dddd dddd)
    AD1CON1bits.SSRC = 0x7; // 111 = Internal counter ends sampling and starts conversion (auto convert)
    AD1CON1bits.ASAM = 1;   // Sampling begins immediately after last conversion completes
    
    AD1CON2bits.VCFG = 0; //000 AVDD AVSS
    AD1CON2bits.OFFCAL = 0; // Disable Offset Calibration mode. The inputs to the SHA are controlled by AD1CHS or AD1CSSL
    AD1CON2bits.CSCNA = 0; //Do not scan inputs
    AD1CON2bits.SMPI = 7; //generate interupt on every 8 sample
    AD1CON2bits.BUFM = 1; //1 = Buffer configured as two 8-word buffers
    AD1CON2bits.ALTS = 0; //Always use MUX A input multiplexer settings

    //2uS convertion time /40MHz PBdiv = 1
    // convertion time = Tad * 12 + SAMC * Tad
    AD1CON3bits.ADRC = 0; //PB clock source 40MHz
    AD1CON3bits.ADCS = 100; //100ns Tad; Tad = TPB * 2 * (ADCS + 1)
    AD1CON3bits.SAMC = 16; //sample = 8*Tad
//    AD1CON3bits.ADRC = 1; //PB clock source 40MHz
//    AD1CON3bits.ADCS = 0; //100ns Tad; Tad = TPB * 2 * (ADCS + 1)
//    AD1CON3bits.SAMC = 16; //sample = 8*Tad
    
    //MUX B
    AD1CHSbits.CH0NB = 0; // 0 = Channel 0 negative input is VR-
    AD1CHSbits.CH0SB = 0; // Channel 0 positive input is AN0
    //MUX A
    AD1CHSbits.CH0NA = 0;
    AD1CHSbits.CH0SA = 6; //Set PICmx220.AN6 = PINGUINOx220.AN0 channel
        
    ANSELCbits.ANSC0 = 1; //Set PICmx220.AN6 = PINGUINOx220.AN0 to analog
    ANSELCbits.ANSC1 = 1; //Set PICmx220.AN7 = PINGUINOx220.AN1 to analog
    TRISCbits.TRISC0 = 1;
    TRISCbits.TRISC1 = 1;

    AD1CSSLbits.w = 0; // ADC Input Scan Select Register
//    AD1CSSLbits.CSSL6 = 1;
//    AD1CSSLbits.CSSL7 = 1;

    IPC5bits.AD1IP = 6;    //AtoD1 Interrupt Priority
    IPC5bits.AD1IS = 3;    //AtoD1 Subpriority
    IFS0bits.AD1IF = 0;		//Clear irq flag
	
    IEC0bits.AD1IE = 1;    //ADC interrupt enable    EnableADC10(); // Enable the ADC
    EnableADC10();
}

uint8_t samples[10];
int32_t sampleCounter = 0;

void __ISR(_ADC_VECTOR, ipl6) ADCInterruptHandler()
{
    IFS0bits.AD1IF = 0;		//Clear irq flag

    if(AD1CON2bits.BUFS == 0)
    {
        //process lower buffer BUF0 - BUF7
        EP1INEvenBuffer[sampleCounter] = ADC1BUF0 - 512;
        EP1INEvenBuffer[sampleCounter] = ADC1BUF1 - 512;
        EP1INEvenBuffer[sampleCounter] = ADC1BUF2 - 512;
        EP1INEvenBuffer[sampleCounter] = ADC1BUF3 - 512;
        EP1INEvenBuffer[sampleCounter] = ADC1BUF4 - 512;
        EP1INEvenBuffer[sampleCounter] = ADC1BUF5 - 512;
        EP1INEvenBuffer[sampleCounter] = ADC1BUF6 - 512;
        EP1INEvenBuffer[sampleCounter] = ADC1BUF7 - 512;
        EP1INEvenBuffer[sampleCounter] = ADC1BUF8 - 512;
        EP1INEvenBuffer[sampleCounter] = ADC1BUF9 - 512;
        EP1INEvenBuffer[sampleCounter] = ADC1BUFA - 512;
        EP1INEvenBuffer[sampleCounter] = ADC1BUFB - 512;
        EP1INEvenBuffer[sampleCounter] = ADC1BUFC - 512;
        EP1INEvenBuffer[sampleCounter] = ADC1BUFD - 512;
        EP1INEvenBuffer[sampleCounter] = ADC1BUFE - 512;
        EP1INEvenBuffer[sampleCounter] = ADC1BUFF - 512;
    }
    else
    {
        //process higher buffer BUF8 - BUF15
        EP1INEvenBuffer[sampleCounter] = ADC1BUF0 - 512;
        EP1INEvenBuffer[sampleCounter] = ADC1BUF1 - 512;
        EP1INEvenBuffer[sampleCounter] = ADC1BUF2 - 512;
        EP1INEvenBuffer[sampleCounter] = ADC1BUF3 - 512;
        EP1INEvenBuffer[sampleCounter] = ADC1BUF4 - 512;
        EP1INEvenBuffer[sampleCounter] = ADC1BUF5 - 512;
        EP1INEvenBuffer[sampleCounter] = ADC1BUF6 - 512;
        EP1INEvenBuffer[sampleCounter] = ADC1BUF7 - 512;
        EP1INEvenBuffer[sampleCounter] = ADC1BUF8 - 512;
        EP1INEvenBuffer[sampleCounter] = ADC1BUF9 - 512;
        EP1INEvenBuffer[sampleCounter] = ADC1BUFA - 512;
        EP1INEvenBuffer[sampleCounter] = ADC1BUFB - 512;
        EP1INEvenBuffer[sampleCounter] = ADC1BUFC - 512;
        EP1INEvenBuffer[sampleCounter] = ADC1BUFD - 512;
        EP1INEvenBuffer[sampleCounter] = ADC1BUFE - 512;
        EP1INEvenBuffer[sampleCounter] = ADC1BUFF - 512;
    }

    if(++sampleCounter > 63){
        sampleCounter = 0;
    }
}

/********************************************************************
 * Function:        void BlinkUSBStatus(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        BlinkUSBStatus turns on and off LEDs 
 *                  corresponding to the USB device state.
 *
 * Note:            mLED macros can be found in HardwareProfile.h
 *                  USBDeviceState is declared and updated in
 *                  usb_device.c.
 *******************************************************************/
void BlinkUSBStatus(void)
{
    #define mLED_Both_Off()         {mLED_1_Off();mLED_2_Off();}
    #define mLED_Both_On()          {mLED_1_On();mLED_2_On();}
    #define mLED_Only_1_On()        {mLED_1_On();mLED_2_Off();}
    #define mLED_Only_2_On()        {mLED_1_Off();mLED_2_On();}

    if(USBSuspendControl == 1)	//If device is in USB suspend, then need to
    {							//make LEDs take minimal average power.
    	if(led_count == 0)
    		led_count = 15;
    	led_count--;

	    mLED_Both_Off();
	    if(led_count==0)
	    {
		    mLED_Both_On();		//Very breifly turn on the LEDs, with very low duty cycle
		    Nop();				//so as to avoid consuming excess average current.
		    Nop();				
   		    mLED_Both_Off();		
		}  
    }
    else
    {
	    if(led_count == 0)
		{
    		led_count = 10000U;
    	}
    	led_count--;
    	
        if(USBDeviceState == DETACHED_STATE)
        {
            mLED_Both_Off();
        }
        else if(USBDeviceState == ATTACHED_STATE)
        {
            mLED_Both_On();
        }
        else if(USBDeviceState == POWERED_STATE)
        {
            mLED_Only_1_On();
        }
        else if(USBDeviceState == DEFAULT_STATE)
        {
            mLED_Only_2_On();
        }
        else if(USBDeviceState == ADDRESS_STATE)
        {
            if(led_count == 0)
            {
                mLED_1_Toggle();
                mLED_2_Off();
            }//end if
        }
        else if(USBDeviceState == CONFIGURED_STATE)
        {
            if(led_count==0)
            {
                mLED_1_Toggle();
                if(mGetLED_1())
                {
                    mLED_2_Off();
                }
                else
                {
                    mLED_2_On();
                }
            }//end if
        }//end if(...)
    }//end if(UCONbits.SUSPND...)

}//end BlinkUSBStatus




// ******************************************************************************************************
// ************** USB Callback Functions ****************************************************************
// ******************************************************************************************************
// The USB firmware stack will call the callback functions USBCBxxx() in response to certain USB related
// events.  For example, if the host PC is powering down, it will stop sending out Start of Frame (SOF)
// packets to your device.  In response to this, all USB devices are supposed to decrease their power
// consumption from the USB Vbus to <2.5mA each.  The USB module detects this condition (which according
// to the USB specifications is 3+ms of no bus activity/SOF packets) and then calls the USBCBSuspend()
// function.  You should modify these callback functions to take appropriate actions for each of these
// conditions.  For example, in the USBCBSuspend(), you may wish to add code that will decrease power
// consumption from Vbus to <2.5mA (such as by clock switching, turning off LEDs, putting the
// microcontroller to sleep, etc.).  Then, in the USBCBWakeFromSuspend() function, you may then wish to
// add code that undoes the power saving things done in the USBCBSuspend() function.

// The USBCBSendResume() function is special, in that the USB stack will not automatically call this
// function.  This function is meant to be called from the application firmware instead.  See the
// additional comments near the function.

// Note *: The "usb_20.pdf" specs indicate 500uA or 2.5mA, depending upon device classification. However,
// the USB-IF has officially issued an ECN (engineering change notice) changing this to 2.5mA for all 
// devices.  Make sure to re-download the latest specifications to get all of the newest ECNs.

/******************************************************************************
 * Function:        void USBCBSuspend(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Call back that is invoked when a USB suspend is detected
 *
 * Note:            None
 *****************************************************************************/
void USBCBSuspend(void)
{
    //Example power saving code.  Insert appropriate code here for the desired
    //application behavior.  If the microcontroller will be put to sleep, a
    //process similar to that shown below may be used:

    //ConfigureIOPinsForLowPower();
    //SaveStateOfAllInterruptEnableBits();
    //DisableAllInterruptEnableBits();
    //EnableOnlyTheInterruptsWhichWillBeUsedToWakeTheMicro();	//should enable at least USBActivityIF as a wake source
    //Sleep();
    //RestoreStateOfAllPreviouslySavedInterruptEnableBits();	//Preferrably, this should be done in the USBCBWakeFromSuspend() function instead.
    //RestoreIOPinsToNormal();									//Preferrably, this should be done in the USBCBWakeFromSuspend() function instead.

    //Alternatively, the microcontorller may use clock switching to reduce current consumption.

    //IMPORTANT NOTE: Do not clear the USBActivityIF (ACTVIF) bit here.  This bit is
    //cleared inside the usb_device.c file.  Clearing USBActivityIF here will cause
    //things to not work as intended.
}

/******************************************************************************
 * Function:        void USBCBWakeFromSuspend(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The host may put USB peripheral devices in low power
 *					suspend mode (by "sending" 3+ms of idle).  Once in suspend
 *					mode, the host may wake the device back up by sending non-
 *					idle state signalling.
 *					
 *					This call back is invoked when a wakeup from USB suspend 
 *					is detected.
 *
 * Note:            None
 *****************************************************************************/
void USBCBWakeFromSuspend(void)
{
	// If clock switching or other power savings measures were taken when
	// executing the USBCBSuspend() function, now would be a good time to
	// switch back to normal full power run mode conditions.  The host allows
	// 10+ milliseconds of wakeup time, after which the device must be 
	// fully back to normal, and capable of receiving and processing USB
	// packets.  In order to do this, the USB module must receive proper
	// clocking (IE: 48MHz clock must be available to SIE for full speed USB
	// operation).  
	// Make sure the selected oscillator settings are consistent with USB 
    // operation before returning from this function.

	//Switch clock back to main clock source necessary for USB operation
	//Previous clock source was something low frequency as set in the 
	//USBCBSuspend() function.
}

/********************************************************************
 * Function:        void USBCB_SOF_Handler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USB host sends out a SOF packet to full-speed
 *                  devices every 1 ms. This interrupt may be useful
 *                  for isochronous pipes. End designers should
 *                  implement callback routine as necessary.
 *
 * Note:            None
 *******************************************************************/
void USBCB_SOF_Handler(void)
{
    // No need to clear UIRbits.SOFIF to 0 here.
    // Callback caller is already doing that.
}

/*******************************************************************
 * Function:        void USBCBErrorHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The purpose of this callback is mainly for
 *                  debugging during development. Check UEIR to see
 *                  which error causes the interrupt.
 *
 * Note:            None
 *******************************************************************/
void USBCBErrorHandler(void)
{
    // No need to clear UEIR to 0 here.
    // Callback caller is already doing that.

	// Typically, user firmware does not need to do anything special
	// if a USB error occurs.  For example, if the host sends an OUT
	// packet to your device, but the packet gets corrupted (ex:
	// because of a bad connection, or the user unplugs the
	// USB cable during the transmission) this will typically set
	// one or more USB error interrupt flags.  Nothing specific
	// needs to be done however, since the SIE will automatically
	// send a "NAK" packet to the host.  In response to this, the
	// host will normally retry to send the packet again, and no
	// data loss occurs.  The system will typically recover
	// automatically, without the need for application firmware
	// intervention.
	
	// Nevertheless, this callback function is provided, such as
	// for debugging purposes.
}


/*******************************************************************
 * Function:        void USBCBCheckOtherReq(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        When SETUP packets arrive from the host, some
 * 					firmware must process the request and respond
 *					appropriately to fulfill the request.  Some of
 *					the SETUP packets will be for standard
 *					USB "chapter 9" (as in, fulfilling chapter 9 of
 *					the official USB specifications) requests, while
 *					others may be specific to the USB device class
 *					that is being implemented.  For example, a HID
 *					class device needs to be able to respond to
 *					"GET REPORT" type of requests.  This
 *					is not a standard USB chapter 9 request, and 
 *					therefore not handled by usb_device.c.  Instead
 *					this request should be handled by class specific 
 *					firmware, such as that contained in usb_function_hid.c.
 *
 * Note:            None
 *****************************************************************************/
void USBCBCheckOtherReq(void)
{
    //Check for class specific requests, and if necessary, handle it.
    USBCheckVendorRequest();
}//end


/*******************************************************************
 * Function:        void USBCBStdSetDscHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USBCBStdSetDscHandler() callback function is
 *					called when a SETUP, bRequest: SET_DESCRIPTOR request
 *					arrives.  Typically SET_DESCRIPTOR requests are
 *					not used in most applications, and it is
 *					optional to support this type of request.
 *
 * Note:            None
 *****************************************************************************/
void USBCBStdSetDscHandler(void)
{
    // Must claim session ownership if supporting this request
}//end


/******************************************************************************
 * Function:        void USBCBInitEP(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called when the device becomes
 *                  initialized, which occurs after the host sends a
 * 					SET_CONFIGURATION (wValue not = 0) request.  This 
 *					callback function should initialize the endpoints 
 *					for the device's usage according to the current 
 *					configuration.
 *
 * Note:            None
 *****************************************************************************/
void USBCBInitEP(void)
{
    USBEnableEndpoint(1, USB_IN_ENABLED | USB_HANDSHAKE_ENABLED | USB_DISALLOW_SETUP);
    USBEnableEndpoint(2, USB_OUT_ENABLED | USB_HANDSHAKE_ENABLED | USB_DISALLOW_SETUP);

    EP2OUTEvenHandle = USBTransferOnePacket(1, OUT_FROM_HOST,(BYTE*)&EP1INEvenBuffer,64);
    EP2OUTOddHandle = USBTransferOnePacket(1, OUT_FROM_HOST,(BYTE*)&EP1INOddBuffer,64);
    EP1INEvenNeedsServicingNext = TRUE;
    EP2OUTEvenNeedsServicingNext = TRUE;
}

/********************************************************************
 * Function:        void USBCBSendResume(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USB specifications allow some types of USB
 * 					peripheral devices to wake up a host PC (such
 *					as if it is in a low power suspend to RAM state).
 *					This can be a very useful feature in some
 *					USB applications, such as an Infrared remote
 *					control	receiver.  If a user presses the "power"
 *					button on a remote control, it is nice that the
 *					IR receiver can detect this signalling, and then
 *					send a USB "command" to the PC to wake up.
 *					
 *					The USBCBSendResume() "callback" function is used
 *					to send this special USB signalling which wakes 
 *					up the PC.  This function may be called by
 *					application firmware to wake up the PC.  This
 *					function will only be able to wake up the host if
 *                  all of the below are true:
 *					
 *					1.  The USB driver used on the host PC supports
 *						the remote wakeup capability.
 *					2.  The USB configuration descriptor indicates
 *						the device is remote wakeup capable in the
 *						bmAttributes field.
 *					3.  The USB host PC is currently sleeping,
 *						and has previously sent your device a SET 
 *						FEATURE setup packet which "armed" the
 *						remote wakeup capability.   
 *
 *                  If the host has not armed the device to perform remote wakeup,
 *                  then this function will return without actually performing a
 *                  remote wakeup sequence.  This is the required behavior, 
 *                  as a USB device that has not been armed to perform remote 
 *                  wakeup must not drive remote wakeup signalling onto the bus;
 *                  doing so will cause USB compliance testing failure.
 *                  
 *					This callback should send a RESUME signal that
 *                  has the period of 1-15ms.
 *
 * Note:            This function does nothing and returns quickly, if the USB
 *                  bus and host are not in a suspended condition, or are 
 *                  otherwise not in a remote wakeup ready state.  Therefore, it
 *                  is safe to optionally call this function regularly, ex: 
 *                  anytime application stimulus occurs, as the function will
 *                  have no effect, until the bus really is in a state ready
 *                  to accept remote wakeup. 
 *
 *                  When this function executes, it may perform clock switching,
 *                  depending upon the application specific code in 
 *                  USBCBWakeFromSuspend().  This is needed, since the USB
 *                  bus will no longer be suspended by the time this function
 *                  returns.  Therefore, the USB module will need to be ready
 *                  to receive traffic from the host.
 *
 *                  The modifiable section in this routine may be changed
 *                  to meet the application needs. Current implementation
 *                  temporary blocks other functions from executing for a
 *                  period of ~3-15 ms depending on the core frequency.
 *
 *                  According to USB 2.0 specification section 7.1.7.7,
 *                  "The remote wakeup device must hold the resume signaling
 *                  for at least 1 ms but for no more than 15 ms."
 *                  The idea here is to use a delay counter loop, using a
 *                  common value that would work over a wide range of core
 *                  frequencies.
 *                  That value selected is 1800. See table below:
 *                  ==========================================================
 *                  Core Freq(MHz)      MIP         RESUME Signal Period (ms)
 *                  ==========================================================
 *                      48              12          1.05
 *                       4              1           12.6
 *                  ==========================================================
 *                  * These timing could be incorrect when using code
 *                    optimization or extended instruction mode,
 *                    or when having other interrupts enabled.
 *                    Make sure to verify using the MPLAB SIM's Stopwatch
 *                    and verify the actual signal on an oscilloscope.
 *******************************************************************/
void USBCBSendResume(void)
{
    static WORD delay_count;
    
    //First verify that the host has armed us to perform remote wakeup.
    //It does this by sending a SET_FEATURE request to enable remote wakeup,
    //usually just before the host goes to standby mode (note: it will only
    //send this SET_FEATURE request if the configuration descriptor declares
    //the device as remote wakeup capable, AND, if the feature is enabled
    //on the host (ex: on Windows based hosts, in the device manager 
    //properties page for the USB device, power management tab, the 
    //"Allow this device to bring the computer out of standby." checkbox 
    //should be checked).
    if(USBGetRemoteWakeupStatus() == TRUE) 
    {
        //Verify that the USB bus is in fact suspended, before we send
        //remote wakeup signalling.
        if(USBIsBusSuspended() == TRUE)
        {
            USBMaskInterrupts();
            
            //Clock switch to settings consistent with normal USB operation.
            USBCBWakeFromSuspend();
            USBSuspendControl = 0; 
            USBBusIsSuspended = FALSE;  //So we don't execute this code again, 
                                        //until a new suspend condition is detected.

            //Section 7.1.7.7 of the USB 2.0 specifications indicates a USB
            //device must continuously see 5ms+ of idle on the bus, before it sends
            //remote wakeup signalling.  One way to be certain that this parameter
            //gets met, is to add a 2ms+ blocking delay here (2ms plus at 
            //least 3ms from bus idle to USBIsBusSuspended() == TRUE, yeilds
            //5ms+ total delay since start of idle).
            delay_count = 3600U;        
            do
            {
                delay_count--;
            }while(delay_count);
            
            //Now drive the resume K-state signalling onto the USB bus.
            USBResumeControl = 1;       // Start RESUME signaling
            delay_count = 1800U;        // Set RESUME line for 1-13 ms
            do
            {
                delay_count--;
            }while(delay_count);
            USBResumeControl = 0;       //Finished driving resume signalling

            USBUnmaskInterrupts();
        }
    }
}


/*******************************************************************
 * Function:        BOOL USER_USB_CALLBACK_EVENT_HANDLER(
 *                        int event, void *pdata, WORD size)
 *
 * PreCondition:    None
 *
 * Input:           int event - the type of event
 *                  void *pdata - pointer to the event data
 *                  WORD size - size of the event data
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called from the USB stack to
 *                  notify a user application that a USB event
 *                  occured.  This callback is in interrupt context
 *                  when the USB_INTERRUPT option is selected.
 *
 * Note:            None
 *******************************************************************/
BOOL USER_USB_CALLBACK_EVENT_HANDLER(int event, void *pdata, WORD size)
{
    switch( event )
    {
        case EVENT_TRANSFER:
            //Add application specific callback task or callback function here if desired.
            break;
        case EVENT_SOF:
            USBCB_SOF_Handler();
            break;
        case EVENT_SUSPEND:
            USBCBSuspend();
            break;
        case EVENT_RESUME:
            USBCBWakeFromSuspend();
            break;
        case EVENT_CONFIGURED: 
            USBCBInitEP();
            break;
        case EVENT_SET_DESCRIPTOR:
            USBCBStdSetDscHandler();
            break;
        case EVENT_EP0_REQUEST:
            USBCBCheckOtherReq();
            break;
        case EVENT_BUS_ERROR:
            USBCBErrorHandler();
            break;
        case EVENT_TRANSFER_TERMINATED:
            //Add application specific callback task or callback function here if desired.
            //The EVENT_TRANSFER_TERMINATED event occurs when the host performs a CLEAR
            //FEATURE (endpoint halt) request on an application endpoint which was 
            //previously armed (UOWN was = 1).  Here would be a good place to:
            //1.  Determine which endpoint the transaction that just got terminated was 
            //      on, by checking the handle value in the *pdata.
            //2.  Re-arm the endpoint if desired (typically would be the case for OUT 
            //      endpoints).
            break;
        default:
            break;
    }      
    return TRUE; 
}
/** EOF main.c ***************************************************************/
