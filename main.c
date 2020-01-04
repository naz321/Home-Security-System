#include "main.h"
#include "driverlib/driverlib.h"
#include "hal_LCD.h"
#include "driverlib/timer_a.h"
#include <stdio.h>

char ADCState = 0; //Busy state of the ADC
int16_t ADCResult = 0; //Storage for the ADC conversion result
int zoneNum = 0;
int reset = 0;
char display[4] = "GGGG";
char time_display[] = {'0','1','2','3','4','5','6','7','8','9'};

void Alarm();
void Reset_Alarm();
void TriggerZoneNotOkay(int zone);
int TriggerZone();
int passed = 0;
void TurnOnGreenLED();
void TurnOnYellowLED();
void ADC_ISR(void);
void Init_RTC(void);
void displayText(char *msg);
int RTCTime(void);
void RTCCountdown(void);
int countdown_value =0;

enum states{ZONE_ARMED, ZONE_NOT_OKAY, ZONE_NOT_ARMED, MONITOR};

void main(void)
{
    enum states zoneState = ZONE_NOT_ARMED;

    //Turn off interrupts during initialization
    __disable_interrupt();

    //Stop watchdog timer unless you plan on using it
    WDT_A_hold(WDT_A_BASE);

    // Initializations - see functions for more detail
    Init_GPIO();    //Sets all pins to output low as a default
    Init_Clock();   //Sets up the necessary system clocks
    Init_UART();    //Sets up an echo over a COM port
    Init_LCD();     //Sets up the LaunchPad LCD display

    //used for audio transducer
    Init_PWM();     //Sets up a PWM output
    Init_ADC();     //Sets up the ADC to sample

    //RTC Inits
    Init_RTC();


    PMM_unlockLPM5(); //Disable the GPIO power-on default high-impedance mode to activate previously configured port settings

    //All done initializations - turn interrupts back on.
    __enable_interrupt();


    //P1.6 set as output for zone 1 red led
    GPIO_setAsOutputPin(GPIO_PORT_P1,GPIO_PIN6);
    //P5.0 set as output for zone 1 yellow, green led and inverter
    GPIO_setAsOutputPin(GPIO_PORT_P5,GPIO_PIN0);
    //P5.2 set as output for zone 2 red led
    GPIO_setAsOutputPin(GPIO_PORT_P5,GPIO_PIN2);
    //P5.3 set as output for zone 2 yellow, green led and inverter
    GPIO_setAsOutputPin(GPIO_PORT_P5,GPIO_PIN3);
    //P1.4 set as output for zone 3 red led
    GPIO_setAsOutputPin(GPIO_PORT_P1,GPIO_PIN4);
    //P1.5 set as output for zone 3 yellow, green led and inverter
    GPIO_setAsOutputPin(GPIO_PORT_P1,GPIO_PIN5);
    //P8.2 set as output for zone 4 red led
    GPIO_setAsOutputPin(GPIO_PORT_P8,GPIO_PIN2);
    //P8.3 set as output for zone 4 yellow, green led and inverter
    GPIO_setAsOutputPin(GPIO_PORT_P8,GPIO_PIN3);

    //P2.7 set as input for reed switch 4
    GPIO_setAsInputPin(GPIO_PORT_P2,GPIO_PIN7);
    //P8.0 set as input for reed switch 3
    GPIO_setAsInputPin(GPIO_PORT_P8,GPIO_PIN0);
    //P5.1 set as input for reed switch 2
    GPIO_setAsInputPin(GPIO_PORT_P5,GPIO_PIN1);
    //P2.5 set as input for reed switch 1
    GPIO_setAsInputPin(GPIO_PORT_P2,GPIO_PIN5);

    while(1){
        switch(zoneState){
        case ZONE_NOT_ARMED:
            //turn on yellow led
            TurnOnYellowLED();
            countdown_value = RTCTime();
            zoneState = ZONE_ARMED;
            RTCCountdown();

            break;

        case ZONE_NOT_OKAY:
            //turn on red led
            //turn on the buzzer
            //write to LCD which zone has been triggered
            TriggerZoneNotOkay(zoneNum);
            zoneState = MONITOR;
            break;

        case ZONE_ARMED:
            //turn on green led
            TurnOnGreenLED();
            displayScrollText("ZONES ARMED");
            zoneState = MONITOR;
            break;

        case MONITOR:
            //check if anything breached the reed switches. If it did, go to ZONE_NOT_OK, else stay in this state until rtc has been disabled
            showChar(display[0], pos1);
            showChar(display[1], pos2);
            showChar(display[2], pos3);
            showChar(display[3], pos4);


            if(TriggerZone()){
                zoneState = ZONE_NOT_OKAY;
            }else{
                if ((GPIO_getInputPinValue(SW1_PORT, SW1_PIN) == 0)) {
                    Reset_Alarm();
                    zoneState = ZONE_NOT_ARMED;
                    displayScrollText("ZONES UNARMED");
                    clearLCD();
                }
                else{
                    zoneState = MONITOR;
                }
            }

            break;
        }
    }
}

int RTCTime(){
    int timer_value = 10;
    unsigned int lcd_left =0;
    int button1 =0;
    int button2 =0;
    char display_time[2] = "00";

    showChar(display_time[0], pos1);
    showChar(display_time[1], pos2);
    while(1){

        button1 = (GPIO_getInputPinValue(SW1_PORT, SW1_PIN));
        button2 = (GPIO_getInputPinValue(SW2_PORT, SW2_PIN));

        if(button1==0 && button2==0){
            break;
        }

        if ((GPIO_getInputPinValue(SW1_PORT, SW1_PIN) == 0) && lcd_left > 0){
            lcd_left--;
            showChar(time_display[lcd_left], pos1);
            __delay_cycles(500000);
        }

        if ((GPIO_getInputPinValue(SW2_PORT, SW2_PIN) == 0) && lcd_left < 6){
            lcd_left++;
            showChar(time_display[lcd_left], pos1);
            __delay_cycles(500000);
        }
    }

    timer_value = lcd_left*10;
    return timer_value;
}

void RTCCountdown(){
    char display_countdown[] = {'0', '0'};

    while(countdown_value>=0){
        if(passed){
            passed = 0;
            sprintf(display_countdown, "%d", countdown_value);
            if(countdown_value>=10){
                showChar(display_countdown[0], pos1);
                showChar(display_countdown[1], pos2);
                __delay_cycles(1000000);
            }
            else{
                showChar('0', pos1);
                showChar(display_countdown[0], pos2);
                __delay_cycles(1000000);
            }
            countdown_value--;
        }
    }
}


void TriggerZoneNotOkay(int zone){
    switch(zone){
    case 1:
        GPIO_setOutputHighOnPin(GPIO_PORT_P1,GPIO_PIN6);
        display[0] = 'B';
        Alarm();
        break;

    case 2:
        GPIO_setOutputHighOnPin(GPIO_PORT_P5,GPIO_PIN2);
        display[1] = 'B';
        Alarm();
        break;

    case 3:
        GPIO_setOutputHighOnPin(GPIO_PORT_P1,GPIO_PIN4);
        display[2] = 'B';
        Alarm();
        break;

    case 4:
        GPIO_setOutputHighOnPin(GPIO_PORT_P8,GPIO_PIN2);
        display[3] = 'B';
        Alarm();
        break;

    case 5:
        GPIO_setOutputHighOnPin(GPIO_PORT_P1,GPIO_PIN6);
        GPIO_setOutputHighOnPin(GPIO_PORT_P5,GPIO_PIN2);
        GPIO_setOutputHighOnPin(GPIO_PORT_P1,GPIO_PIN4);
        GPIO_setOutputHighOnPin(GPIO_PORT_P8,GPIO_PIN2);
        sprintf(display, "BBBB");

        Alarm();
        break;
    }
}

void TurnOnGreenLED(){
    //set as low to turn on green LEDs since they are inverted
    GPIO_setOutputLowOnPin(GPIO_PORT_P5,GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5,GPIO_PIN3);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN5);
    GPIO_setOutputLowOnPin(GPIO_PORT_P8,GPIO_PIN3);
}

void TurnOnYellowLED(){
    //set as high to turn on yellow LEDs
    GPIO_setOutputHighOnPin(GPIO_PORT_P5,GPIO_PIN0);
    GPIO_setOutputHighOnPin(GPIO_PORT_P5,GPIO_PIN3);
    GPIO_setOutputHighOnPin(GPIO_PORT_P1,GPIO_PIN5);
    GPIO_setOutputHighOnPin(GPIO_PORT_P8,GPIO_PIN3);
}

int TriggerZone(){
    if((GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN5)==0)){
        zoneNum = 1;
        return 1;
    }

    if((GPIO_getInputPinValue(GPIO_PORT_P5, GPIO_PIN1)==0)){
        zoneNum = 2;
        return 1;
    }

    if((GPIO_getInputPinValue(GPIO_PORT_P8, GPIO_PIN0)==0)){
        zoneNum = 3;
        return 1;
    }

    if((GPIO_getInputPinValue(GPIO_PORT_P2,GPIO_PIN7)==0)){
        zoneNum = 4;
        return 1;
    }



    //listening to mic and amp
    ADCState = 1; //Set flag to indicate ADC is busy - ADC ISR (interrupt) will clear it
    ADC_startConversion(ADC_BASE, ADC_SINGLECHANNEL);

    if ((int)ADCResult > 800){
        zoneNum = 5;
        return 1;
    }

    return 0;
}

void Alarm(){
    Timer_A_outputPWM(TIMER_A0_BASE, &param);
}

void Reset_Alarm(){
    //turn off all output pins
    GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN6);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5,GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5,GPIO_PIN2);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5,GPIO_PIN3);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN4);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN5);
    GPIO_setOutputLowOnPin(GPIO_PORT_P8,GPIO_PIN2);
    GPIO_setOutputLowOnPin(GPIO_PORT_P8,GPIO_PIN3);


    sprintf(display, "GGGG");
    Timer_A_stop(TIMER_A0_BASE);

}

void Init_GPIO(void)
{
    // Set all GPIO pins to output low to prevent floating input and reduce power consumption
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);

    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);

    //Set LaunchPad switches as inputs - they are active low, meaning '1' until pressed
    GPIO_setAsInputPinWithPullUpResistor(SW1_PORT, SW1_PIN);
    GPIO_setAsInputPinWithPullUpResistor(SW2_PORT, SW2_PIN);

    //Set LED1 and LED2 as outputs
    //GPIO_setAsOutputPin(LED1_PORT, LED1_PIN); //Comment if using UART
    GPIO_setAsOutputPin(LED2_PORT, LED2_PIN);
}

/* Clock System Initialization */
void Init_Clock(void)
{
    /*
     * The MSP430 has a number of different on-chip clocks. You can read about it in
     * the section of the Family User Guide regarding the Clock System ('cs.h' in the
     * driverlib).
     */

    /*
     * On the LaunchPad, there is a 32.768 kHz crystal oscillator used as a
     * Real Time Clock (RTC). It is a quartz crystal connected to a circuit that
     * resonates it. Since the frequency is a power of two, you can use the signal
     * to drive a counter, and you know that the bits represent binary fractions
     * of one second. You can then have the RTC module throw an interrupt based
     * on a 'real time'. E.g., you could have your system sleep until every
     * 100 ms when it wakes up and checks the status of a sensor. Or, you could
     * sample the ADC once per second.
     */
    //Set P4.1 and P4.2 as Primary Module Function Input, XT_LF
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN1 + GPIO_PIN2, GPIO_PRIMARY_MODULE_FUNCTION);

    // Set external clock frequency to 32.768 KHz
    CS_setExternalClockSource(32768);
    // Set ACLK = XT1
    CS_initClockSignal(CS_ACLK, CS_XT1CLK_SELECT, CS_CLOCK_DIVIDER_1);
    // Initializes the XT1 crystal oscillator
    CS_turnOnXT1LF(CS_XT1_DRIVE_1);
    // Set SMCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_SMCLK, CS_DCOCLKDIV_SELECT, CS_CLOCK_DIVIDER_1);
    // Set MCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_MCLK, CS_DCOCLKDIV_SELECT, CS_CLOCK_DIVIDER_1);
}

/* UART Initialization */
void Init_UART(void)
{
    /* UART: It configures P1.0 and P1.1 to be connected internally to the
     * eSCSI module, which is a serial communications module, and places it
     * in UART mode. This let's you communicate with the PC via a software
     * COM port over the USB cable. You can use a console program, like PuTTY,
     * to type to your LaunchPad. The code in this sample just echos back
     * whatever character was received.
     */

    //Configure UART pins, which maps them to a COM port over the USB cable
    //Set P1.0 and P1.1 as Secondary Module Function Input.
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN1, GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P1, GPIO_PIN0, GPIO_PRIMARY_MODULE_FUNCTION);

    /*
     * UART Configuration Parameter. These are the configuration parameters to
     * make the eUSCI A UART module to operate with a 9600 baud rate. These
     * values were calculated using the online calculator that TI provides at:
     * http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
     */

    //SMCLK = 1MHz, Baudrate = 9600
    //UCBRx = 6, UCBRFx = 8, UCBRSx = 17, UCOS16 = 1
    EUSCI_A_UART_initParam param = {0};
        param.selectClockSource = EUSCI_A_UART_CLOCKSOURCE_SMCLK;
        param.clockPrescalar    = 6;
        param.firstModReg       = 8;
        param.secondModReg      = 17;
        param.parity            = EUSCI_A_UART_NO_PARITY;
        param.msborLsbFirst     = EUSCI_A_UART_LSB_FIRST;
        param.numberofStopBits  = EUSCI_A_UART_ONE_STOP_BIT;
        param.uartMode          = EUSCI_A_UART_MODE;
        param.overSampling      = 1;

    if(STATUS_FAIL == EUSCI_A_UART_init(EUSCI_A0_BASE, &param))
    {
        return;
    }

    EUSCI_A_UART_enable(EUSCI_A0_BASE);

    EUSCI_A_UART_clearInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);

    // Enable EUSCI_A0 RX interrupt
    EUSCI_A_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
}

/* EUSCI A0 UART ISR - Echoes data back to PC host */
#pragma vector=USCI_A0_VECTOR
__interrupt
void EUSCIA0_ISR(void)
{
    uint8_t RxStatus = EUSCI_A_UART_getInterruptStatus(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG);

    EUSCI_A_UART_clearInterrupt(EUSCI_A0_BASE, RxStatus);

    if (RxStatus)
    {
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, EUSCI_A_UART_receiveData(EUSCI_A0_BASE));
    }
}

/* PWM Initialization */
void Init_PWM(void)
{
    /*
     * The internal timers (TIMER_A) can auto-generate a PWM signal without needing to
     * flip an output bit every cycle in software. The catch is that it limits which
     * pins you can use to output the signal, whereas manually flipping an output bit
     * means it can be on any GPIO. This function populates a data structure that tells
     * the API to use the timer as a hardware-generated PWM source.
     *
     */
    //Generate PWM - Timer runs in Up-Down mode
    param.clockSource           = TIMER_A_CLOCKSOURCE_SMCLK;
    param.clockSourceDivider    = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    param.timerPeriod           = TIMER_A_PERIOD; //Defined in main.h
    param.compareRegister       = TIMER_A_CAPTURECOMPARE_REGISTER_1;
    param.compareOutputMode     = TIMER_A_OUTPUTMODE_RESET_SET;
    param.dutyCycle             = HIGH_COUNT; //Defined in main.h

    //PWM_PORT PWM_PIN (defined in main.h) as PWM output
    GPIO_setAsPeripheralModuleFunctionOutputPin(PWM_PORT, PWM_PIN, GPIO_PRIMARY_MODULE_FUNCTION);
}

void Init_ADC(void)
{
    /*
     * To use the ADC, you need to tell a physical pin to be an analog input instead
     * of a GPIO, then you need to tell the ADC to use that analog input. Defined
     * these in main.h for A9 on P8.1.
     */

    //Set ADC_IN to input direction
    GPIO_setAsPeripheralModuleFunctionInputPin(ADC_IN_PORT, ADC_IN_PIN, GPIO_PRIMARY_MODULE_FUNCTION);

    //Initialize the ADC Module
    /*
     * Base Address for the ADC Module
     * Use internal ADC bit as sample/hold signal to start conversion
     * USE MODOSC 5MHZ Digital Oscillator as clock source
     * Use default clock divider of 1
     */
    ADC_init(ADC_BASE,
             ADC_SAMPLEHOLDSOURCE_SC,
             ADC_CLOCKSOURCE_ADCOSC,
             ADC_CLOCKDIVIDER_1);

    ADC_enable(ADC_BASE);

    /*
     * Base Address for the ADC Module
     * Sample/hold for 16 clock cycles
     * Do not enable Multiple Sampling
     */
    ADC_setupSamplingTimer(ADC_BASE,
                           ADC_CYCLEHOLD_16_CYCLES,
                           ADC_MULTIPLESAMPLESDISABLE);

    //Configure Memory Buffer
    /*
     * Base Address for the ADC Module
     * Use input ADC_IN_CHANNEL
     * Use positive reference of AVcc
     * Use negative reference of AVss
     */
    ADC_configureMemory(ADC_BASE,
                        ADC_IN_CHANNEL,
                        ADC_VREFPOS_AVCC,
                        ADC_VREFNEG_AVSS);

    ADC_clearInterrupt(ADC_BASE,
                       ADC_COMPLETED_INTERRUPT);

    //Enable Memory Buffer interrupt
    ADC_enableInterrupt(ADC_BASE,
                        ADC_COMPLETED_INTERRUPT);
}

//ADC interrupt service routine
#pragma vector=ADC_VECTOR
__interrupt
void ADC_ISR(void)
{
    uint8_t ADCStatus = ADC_getInterruptStatus(ADC_BASE, ADC_COMPLETED_INTERRUPT_FLAG);

    ADC_clearInterrupt(ADC_BASE, ADCStatus);

    if (ADCStatus)
    {
        ADCState = 0; //Not busy anymore
        ADCResult = ADC_getResults(ADC_BASE);
    }
}

void Init_RTC(void){
    //Initialize RTC with interval time 32768
    RTC_init(RTC_BASE,
             32768,
        RTC_CLOCKPREDIVIDER_1);

    RTC_clearInterrupt(RTC_BASE,
        RTC_OVERFLOW_INTERRUPT_FLAG);

    //Enable interrupt for RTC overflow
    RTC_enableInterrupt(RTC_BASE,
        RTC_OVERFLOW_INTERRUPT);

    //Start RTC Clock with clock source SMCLK
    RTC_start(RTC_BASE, RTC_CLOCKSOURCE_SMCLK);
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=RTC_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(RTC_VECTOR)))
#endif
void RTC_ISR (void)
{
    switch (__even_in_range(RTCIV,2)){
        case 0: break;  //No interrupts
        case 2:         //RTC overflow
            //one sec flag
            passed =1;
            break;
        default: break;
    }
}