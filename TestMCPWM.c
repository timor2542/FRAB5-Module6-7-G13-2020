#include "xc.h"
#include <stdio.h>

void initPWM()
{
    TRISBbits.TRISB12=0;
    TRISBbits.TRISB14=0;
    // RB14 and Rb12
    PORTB = 0x00; // clear the outputs 
 
    PTCONbits.PTOPS = 1; // PWM timer post-scale
    PTCONbits.PTCKPS = 0; // PWM timer pre-scale
    PTCONbits.PTMOD = 2; // PWM operates in Up-down Mode continuously 
    //--> interupt each time wwe get to zero 
    PTMR = 0; // PWM counter value, start at 0
    PTPER = 799; // PWM Timebase period
    
    PWMCON1bits.PMOD2 = 0; // PWM in complimentary mode
    PWMCON1bits.PMOD1 = 0; // PWM in complimentary mode
    PWMCON1bits.PEN2H = 1; // PWM High pin is enabled
    PWMCON1bits.PEN1H = 1; // PWM High pin is enabled
	
    DTCON1bits.DTAPS = 0;  //DeadTime pre-scaler = Tcy
    DTCON1bits.DTA = 0;   //DeadTime value for 4 us. 

    PDC1 = p1; // PWM#1 Duty Cycle register (11-bit)
    PDC2 = p2; // PWM#2 Duty Cycle register (11-bit)

    PTCONbits.PTEN = 1; // Enable PWM Timerbase!
}
int adc_value(int channel){
    AD1CHS0 = channel;
    AD1CON1bits.SAMP = 1; // Start sampling
    int i;
    for(i = 0; i < 15; i++); // Wait 5 * TAD
    AD1CON1bits.SAMP = 0; // Start converting
    while(!AD1CON1bits.DONE); // Conversion done?
    AD1CON1bits.DONE = 0; // Clear conversion done status bit
    return ADC1BUF0;
    
}

void initPLL() // Set Fcy to 10 MHz
{
    PLLFBD = 96;           // M  = 98
    CLKDIVbits.PLLPRE = 7;  // N1 = 9
    CLKDIVbits.PLLPOST = 2; // N2 = 4
    OSCTUN = 0;             // Tune FRC oscillator, if FRC is used
    
    // Clock switching to incorporate PLL
    __builtin_write_OSCCONH(0x01);    // Initiate Clock Switch to FRCPLL
    // Oscillator with PLL (NOSC=0b011)
    __builtin_write_OSCCONL(0x01);    // Start clock switching

    while (OSCCONbits.COSC != 0b001); // Wait for Clock switch to occur
    while (OSCCONbits.LOCK!=1) {};    // Wait for PLL to lock
}


int main(void) {
    
    /* Set Port */
    AD1PCFGL |= 0xFFFF; // Set all port is digital.
    TRISA |= 0x001F; // Set all port A is input.
    TRISB &= 0xFFFD; // Set RB1 is output, other of port B is input.
    
    /*disable global interrupt*/
    __builtin_disable_interrupts();
    initPLL();
    
    /* Timer 2 is Analog Reading Time */
    T2CONbits.TCKPS = 0b11; // Set prescaler 1:256
    PR2 = 7813; // Set period at 200ms

    /* Timer 3 is LED toggle Time */
    T3CONbits.TCKPS = 0b11; // Set prescaler 1:256
    PR3 = 39063; // Set period at 1000ms
    OC2RS = 19532; // Set period at 500ms
    OC2CONbits.OCM = 0b000; // Disable Output Compare Module
    OC2CONbits.OCTSEL = 1;  // OC2 use timer3 as counter source
    OC2CONbits.OCM = 0b110; //set to PWM without fault pin mode
    
    __builtin_write_OSCCONL(OSCCON & 0xBF); //PPS RECONFIG UNLOCK 
    _RP1R = 0b10011;                 //remap RP2 connect to OC2
    __builtin_write_OSCCONL(OSCCON | 0x40); //PPS RECONFIG LOCK
    
    /* ADC Reading */
    
    AD1CON1bits.AD12B = 0;
    AD1CON3bits.ADCS = 2;
    AD1CON1bits.ADON = 1;
    int i;
    for(i = 0; i < 800; i++); // wait ADC module stabilize 20us
    
    /*enable global interrupt*/
    __builtin_enable_interrupts();
    
    /* Turn on Timer2&3 */
    T2CONbits.TON = 1;
    T3CONbits.TON = 1;
    
    /* Notice __dataADC */
    int __dataADC = 0;
    while(1)
    {
        if(TMR2 > 7500) // Spare time of 200ms
        {
            __dataADC = adc_value(2);
        }
        printf("TMR2= %5u, TMR3= %5u, AN2= %5d, RB1= %1d\n", TMR2, TMR3, __dataADC, _RB1);
    }
    
    return 0;
}