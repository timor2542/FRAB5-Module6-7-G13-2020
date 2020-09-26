/*
 * Program  :   Test PWM Motor Control
 * Description  :   
 * 
 * 
 */
#include "xc.h"
#include <stdio.h>

_FOSCSEL(FNOSC_FRC);
_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF);

#define FCY 40000000

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
void run_motor(char ch, char dir, char pow)
{
    if(ch == 1)
    {
        if(dir == 'F'){
            LATBbits.LATB0 = 0;
            LATBbits.LATB1 = 0;
        }
        else if(dir == 'B'){
            LATBbits.LATB0 = 0;
            LATBbits.LATB1 = 0;
        }
    }
    else if(ch == 2)
    {
        
    }
}
void stop_motor(char channel){
    switch(channel)
    {
        case 1:
        {
            LATBbits.LATB0 = 0;
            LATBbits.LATB1 = 0;
            break;
        }
        case 2:
        {
            LATBbits.LATB2 = 0;
            LATBbits.LATB3 = 0;
            break;
        }
        case 12:
        {
            LATBbits.LATB0 = 0;
            LATBbits.LATB1 = 0;
            LATBbits.LATB2 = 0;
            LATBbits.LATB3 = 0;
            break;
        }
        default:
        {
            LATBbits.LATB0 = 0;
            LATBbits.LATB1 = 0;
            LATBbits.LATB2 = 0;
            LATBbits.LATB3 = 0;
            break;
        } 
    }
}

int main(void) {
    
    /* Set Port */
    AD1PCFGL |= 0xFFFF; // Set all port is digital.
    TRISA |= 0x001F; // Set all port A is input.
    TRISB &= 0xFFFD; // Set RB1 is output, other of port B is input.
    
    /*disable global interrupt*/
    __builtin_disable_interrupts();
    initPLL();
    while(1);
    return 0;
}