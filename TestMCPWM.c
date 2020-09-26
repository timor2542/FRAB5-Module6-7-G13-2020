/*
 * Program  :   Test PWM Motor Control
 * Description  :   
 * 
 * 
 */
#include "xc.h"
#include <stdio.h>

#define MOTOR_PWM_FREQ 500        //motor PWM frequency in Hz
#define FCY 40000000UL            
#define MOTOR_DUTY 70           //motor duty in %

void initPWM()
{
         //setup PWM ports
     P1TCONbits.PTEN = 0; 
     
     PWM1CON1 = 0;                //clear all bits (use defaults)
     PWM1CON1bits.PMOD1 = 0;     //PWM1Ly,PWM1Hy are in independent running mode
     PWM1CON1bits.PEN1L = 0;     //PWM1L1 NORMAL I/O
     PWM1CON1bits.PEN1H = 1;     //PWM1H1 PWM OUTPUT
     PWM1CON1bits.PEN2L = 0;     //PWM1L1 NORMAL I/O
     PWM1CON1bits.PEN2H = 1;     //PWM1H2 PWM OUTPUT
     
 
     //PWM mode and prescaler
     //PWM1, MOTORS 0,1,2    
     P1TCON = 0;                    //clear all bits (use defaults)
     P1TCONbits.PTMOD = 0b00;    //Free-runing mode 
     P1TCONbits.PTCKPS = 0b11;    // 1:64 prescaler
 
     //setup desired frequency by setting period for 1:64 prescaler
     P1TPER = (FCY  / 64 / MOTOR_PWM_FREQ) - 1;    
 
     //update duty cycle 
     P1DC1 = (2UL*P1TPER+2)*MOTOR_DUTY/100;  
     
     //ENABLE PWM1
     P1TMR = 0;
     P1TCONbits.PTEN = 1; 
}

void initPLL() // Set Fcy to 40 MHz
{
    PLLFBD = 150;           // M  = 152
    CLKDIVbits.PLLPRE = 5;  // N1 = 7
    CLKDIVbits.PLLPOST = 0; // N2 = 2
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