/*
 * Program      :   Test PWM Motor Control
 * Description  :   Generate PWM signal drive DC motor by IC L298N
 * Frequency    :   7.378 MHz at PLL
 * Filename     :   TestMCPWM.c
 * C Compiler   :   XC16 Complier by Microship Technology
 * Author       :   Mr.Krittamet Thawong and Mr.Wisarut Kasemphumirat KMUTT FIBO FRAB#5
 */
#include "xc.h"
#include <stdio.h>

#define GLOBAL_INT_DISABLE __builtin_disable_interrupts()
#define GLOBAL_INT_ENABLE __builtin_enable_interrupts()

#define FCY 40000000UL
#define MOTOR_PWM_FREQ 500        //motor PWM frequency in Hz
#define MOTOR_DUTY 70           //motor duty in %

/*void __attribute__((interrupt, no_auto_psv)) _PWM1Interrupt(void){
    IFS3bits.PWM1IF = 0;
}*/

void delay(unsigned int ms) // Keep counter for loop
{
    unsigned int x,a;
    for(x=0;x<ms;x++)
    {
        for(a=0;a<816;a++); // Loop for delay 1 millisecond per unit
    }
}

void __setIO(int __byte1,int __byte2,int __byte3){
    AD1PCFGL = __byte1; // Set all port is digital.
    TRISA = __byte2; // Set all port A is input.
    TRISB = __byte3; // Set all port B is output.
}
void initPWM()
{
     //setup PWM ports
    
     PWM1CON1 = 0;                  //  Clear all bits (use defaults)
     PWM1CON1bits.PMOD1 = 0;        //  PWM1Ly,PWM1Hy are in independent running mode
     PWM1CON1bits.PEN1L = 0;        //  PWM1L1 NORMAL I/O
     PWM1CON1bits.PEN1H = 1;        //  PWM1H1 PWM OUTPUT
     PWM1CON1bits.PEN2L = 0;        //  PWM1L1 NORMAL I/O
     PWM1CON1bits.PEN2H = 1;        //  PWM1H2 PWM OUTPUT
     
 
     //PWM mode and prescaler
     //PWM1, MOTORS 0,1,2    
     P1TCON = 0;                    //  Clear all bits (use defaults)
     P1TCONbits.PTMOD = 0b00;       //  Free-runing mode 
     P1TCONbits.PTCKPS = 0b11;      //  1:64 prescaler
 
     //Setup desired frequency by setting period for 1:64 prescaler
     P1TPER = (FCY  / 64 / MOTOR_PWM_FREQ) - 1;    

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
void initADC()
{
     /* ADC Reading */
    AD1CON1bits.AD12B = 0;
    AD1CON3bits.ADCS = 2;
    AD1CON1bits.ADON = 1;
    int i;
    for(i = 0; i < 800; i++); // wait ADC module stabilize 20us
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
            LATBbits.LATB0 = 1;
            LATBbits.LATB1 = 0;
        }
        else if(dir == 'B'){
            LATBbits.LATB0 = 0;
            LATBbits.LATB1 = 1;
        }
            //update duty cycle 
    P1DC1 = (2UL*P1TPER+2)*pow/100;  
    }
    else if(ch == 2)
    {
        if(dir == 'F'){
            LATBbits.LATB2 = 1;
            LATBbits.LATB3 = 0;
        }
        else if(dir == 'B'){
            LATBbits.LATB2 = 0;
            LATBbits.LATB3 = 1;
        }
    P1DC2 = (2UL*P1TPER+2)*pow/100;
    }
    else if(ch == 12)
    {
        if(dir == 'F'){
            LATBbits.LATB0 = 1;
            LATBbits.LATB1 = 0;
            LATBbits.LATB2 = 1;
            LATBbits.LATB3 = 0;
        }
        else if(dir == 'B'){
            LATBbits.LATB0 = 0;
            LATBbits.LATB1 = 1;
            LATBbits.LATB2 = 0;
            LATBbits.LATB3 = 1;
        }
     P1DC1 = (2UL*P1TPER+2)*pow/100;
     P1DC2 = (2UL*P1TPER+2)*pow/100;
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
    __setIO(0xFFFF, 0x001F, 0x0000);
    /*disable global interrupt*/
    GLOBAL_INT_DISABLE;
    initPLL();
    initPWM();
    initADC();
    /*enable global interrupt*/
    GLOBAL_INT_ENABLE;
    
    while(1)
    {
        run_motor(1,'F',MOTOR_DUTY);delay(2000);
        stop_motor(1);delay(2000);
        run_motor(1,'B',MOTOR_DUTY);delay(2000);
        stop_motor(1);delay(2000);
        run_motor(2,'F',MOTOR_DUTY);delay(2000);
        stop_motor(2);delay(2000);
        run_motor(2,'B',MOTOR_DUTY);delay(2000);
        stop_motor(2);delay(2000);
        run_motor(12,'F',MOTOR_DUTY);delay(2000);
        stop_motor(12);delay(2000);
        run_motor(12,'B',MOTOR_DUTY);delay(2000);
        stop_motor(12);delay(2000);
    }
    
    return 0;
}