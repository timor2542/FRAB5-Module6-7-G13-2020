/*
 * Program      :   Module 6-7 Milestone 3
 * Description  :   Generate PWM signal drive DC motor by Cytron MDD10A Rev. 2.0
 * Frequency    :   8 MHz at PLL
 * Filename     :   FinalizeProjectG13.c
 * C Compiler   :   XC16 Complier by Microship Technology
 * Author       :   Mr.Krittamet Thawong and Mr.Wisaruth Kasemphumirat KMUTT FIBO FRAB#5
 * Update       :   7 November 2020 10:08 AM UTC+7 Bangkok
 * Test Port
 */
#include "math.h" 
#include "xc.h"                                                 // Include XC16 Complier
#include <stdio.h>                                              // Standard I/O Library
#include "configuration.h"                                      // Bit Configuration
#include "UART_rev3.h"
#include "FRAB5G13Define.h"
#include <libpic30.h>
long map(long x, long in_min, long in_max, long out_min, long out_max) 
{
 return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
long high_byte(long num)
{
    return (num << 8) & 0xFF00;
}
long low_byte(long num)
{
    return num & 0xFF;
}
void Reset_PIDX()
{
    PID_IntegratedX = 0.0;
    PID_Prev_InputX = 0.0;
    PID_First_TimeX = TRUE;
}
void Reset_PIDZ()
{
    PID_IntegratedZ = 0.0;
    PID_Prev_InputZ = 0.0;
    PID_First_TimeZ = TRUE;
}
void Init_PIDX(double KpX, double KiX, double KdX, double MinOutputX, double MaxOutputX)
{
    PID_KpX         = KpX;
    PID_KiX         = KiX;
    PID_KdX         = KdX;
    PID_MinOutputX  = MinOutputX;
    PID_MaxOutputX  = MaxOutputX;
    PID_IntegratedX = 0.0;
    PID_Prev_InputX = 0.0;
    PID_First_TimeX = TRUE;
}
void Init_PIDZ(double KpZ, double KiZ, double KdZ, double MinOutputZ, double MaxOutputZ)
{
    PID_KpZ         = KpZ;
    PID_KiZ         = KiZ;
    PID_KdZ         = KdZ;
    PID_MinOutputZ  = MinOutputZ;
    PID_MaxOutputZ  = MaxOutputZ;
    PID_IntegratedZ = 0.0;
    PID_Prev_InputZ = 0.0;
    PID_First_TimeZ = TRUE;
}
double PID_CalculateX(double SetPointX, double InputValueX)
{
    double ErrX, ErrValueX, DiffValueX, ResultX;
//    char dir = 0;

    ErrX = SetPointX - InputValueX;
    
    ErrValueX  = ErrX * PID_KpX;

    // --- Calculate integrated value ---
    PID_IntegratedX = PID_IntegratedX + (ErrX * PID_KiX);
    // limit it to output minimum and maximum
    if (PID_IntegratedX < PID_MinOutputX) 
      PID_IntegratedX = PID_MinOutputX;
    if (PID_IntegratedX > PID_MaxOutputX)
      PID_IntegratedX = PID_MaxOutputX;

    // --- calculate derivative value ---
    if (PID_First_TimeX)
    {
      // to avoid a huge DiffValue the first time (PID_Prev_Input = 0)
      PID_First_TimeX = FALSE;
      PID_Prev_InputX = InputValueX;
    }
    DiffValueX = (InputValueX - PID_Prev_InputX) * PID_KdX;
    PID_Prev_InputX = InputValueX;

    // --- calculate total ---
    ResultX = ErrValueX + PID_IntegratedX + DiffValueX; // mind the minus sign!!!
    // limit it to output minimum and maximum
    if (ResultX < PID_MinOutputX)
    {
        ResultX = PID_MinOutputX;
    }
    else if (ResultX > PID_MaxOutputX)
    {
        ResultX = PID_MaxOutputX;
    }
    
    if(ResultX < 0)
    {
        DIRX = 1;
    }
    else if (ResultX >= 0)
    {
        DIRX = 0;
    }
    
    if(ResultX > -SET_MIN_SPEEDX && ResultX < 0)
    {
        ResultX = -SET_MIN_SPEEDX;
    }
    else if(ResultX > 0 && ResultX < SET_MIN_SPEEDX)
    {
        ResultX = SET_MIN_SPEEDX;
    }
    
    return (abs(ResultX));
}
double PID_CalculateZ(double SetPointZ, double InputValueZ)
{
    double ErrZ, ErrValueZ, DiffValueZ, ResultZ;
//    char dir = 0;

    ErrZ = SetPointZ - InputValueZ;
    
    ErrValueZ  = ErrZ * PID_KpZ;

    // --- Calculate integrated value ---
    PID_IntegratedZ = PID_IntegratedZ + (ErrZ * PID_KiZ);
    // limit it to output minimum and maximum
    if (PID_IntegratedZ < PID_MinOutputZ) 
      PID_IntegratedZ = PID_MinOutputZ;
    if (PID_IntegratedZ > PID_MaxOutputZ)
      PID_IntegratedZ = PID_MaxOutputZ;

    // --- calculate derivative value ---
    if (PID_First_TimeZ)
    {
      // to avoid a huge DiffValue the first time (PID_Prev_Input = 0)
      PID_First_TimeZ = FALSE;
      PID_Prev_InputZ = InputValueZ;
    }
    DiffValueZ = (InputValueZ - PID_Prev_InputZ) * PID_KdZ;
    PID_Prev_InputZ = InputValueZ;

    // --- calculate total ---
    ResultZ = ErrValueZ + PID_IntegratedZ + DiffValueZ; // mind the minus sign!!!
    // limit it to output minimum and maximum
    if (ResultZ < PID_MinOutputZ)
    {
        ResultZ = PID_MinOutputZ;
    }
    else if (ResultZ > PID_MaxOutputZ)
    {
        ResultZ = PID_MaxOutputZ;
    }
    
    if(ResultZ < 0)
    {
        DIRZ = 1;
    }
    else if (ResultZ >= 0)
    {
        DIRZ = 0;
    }
    
    if(ResultZ > -SET_MIN_SPEEDZ && ResultZ < 0)
    {
        ResultZ = -SET_MIN_SPEEDZ;
    }
    else if(ResultZ > 0 && ResultZ < SET_MIN_SPEEDZ)
    {
        ResultZ = SET_MIN_SPEEDZ;
    }
    
    return (abs(ResultZ));
}
void setADIO(unsigned int __byte1,unsigned int __byte2,unsigned int __byte3)// Set
{
    AD1PCFGL = __byte1;
    TRISA = __byte2;
    TRISB = __byte3;
}
/*void XINT(char __selectINT, char __selectRP) // External Mapping Function
{
    OSCCON = 0x46;
    OSCCON = 0x57;
    OSCCONbits.IOLOCK = 0;
    
    if(__selectINT == 0x01)
    {
        RPINR0bits.INT1R = __selectRP;
    }
    else if(__selectINT == 0x02)
    {
        RPINR1bits.INT2R = __selectRP;
    }
    
    OSCCON = 0x46;
    OSCCON = 0x57;
    OSCCONbits.IOLOCK = 1;
}*/
void initPWM()
{
     //setup PWM ports
     P1TCONbits.PTEN = 0;
     PWM1CON1 = 0;                                          //  Clear all bits (use defaults)
     
     P2TCONbits.PTEN = 0;
     PWM2CON1 = 0; 
     
     PWM1CON1bits.PMOD1 = 1;                                //  PWM1L1,PWM1H1 are in independent running mode
     PWM1CON1bits.PMOD2 = 1;                                //  PWM1L2,PWM1H2 are in independent running mode
     PWM1CON1bits.PMOD3 = 1;                                //  PWM1L3,PWM1H3 are in independent running mode
     PWM2CON1bits.PMOD1 = 1;                                //  PWM2L1,PWM2H1 are in independent running mode
     
     PWM1CON1bits.PEN1L = 0;                                //  PWM1L1 NORMAL I/O
     PWM1CON1bits.PEN1H = 1;                                //  PWM1H1 PWM OUTPUT
     PWM1CON1bits.PEN2L = 0;                                //  PWM1L2 NORMAL I/O
     PWM1CON1bits.PEN2H = 1;                                //  PWM1H2 PWM OUTPUT
     PWM1CON1bits.PEN3L = 0;                                //  PWM1L3 NORMAL I/O
     PWM1CON1bits.PEN3H = 0;                                //  PWM1H3 NORMAL I/O
     PWM2CON1bits.PEN1L = 0;                                //  PWM2L1 NORMAL I/O
     PWM2CON1bits.PEN1H = 0;                                //  PWM2H1 PWM OUTPUT

     //PWM1 Mode and Pre-scaler
     //PWM1H,PWM2H ON DC MOTOR 2-Channel and 1-Channel    
     P1TCON = 0;                    //  Clear all bits (use defaults)
     P1TCONbits.PTMOD = 0b00;       //  Free-runing mode 
     P1TCONbits.PTCKPS = 0b11;      //  1:64 prescaler
     P2TCON = 0;                    //  Clear all bits (use defaults)
     P2TCONbits.PTMOD = 0b00;       //  Free-runing mode 
     P2TCONbits.PTCKPS = 0b11;      //  1:64 prescaler
 
     //Setup desired frequency by setting period for 1:64 prescaler
     P1TPER = (FCY  / 64 / MOTOR_PWM_FREQ) - 1;
     P2TPER = (FCY  / 64 / MOTOR_PWM_FREQ) - 1;

     //ENABLE PWM1
     P1TMR = 0;
     P2TMR = 0; 
}
void PWM1Freq(unsigned int __MOTOR_PWM_FREQ)
{
    P1TCONbits.PTEN = 0;
    P1TPER = (FCY  / 64 / __MOTOR_PWM_FREQ) - 1;
    P1TMR = 0;
    //P1TCONbits.PTEN = 1;
}
void PWM2Freq(unsigned int __MOTOR_PWM_FREQ)
{
    P2TCONbits.PTEN = 0;
    P2TPER = (FCY  / 64 / __MOTOR_PWM_FREQ) - 1;
    P2TMR = 0;
    //P2TCONbits.PTEN = 1;
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
    for(i = 0; i < 800; i++); Nop(); // wait ADC module stabilize 20us
}
int adc_value(int channel)
{
    AD1CHS0 = channel;
    AD1CON1bits.SAMP = 1; // Start sampling
    int i;
    for(i = 0; i < 15; i++); // Wait 5 * TAD
    AD1CON1bits.SAMP = 0; // Start converting
    while(!AD1CON1bits.DONE); // Conversion done?
    AD1CON1bits.DONE = 0; // Clear conversion done status bit
    return ADC1BUF0;
}
void initQEI()
{
    __builtin_write_OSCCONL(OSCCON & 0xBF); // PPS RECONFIG UNLOCK 
    
    RPINR18bits.U1RXR = 6;                  // Set UART1 RX to MCU pin
    RPOR2bits.RP5R = 0b00011;               // Set UART1 TX from MCU pin
    RPINR14bits.QEA1R = 3;          // Remap __chQEI1A connect to QEI1_A
    RPINR14bits.QEB1R = 2;          // Remap __chQEI1B connect to QEI1_B
    RPINR16bits.QEA2R = 1;          // Remap __chQEI2A connect to QEI2_A
    RPINR16bits.QEB2R = 0;          // Remap __chQEI2B connect to QEI2_B
    
    __builtin_write_OSCCONL(OSCCON | 0x40); //  PPS RECONFIG LOCK 
    
    /*QEI1 Mode Select*/
    QEI1CONbits.QEIM = 0b000;           // QEI1 Mode disable
    QEI1CONbits.PCDOUT = 0;             // no direction pin out
    QEI1CONbits.QEIM = 0b111;           // 4x ,no index
    
    /*QEI2 Mode Select*/
    QEI2CONbits.QEIM = 0b000;           // QEI2 Mode disable
    QEI2CONbits.PCDOUT = 0;             // no direction pin out
    QEI2CONbits.QEIM = 0b111;           // 4x ,no index
    
    /*Digital Filter QEI1 Config */
    DFLT1CONbits.QECK = 0b111;          // clock divider Fcy/256
    DFLT1CONbits.QEOUT = 1;             // enable filter
    
    /*Digital Filter QEI2 Config */
    DFLT2CONbits.QECK = 0b111;          // clock divider Fcy/256
    DFLT2CONbits.QEOUT = 1;             // enable filter
    
}
void PWM1(char ch, char dir, char POW)
{
    if(ch == 0x01)
    {
        if(dir == 0x00)
        {
            _LATA2 = 0;
        }
        else if(dir == 0x01)
        {
            _LATA2 = 1;
        }
        P1DC1 = (2UL+2*P1TPER)*POW/100;
    }
    else if(ch == 0x02)
    {
        if(dir == 0x00)
        {
            _LATA3 = 0;
        }
        else if(dir == 0x01)
        {
            _LATA3 = 1;
        }
        P1DC2 = (2UL+2*P1TPER)*POW/100;
    }
    else if(ch == 0x66)
    {
        if(dir == 0x00)
        {
            _LATA2 = 0;
            _LATA3 = 0;

        }
        else if(dir == 0x01)
        {
            _LATA2 = 1;
            _LATA3 = 1;
        }
        P1DC1 = (2UL+2*P1TPER)*POW/100;
        P1DC2 = (2UL+2*P1TPER)*POW/100;
    }
}
void PWM2(char dir, char POW)
{
    if(dir == 0x00)
    {
        _LATB13 = 0;
    }
    else if(dir == 0x01)
    {
        _LATB13 = 1;
    }
    P2DC1 = (2UL+2*P2TPER)*POW/100;
}
void PWM1Stop(char ch)
{
    if(ch == 0x01)
    {
        _LATA2 = 0;
        P1DC1 = 0;
    }
    else if(ch == 0x02)
    {
        _LATA3 = 0;
        P1DC2 = 0;
    }
    else if(ch == 0x66){
        _LATA2 = 0;
        _LATA3 = 0;
        P1DC1 = 0;
        P1DC2 = 0;
    }
}
void PWM2Stop()
{
    P2DC1 = 0;
}
void initTimer1() //Timer1
{
    TIMER1_OFF;
    TIMER1_PRESCALE(0b10); //set timer prescaler to 1:64
    PR1 = 6250;
}
void initTimer4() //Timer1
{
    TIMER4_OFF;
    TIMER4_PRESCALE(0b10); //set timer prescaler to 1:64
    //PR4 = 6250;
    PR4 = 625;
}
void initOCServo() // Timer23
{
    TIMER2_OFF;
    TIMER3_OFF;
    OC1CONbits.OCM = 0b000; //Disable Output Compare Module
    OC2CONbits.OCM = 0b000; //Disable Output Compare Module
    TIMER2_PRESCALE(0b10); //set timer prescaler to 1:64
    TIMER3_PRESCALE(0b10); //set timer prescaler to 1:64
    
    PR2 = 12500;            //set period to 12,500 tick per cycle (Frequency of Servo = 50 Hz)
    OC1CONbits.OCTSEL = 0;  //OC1 use timer2 as counter source
    
    PR3 = 12500;            //set period to 12,500 tick per cycle (Frequency of Servo = 50 Hz)
    OC2CONbits.OCTSEL = 1;  //OC2 use timer3 as counter source

    __builtin_write_OSCCONL(OSCCON & 0xBF); //PPS RECONFIG UNLOCK 
    _RP10R = 0b10010;                 //remap RP11 connect to OC1
    _RP9R = 0b10011;                 //remap RP11 connect to OC2
    __builtin_write_OSCCONL(OSCCON | 0x40); //PPS RECONFIG LOCK
    OC1CONbits.OCM = 0b110; //set to pwm without fault pin mode
    OC2CONbits.OCM = 0b110; //set to pwm without fault pin mode
    
}
void servo_angle(char ch, signed int angle) //set velocity
{
    if(ch == 0x01)
    {
         //set to pwm without fault pin mode
        if(angle < 0 && angle > 180)
        {
            OC1RS = 0;
        }
        else if(angle >= 0 && angle <= 180)
        {
            long value = map(angle,0,180,313,1562); // Angle 0-180 = OCxRS 313-1562
            OC1RS = value;
        }
    }
    else if(ch == 0x02)
    {
        //OC2CONbits.OCM = 0b110; //set to pwm without fault pin mode
        if(angle < 0 && angle > 180)
        {
            OC2RS = 0;
        }
        else if(angle >= 0 && angle <= 180)
        {
            long value = map(angle,0,180,313,1562); // Angle 0-180 = OCxRS 313-1562
            OC2RS = value;
        }
    }
    else if(ch == 0x66)
    {
        if(angle < 0 && angle > 180)
        {
            OC1RS = 0;
            OC2RS = 0;
        }
        else if(angle >= 0 && angle <= 180)
        {
            long value = map(angle,0,180,313,1562); // Angle 0-180 = OCxRS 313-1562
            OC1RS = value;
            OC2RS = value;
        }
    }
}
void servo_stop(char ch)
{
    switch(ch)
    {
        case 0x01:
        {
            OC1RS = 0;
        }
        case 0x02:
        {
            OC2RS = 0;
        }
        case 0x66:
        {
            OC1RS = 0;
            OC2RS = 0;
        }
    }
}
void updatePositionX(long _posx)
{
    set_position_x = _posx;
}
void updatePositionY(long _posy)
{
    set_position_y = _posy;
}
void updatePositionZ(long _posz)
{
    set_position_z = _posz;
}
void updatePositionXY(long _posx, long _posy)
{
    set_position_x = _posx;
    set_position_y = _posy;
}
void updatePositionXZ(long _posx, long _posz)
{
    set_position_x = _posx;
    set_position_z = _posz;
}
void updatePositionXZY(long _posx, long _posz, long _posy)
{
    set_position_x = _posx;
    set_position_z = _posz;
    set_position_y = _posy;
}
void updateAngleServoPickUp(int _angle)
{
    sv1_angle = _angle;
}
void updateAngleServoRotate(int _angle)
{
    sv2_angle = _angle;
}
void ResetPulse()
{
    POS1CNT = 0;
    POS2CNT = 0;
    step_value = 0;
}
void StepMotor(unsigned int __SetPoint) 
{
    //_LATB13 = dir; //Changes the rotations direction
    _LATB15 = 0;
    if(__SetPoint < 0)
    {
        __SetPoint = 0;
    }
    
    if(step_value < __SetPoint)
    {
        DIRZ = 0;
        _LATB13 = DIRZ;
    }
    else{
        DIRZ = 1;
        _LATB13 = DIRZ;
    }
    
    int x;
    // Makes N pulses for making one full cycle rotation
    for (x = 0; x < PULSE; x++) {

        _LATB8 = 1;
        __delay_us(DELAY_STEP_MOTOR);
        _LATB8 = 0;
        __delay_us(DELAY_STEP_MOTOR);
    }
    
    if(DIRZ == 0)
    {
        step_value++;
    }
    else
    {
        step_value--;
    }
}
void StepMotorStop()
{
    //_LATB15 = 1;
    _LATB8 = 0;
}
void __attribute__((interrupt, no_auto_psv)) _T4Interrupt(void)
{
    
    if(RunningMotor == TRUE || RunningServoPickUp == TRUE || RunningServoRotate == TRUE)
    {
        _LATA4 = 1;
    }
    else
    {
        _LATA4 = 0;
    }
    
    if(RunningMotor == TRUE)
    {
        if(set_position_x == abs(POS1CNT) && set_position_z == abs(POS2CNT) && set_position_y == step_value)
        {
            RunningMotor = FALSE;
            PWM1Stop(ALL2);
            StepMotorStop();
            /*if (step_value <= 0) {
                PWM2CON1bits.PEN1H = 1; //  PWM2H1 PWM OUTPUT
                PWM2_ENABLE;
                while (_RB4 == 0) {
                    PWM2(BACKWARD, 50);
                }
                PWM2Stop();
                PWM2_DISABLE;
                PWM2CON1bits.PEN1H = 0;
                _LATB15 = 1;
            } else {
                _LATB15 = 0;
            }*/
        }
        else if(set_position_x != abs(POS1CNT) && set_position_z == abs(POS2CNT) && set_position_y == step_value)
        {
            RunningMotor = TRUE;
            PWM1Stop(M2);
            StepMotorStop();
            PWM1(M1, DIRX, (int)PID_CalculateX(set_position_x,abs(POS1CNT)));
        }
        else if(set_position_x == abs(POS1CNT) && set_position_z != abs(POS2CNT) && set_position_y == step_value)
        {
            RunningMotor = TRUE;
            PWM1Stop(M1);
            StepMotorStop();
            PWM1(M2, DIRZ, (int)PID_CalculateZ(set_position_z,abs(POS2CNT)));
        }
        else if(set_position_x == abs(POS1CNT) && set_position_z == abs(POS2CNT) && set_position_y != step_value)
        {
            RunningMotor = TRUE;
            PWM1Stop(ALL2);
            StepMotor(set_position_y);
        }
        else if(set_position_x != abs(POS1CNT) && set_position_z != abs(POS2CNT) && set_position_y == step_value)
        {
            RunningMotor = TRUE;
            StepMotorStop();
            PWM1(M1, DIRX, (int)PID_CalculateX(set_position_x,abs(POS1CNT)));
            PWM1(M2, DIRZ, (int)PID_CalculateZ(set_position_z,abs(POS2CNT)));
        }
        else if(set_position_x != abs(POS1CNT) && set_position_z == abs(POS2CNT) && set_position_y != step_value)
        {
            RunningMotor = TRUE;
            PWM1Stop(M2);
            PWM1(M1, DIRX, (int)PID_CalculateX(set_position_x,abs(POS1CNT)));
            StepMotor(set_position_y);
        }
        else if(set_position_x == abs(POS1CNT) && set_position_z != abs(POS2CNT) && set_position_y != step_value)
        {
            RunningMotor = TRUE;
            PWM1Stop(M1);
            PWM1(M2, DIRZ, (int)PID_CalculateZ(set_position_z,abs(POS2CNT)));
            StepMotor(set_position_y);
        }
        else if(set_position_x != abs(POS1CNT) && set_position_z != abs(POS2CNT) && set_position_y != step_value)
        {
            RunningMotor = TRUE;
            PWM1(M1, DIRX, (int)PID_CalculateX(set_position_x,abs(POS1CNT)));
            PWM1(M2, DIRZ, (int)PID_CalculateZ(set_position_z,abs(POS2CNT)));
            StepMotor(set_position_y);
        }
    }  
    else
    {
        PWM1Stop(ALL2);
        PWM2Stop();
    }
    
    if(RunningServoPickUp == TRUE)
    {
        servo_angle(SERVO1, sv1_angle);
        RunningServoPickUp = FALSE;
    }
    
    if(RunningServoRotate == TRUE)
    {
        servo_angle(SERVO2, sv2_angle);
        RunningServoRotate = FALSE;
    }
    _T4IF = 0;
}
void Homing()
{
    PWM2CON1bits.PEN1H = 1;                                //  PWM2H1 PWM OUTPUT
    PWM2_ENABLE;
    while(_RB4 == 0)
    {
        PWM2(BACKWARD, 50);
    }
    PWM2Stop();
    PWM2_DISABLE;
    PWM2CON1bits.PEN1H = 0;
    __delay_ms(300);
    while(_RB7 == 0)
    {
        PWM1(M1, BACKWARD, 60);
    }
    PWM1Stop(M1);
    __delay_ms(300);
    while(_RB11 == 0)
    {
        PWM1(M2, BACKWARD, 60);
    }
    PWM1Stop(M2);
    ResetPulse();
    __delay_ms(1000);
}
int main(void) 
{
            // Set Port
    // Argument 1 Set all ports A/D I/O;    Analog = 0, Digital = 1
    // Argument 2 Set all ports A I/O       Output = 0, Input = 1
    // Argument 3 Set all ports B I/O       Output = 0, Input = 1
    setADIO(0xFFFC, 0x0003, 0x084F);
    
    // Disable Global Interrupt
    GLOBAL_INT_DISABLE;
    
    // Initialize
    initPLL();  // Initialize PLL at FCY = 40 MHz
    initTimer1();
    initTimer4(); // Initialize Timer4;
    initPWM();  // Initialize PWM
    initADC();  // Initialize ADC
    
    // External Interrupt Configuration
    //XINT(1,4);     // Remap External INT1 on RP4
    //XINT(2,10);     // Remap External INT2 on RP10
    initOCServo();

    // Enable External Interrupt
    //INT0_TRIG_EDGE(POSITIVE_EDGE);              // Set INT0 Positive Edge
    //INT1_TRIG_EDGE(POSITIVE_EDGE);              // Set INT1 Positive Edge
    //INT2_TRIG_EDGE(POSITIVE_EDGE);              // Set INT2 Positive Edge
    //INT0_PRIORITY(7);               // Set INT0 Interrupt Priority by 7
    //INT1_PRIORITY(6);               // Set INT1 Interrupt Priority by 6
    //INT2_PRIORITY(5);               // Set INT2 Interrupt Priority by 5

    //_U1RXIP = 2;
    // Update Frequency of PWM1 and PWM2
    PWM1Freq(500);
    PWM1_ENABLE;
    PWM2Freq(2000);
    
    TIMER2_ON;
    TIMER3_ON;
    // Enable Global Interrupt
    TIMER4INT_ENABLE;
    TIMER4_INT_PRIORITY(1);
    TIMER4_ON;
    
    _LATB15 = 0;
    Init_PIDX(1,0,0,-MOTOR_SPEEDX,MOTOR_SPEEDX); // Kp, Ki, Kd, MinDCPWM, MaxDCPWM
    Init_PIDZ(1,0,0,-MOTOR_SPEEDZ,MOTOR_SPEEDZ); // Kp, Ki, Kd, MinDCPWM, MaxDCPWM
    Homing();
    //_LATB15 = 1;
    initQEI(); // Initialize QEI1 and QEI2
    // UART1 Initialize Communication at Baud Rate 115200 bps
    UART1_Initialize(86, 347);
    GLOBAL_INT_ENABLE;
    while(1)
    {
        numByte = UART1_ReadBuffer(dataArray, NUMBYTE);
        __delay_ms(DELAY_WAIT_STABILITY);
        if (numByte > 0){
            
            int i;//,j;
            for(i = 0; i < NUMBYTE; i++)
            {
                buff_val[i] = 0;
            }
            printf("Received Data(HEX): ");
            for (i = 0; i < numByte; i++){
                printf("0x%2hX ", (unsigned char)dataArray[i]);
                buff_val[i] = dataArray[i];
            }
            printf("\n");
            
            if(buff_val[0] == 'F' && buff_val[1] == 'X' && buff_val[4] == 'Z' && buff_val[7] == 'Y' && buff_val[10] == 'P' && buff_val[12] == 'R' && buff_val[14] == 'S')
            {

                updatePositionX((int)(high_byte((unsigned char)buff_val[2])+low_byte((unsigned char)buff_val[3])));
                updatePositionZ((int)(high_byte((unsigned char)buff_val[5])+low_byte((unsigned char)buff_val[6])));
                updatePositionY((int)(high_byte((unsigned char)buff_val[8])+low_byte((unsigned char)buff_val[9])));
                updateAngleServoPickUp((int)(low_byte((unsigned char)buff_val[11])));
                updateAngleServoRotate((int)(low_byte((unsigned char)buff_val[13])));
                RunningMotor = TRUE;
                RunningServoPickUp = TRUE;
                RunningServoRotate = TRUE;
            }
            else if(buff_val[0] == 'F' && buff_val[1] == 'X' && buff_val[4] == 'Z' && buff_val[7] == 'Y' && buff_val[10] == 'S')
            {
                updatePositionX((int)(high_byte((unsigned char)buff_val[2])+low_byte((unsigned char)buff_val[3])));
                updatePositionZ((int)(high_byte((unsigned char)buff_val[5])+low_byte((unsigned char)buff_val[6])));
                updatePositionY((int)(high_byte((unsigned char)buff_val[8])+low_byte((unsigned char)buff_val[9])));
                RunningMotor = TRUE;
                RunningServoPickUp = FALSE;
                RunningServoRotate = FALSE;
            }
            else if(buff_val[0] == 'F' && buff_val[1] == 'X' && buff_val[4] == 'Z' && buff_val[7] == 'S')
            {
                updatePositionX((int)(high_byte((unsigned char)buff_val[2])+low_byte((unsigned char)buff_val[3])));
                updatePositionZ((int)(high_byte((unsigned char)buff_val[5])+low_byte((unsigned char)buff_val[6])));
                RunningMotor = TRUE;
                RunningServoPickUp = FALSE;
                RunningServoRotate = FALSE;
            }
            else if(buff_val[0] == 'F' && buff_val[1] == 'P' && buff_val[3] == 'R' && buff_val[5] == 'S')
            {
                updateAngleServoPickUp((int)(low_byte((unsigned char)buff_val[2])));
                updateAngleServoRotate((int)(low_byte((unsigned char)buff_val[4])));
                RunningMotor = FALSE;
                RunningServoPickUp = TRUE;
                RunningServoRotate = TRUE;
            }
            else if(buff_val[0] == 'F' && buff_val[1] == 'Y' && buff_val[4] == 'S')
            {
                updatePositionY((int)(high_byte((unsigned char)buff_val[2])+low_byte((unsigned char)buff_val[3])));
                RunningMotor = TRUE;
                RunningServoPickUp = FALSE;
                RunningServoRotate = FALSE;
            }
            else if(buff_val[0] == 'F' && buff_val[1] == 'P' && buff_val[3] == 'S')
            {
                updateAngleServoPickUp((int)(low_byte((unsigned char)buff_val[2])));
                RunningMotor = FALSE;
                RunningServoPickUp = TRUE;
                RunningServoRotate = FALSE;
            }
            else if(buff_val[0] == 'F' && buff_val[1] == 'R' && buff_val[3] == 'S')
            {
                updateAngleServoRotate((int)(low_byte((unsigned char)buff_val[2])));
                RunningMotor = FALSE;
                RunningServoPickUp = FALSE;
                RunningServoRotate = TRUE;
            }
            else if(buff_val[0] == 'F' && buff_val[1] == 'F' && buff_val[2] == 'S')
            {
                RunningMotor = FALSE;
                RunningServoPickUp = FALSE;
                RunningServoRotate = FALSE;
            }
            else
            {
                RunningMotor = FALSE;
                RunningServoPickUp = FALSE;
                RunningServoRotate = FALSE;
            }
        }
    }
    return 0;
}