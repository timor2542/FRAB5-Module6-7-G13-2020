#include "PID_Lib.h"

void Reset_PID()
{
  PID_Integrated = 0.0;
  PID_Prev_Input = 0.0;
  PID_First_Time = TRUE;
}

void Init_PID(float Kp, float Ki, float Kd, float MinOutput, float MaxOutput)
{
  PID_Kp         = Kp;
  PID_Ki         = Ki;
  PID_Kd         = Kd;
  PID_MinOutput  = MinOutput;
  PID_MaxOutput  = MaxOutput;
  PID_Integrated = 0.0;
  PID_Prev_Input = 0.0;
  PID_First_Time = TRUE;
}

float PID_Calculate(float SetPoint, float InputValue)
{
  float Err, ErrValue, DiffValue, Result;


  Err = SetPoint - InputValue;

  // --- calculate proportional value ---
  ErrValue  = Err * PID_Kp;

  // --- Calculate integrated value ---
  PID_Integrated = PID_Integrated + (Err * PID_Ki);
  // limit it to output minimum and maximum
  if (PID_Integrated < PID_MinOutput) 
    PID_Integrated = PID_MinOutput;
  if (PID_Integrated > PID_MaxOutput)
    PID_Integrated = PID_MaxOutput;

  // --- calculate derivative value ---
  if (PID_First_Time)
  {
    // to avoid a huge DiffValue the first time (PID_Prev_Input = 0)
    PID_First_Time = FALSE;
    PID_Prev_Input = InputValue;
  }
  DiffValue = (InputValue - PID_Prev_Input) * PID_Kd;
  PID_Prev_Input = InputValue;

  // --- calculate total ---
  Result = ErrValue + PID_Integrated - DiffValue; // mind the minus sign!!!
  // limit it to output minimum and maximum
  if (Result < PID_MinOutput) 
    Result = PID_MinOutput;
  else if (Result > PID_MaxOutput)
    Result = PID_MaxOutput;
  return (Result);
}