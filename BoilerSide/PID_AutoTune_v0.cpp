
//#include <stdlib.h>
#include "PID_AutoTune_v0.h"
//#include <math.h>
//#include <FreeRTOS.h>
//#include <task.h>

#if ARDUINO >= 100
#include "Arduino.h"
#else
extern "C" {
#include "WConstants.h"
}
#endif

PID_ATune::PID_ATune()
{
    m_input = 0;
    m_output = 0;
    m_controlType = E_PID_ControlType_PID; //default to PID
    m_noiseBand = 0.5;
    m_runningState = E_AutoTuneState_Idle;
    m_oStep = 30;
    SetLookbackSec(10);
    m_lastTime = millis();
}

PID_ATune::PID_ATune(E_PID_ControlType controlType, int lookback, float setpoint, float overshoot, float powerStart,
        float powerStep)
{
    m_input = 0;
    m_output = 0;
    m_controlType = controlType; //default to PID
    if (overshoot == 0)
        m_noiseBand = 0.5;
    else
        m_noiseBand = overshoot;
    m_runningState = E_AutoTuneState_Idle;
    m_oStep = powerStep;
    m_outputStart = powerStart;
    m_setpoint = setpoint;
    SetLookbackSec(lookback);
    m_lastTime = millis();
}

void PID_ATune::Cancel()
{
    m_runningState = E_AutoTuneState_Idle;
}

float PID_ATune::Compute(float input)
{
    //justevaled = false;
    if (m_peakCount > 9 && m_runningState == E_AutoTuneState_Running)
    {
        m_runningState = E_AutoTuneState_Idle;
        FinishUp();
        return m_output;
    }
    unsigned long now = millis();

    float refVal = input;
    //justevaled = true;
    switch (m_runningState)
    {
    // if we're in Idle we need to reach the designated set point:
    case E_AutoTuneState_Idle:
        m_runningState = E_AutoTuneState_Setpoint;
    case E_AutoTuneState_Setpoint:
        if (refVal > m_setpoint + m_noiseBand)
            m_output = m_outputStart - m_oStep;
        else if (refVal < m_setpoint - m_noiseBand)
            m_output = m_outputStart + m_oStep;
        else // we're inside the noise band, so we start the autotune.
        {
            m_runningState = E_AutoTuneState_Running;
            //initialize working variables the first time around
            m_peakType = 0;
            m_peakCount = 0;
            m_justchanged = false;
            m_absMax = refVal;
            m_absMin = refVal;
            //m_setpoint = refVal;
            //m_outputStart = m_output;
            m_output = m_outputStart + m_oStep;
        }
        break;
    case E_AutoTuneState_Running:
        // todo: Check if we need this check when we're going to run this under the periodic monitor.
        if ((now - m_lastTime) < m_sampleTime)
            return m_output;

        m_lastTime = now;

        if (refVal > m_absMax)
            m_absMax = refVal;
        if (refVal < m_absMin)
            m_absMin = refVal;

        //oscillate the output base on the input's relation to the setpoint
        if (refVal > m_setpoint + m_noiseBand)
            m_output = m_outputStart - m_oStep;
        else if (refVal < m_setpoint - m_noiseBand)
            m_output = m_outputStart + m_oStep;

        //bool isMax=true, isMin=true;
        m_isMax = true;
        m_isMin = true;

        //id peaks
        for (int i = m_nLookBack - 1; i >= 0; i--)
        {
            float val = m_lastInputs[i];
            if (m_isMax)
                m_isMax = refVal > val;
            if (m_isMin)
                m_isMin = refVal < val;
            m_lastInputs[i + 1] = m_lastInputs[i];
        }
        m_lastInputs[0] = refVal;
        if (m_nLookBack < 9)
        {  //we don't want to trust the maxes or mins until the inputs array has been filled
            return m_output;
        }

        if (m_isMax)
        {
            if (m_peakType == 0)
                m_peakType = 1;
            if (m_peakType == -1)
            {
                m_peakType = 1;
                m_justchanged = true;
                m_peak2 = m_peak1;
            }
            m_peak1 = now;
            m_peaks[m_peakCount] = refVal;

        }
        else if (m_isMin)
        {
            if (m_peakType == 0)
                m_peakType = -1;
            if (m_peakType == 1)
            {
                m_peakType = -1;
                m_peakCount++;
                m_justchanged = true;
            }

            if (m_peakCount < 10)
                m_peaks[m_peakCount] = refVal;
        }

        if (m_justchanged && m_peakCount > 2)
        { //we've transitioned.  check if we can autotune based on the last peaks
            float avgSeparation = (abs(m_peaks[m_peakCount - 1] - m_peaks[m_peakCount - 2])
                    + abs(m_peaks[m_peakCount - 2] - m_peaks[m_peakCount - 3])) / 2;
            if (avgSeparation < 0.05 * (m_absMax - m_absMin))
            {
                FinishUp();
                m_runningState = E_AutoTuneState_Idle;
                return m_output;

            }
        }
        m_justchanged = false;
        break;
    }
    return m_output;
}

void PID_ATune::FinishUp()
{
    m_output = m_outputStart;
    //we can generate tuning parameters!
    m_Ku = 4 * (2 * m_oStep) / ((m_absMax - m_absMin) * 3.14159); // Kc
    m_Pu = (float) (m_peak1 - m_peak2) / 1000.0; //Tc
}

#define M_WEIGHT_P 0.6
#define M_WEIGHT_I 1.2
#define M_WEIGHT_D 0.075

float PID_ATune::GetKp()
{
    return m_controlType == E_PID_ControlType_PID ? M_WEIGHT_P * m_Ku : 0.4 * m_Ku;
}

float PID_ATune::GetKi()
{
    return m_controlType == E_PID_ControlType_PID ? M_WEIGHT_I * (m_Ku / m_Pu) : 0.48 * m_Ku / m_Pu;  // Ki = Kc/Ti
}

float PID_ATune::GetKd()
{
    return m_controlType == E_PID_ControlType_PID ? M_WEIGHT_D * (m_Pu * m_Ku) : 0;  //Kd = Kc * Td
}

void PID_ATune::SetOutputStep(float Step)
{
    m_oStep = Step;
}

float PID_ATune::GetOutputStep()
{
    return m_oStep;
}

void PID_ATune::SetControlType(E_PID_ControlType type) //0=PI, 1=PID
{
    m_controlType = type;
}

E_PID_ControlType PID_ATune::GetControlType()
{
    return m_controlType;
}

void PID_ATune::SetNoiseBand(float Band)
{
    m_noiseBand = Band;
}

float PID_ATune::GetNoiseBand()
{
    return m_noiseBand;
}

void PID_ATune::SetLookbackSec(int value)
{
    if (value < 1)
        value = 1;

    if (value < 25)
    {
        m_nLookBack = value * 4;
        m_sampleTime = 1;
    }
    else
    {
        m_nLookBack = (value > 100) ? 100 : value;
        m_sampleTime = value * 10;
    }
}

int PID_ATune::GetLookbackSec()
{
    return m_nLookBack * m_sampleTime / 1000;
}

