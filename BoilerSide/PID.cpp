#include "PID.h"

#if ARDUINO >= 100
#include "Arduino.h"
#else
extern "C" {
#include "WConstants.h"
}
#endif

PIDDynamicSampleTime::PIDDynamicSampleTime()
{
  m_input = 0;
  m_output = 0;
  m_lastOutput = 0;
  m_setpoint = 0;
  m_kiErrorThreshold = 0;
  m_outputSmoothingCoeff = 1/((float)(10));
  m_setpointRange = 0;

  m_inAuto = false;
  m_enabled = false;
  m_advancedPid = true;
  m_kBeta = 1;
  m_kLinear = 1;
  m_kTrapezoidalRange = 0;
  m_feedForwardInjection = 0;

  SetOutputLimits(0, 100);
  setControllerDirection(true);
  SetTunings(1, 0, 0, 0);
  setSetPointRange(100);
}

PIDDynamicSampleTime::~PIDDynamicSampleTime()
{
}

float PIDDynamicSampleTime::Compute(float input)
{
  if (!m_enabled)
    return 0;

  float calculatedSetpoint;
  float sampleTimeModifier;
  float dtermSampleTimeModifier;
  float modifiedFFiterm;

  m_input = input;

  if (!m_inAuto)
  {
    m_lastTick = millis();
    m_lastInput = input;
    m_output = m_feedForwardInjection;

    if (m_output > m_outMax)
      m_output = m_outMax;
    else if (m_output < m_outMin)
      m_output = m_outMin;

    return m_output;
  }
  unsigned long currentTick = millis();
  // need to scale the tuning parameters according to the sample time.
  sampleTimeModifier = (currentTick - m_lastTick + 1) * 0.001;

  calculatedSetpoint = m_setpoint;

  /*Compute all the working error variables*/
  m_error = calculatedSetpoint - m_input;
  //#define M_SP_RANGE 500
  //#define M_ITERM_RANGE 30
  //#define M_ITERM_RANGE_SQR M_ITERM_RANGE*M_ITERM_RANGE
  m_dError = (m_error - m_lastError);
  m_dInput = (m_input - m_lastInput);

  if (m_advancedPid)
  {
    m_advError = m_error * (m_kLinear + (1 - m_kLinear) * abs(m_error) / m_setpointRange);
    m_advBError = m_kBeta * m_setpoint - m_input;
    m_advBError = m_advBError * (m_kLinear + (1 - m_kLinear) * abs(m_advBError) / m_setpointRange);

    if ((m_error * m_error) > m_kTrapezoidalRange)
    {
      m_ITerm = 0;
      modifiedFFiterm = 0;
    }
    else
    {
      m_ITerm += m_ki * ((m_error * sampleTimeModifier) / (1 + (10 * m_error * m_error) / m_kTrapezoidalRange));

      modifiedFFiterm = m_ITerm + m_feedForwardInjection;

      if (modifiedFFiterm > m_outMax)
        modifiedFFiterm = m_outMax;
      else if (modifiedFFiterm < m_outMin)
        modifiedFFiterm = m_outMin;
    }
  }
  else
  {
    m_ITerm += (m_ki * m_error * sampleTimeModifier);
    if (m_ITerm > m_outMax)
      m_ITerm = m_outMax;
    else if (m_ITerm < m_outMin)
      m_ITerm = m_outMin;
  }

  /*Compute PIDDynamicSampleTime Output*/
  //    m_output = m_kp * m_error + m_ITerm + m_kd * m_dError / sampleTimeModifier;
  //    if (m_input != m_lastInput)
  //    {
  //        dtermSampleTimeModifier = (currentTick - m_lastDTermTick + 1) * 0.001;
  //        m_DTerm = m_kd * m_dInput / dtermSampleTimeModifier;
  //        m_lastDTermTick = currentTick;
  //    }
  if (m_advancedPid)
  {
    m_PTerm = m_kp * m_advBError;
    m_DTerm = m_kd * m_dInput / sampleTimeModifier;
    m_output = m_kp * m_advBError + modifiedFFiterm - m_DTerm;
  }
  else
  {
    m_PTerm = m_kp * m_error;
    m_DTerm = m_kd * m_dInput / sampleTimeModifier;
    m_output = m_kp * m_error + modifiedFFiterm - m_DTerm;
  }

  // truncate output value:
  if (m_output > m_outMax)
    m_output = m_outMax;
  else if (m_output < m_outMin)
    m_output = m_outMin;

  // compute output smoothing:
  m_output = m_outputSmoothingCoeff * m_output + (1 - m_outputSmoothingCoeff) * m_lastOutput;

  //    if (CLogger::getInstance().checkLevel(M_LOGGER_LEVEL_VERBOSE))
  //    {
  //        printf("PSSID=%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", m_pssId, currentTick - m_lastTick,
  //                m_kp, m_ki, m_kd, m_kTrapezoidalRange, m_setpoint, m_input, m_output, m_PTerm, m_ITerm, m_DTerm);
  //    }

  /*Remember some variables for next time*/
  m_lastTick = currentTick;
  m_lastInput = m_input;
  m_lastOutput = m_output;
  m_lastError = m_error;

  return m_output;
}

void PIDDynamicSampleTime::SetTunings(float Kp, float Ki, float Kd, float kiRange)
{
  if (Kp < 0)
    m_controllerDirection = REVERSE;
  else
    m_controllerDirection = DIRECT;

  m_kp = Kp;
  m_ki = abs(Ki);
  m_kd = abs(Kd);
  m_kiErrorThreshold = kiRange;

  updateTrapezoidalRange();

  if (m_controllerDirection == REVERSE)
  {
    m_ki = (0 - m_ki);
    m_kd = (0 - m_kd);
  }

  //    setKiErrorThreshold(m_outMax/m_kp);

  //    if (m_controllerDirection == REVERSE)
  //    {
  //        m_kp = (0 - m_kp);
  //        m_ki = (0 - m_ki);
  //        m_kd = (0 - m_kd);
  //    }

  Initialize();
}

void PIDDynamicSampleTime::SetOutputLimits(float Min, float Max)
{
  if (Min > Max)
    return;
  m_outMin = Min;
  m_outMax = Max;
  updateTrapezoidalRange();

  if (m_output > m_outMax)
    m_output = m_outMax;
  else if (m_output < m_outMin)
    m_output = m_outMin;

  if (m_ITerm > m_outMax)
    m_ITerm = m_outMax;
  else if (m_ITerm < m_outMin)
    m_ITerm = m_outMin;

  //    setKiErrorThreshold(Max/m_kp);
}

void PIDDynamicSampleTime::setAutoMode(bool mode)
{
  //    if (mode == !m_inAuto)
  //    { /*we just went from manual to auto*/
  //        Initialize();
  //    }
  m_inAuto = mode;
  m_lastTick = millis() - 1;
}

void PIDDynamicSampleTime::setSetPoint(float setpoint, float feedForward)
{
  m_setpoint = setpoint;
  m_lastTick = millis() - 1;

  m_feedForwardInjection = feedForward;
}

void PIDDynamicSampleTime::setSetPointRange(float setpointRange)
{
  m_setpointRange = setpointRange;
  updateTrapezoidalRange();
}

void PIDDynamicSampleTime::Initialize()
{
  //lastInput = Input;
  if (m_ki == 0)
    m_ITerm = 0;
  //    else
  //    m_ITerm = 0;
  if (m_ITerm > m_outMax)
    m_ITerm = m_outMax;
  else if (m_ITerm < m_outMin)
    m_ITerm = m_outMin;
  m_lastInput = 0;
  m_lastError = 0;
  m_feedForwardInjection = 0;
  m_lastTick = millis() - 1;
}

void PIDDynamicSampleTime::reset()
{
  //lastInput = Input;
  m_ITerm = 0;
  m_lastError = 0;
  m_output = 0;
  m_feedForwardInjection = 0;
  setEnabled(false);
}

void PIDDynamicSampleTime::setControllerDirection(bool isDirectOperation)
{
  m_controllerDirection = (isDirectOperation) ? DIRECT : REVERSE;
}

int PIDDynamicSampleTime::getControllerDirection()
{
  return m_controllerDirection;
}

void PIDDynamicSampleTime::updateTrapezoidalRange()
{
  if (m_kiErrorThreshold < 0 || m_setpointRange <= 0)
  {
    m_advancedPid = false;
  }
  else
  {
    m_advancedPid = true;
    if (m_kiErrorThreshold == 0)
    {
      if (m_kp != 0)
      {
        m_kTrapezoidalRange = m_outMax / m_kp;
        m_kTrapezoidalRange *= m_kTrapezoidalRange;
      }
      else
        m_kTrapezoidalRange = 100 * 100;
    }
    else
      m_kTrapezoidalRange = m_kiErrorThreshold * m_kiErrorThreshold;
  }
}

void PIDDynamicSampleTime::setEnabled(bool enabled)
{
  if (enabled == !m_enabled)
  { /*we just went from manual to auto*/
    Initialize();
  }
  m_enabled = enabled;
  m_lastTick = millis() - 1;
  m_feedForwardInjection = 0;
  //    if (!m_advancedPid)
  //        m_ITerm = 0;
}


