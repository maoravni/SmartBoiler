#define DIRECT 0
#define REVERSE 1

#include <stdlib.h>
#include <stdint.h>

class PIDDynamicSampleTime
{
    /*working variables*/
    // temporary vars:
    unsigned long m_lastTick;
    float m_input, m_output, m_setpoint, m_lastOutput;
    float m_error, m_dError;
    float m_dInput;
    float m_lastError;
    float m_PTerm, m_DTerm;
    float m_ITerm, m_lastInput;
    float m_kp, m_ki, m_kd;
    float m_kiErrorThreshold; //!< When temp error is above this threshold, we don't calculate the iTerm.
    float m_outputSmoothingCoeff;
    float m_outMin, m_outMax;
    float m_setpointRange;
    bool m_inAuto;
    bool m_enabled;
    bool m_advancedPid;

    float m_kBeta;
    float m_kLinear;
    float m_advError, m_advBError;
    float m_kTrapezoidalRange;
    float m_feedForwardInjection;

    int m_controllerDirection;

    uint16_t m_pssId;

public:
    PIDDynamicSampleTime();
    virtual ~PIDDynamicSampleTime();

    /**
     * Compute the power output
     * @param input the current temperature.
     * @return the required output.
     */
    float Compute(float input);

    /**
     * Set the cooefficients for this control loop.
     * @param Kp
     * @param Ki
     * @param Kd
     */
    void SetTunings(float Kp, float Ki, float Kd, float kTrapezRange);

    /**
     * Set the power output limits.
     * @param Min
     * @param Max
     */
    void SetOutputLimits(float Min, float Max);

    /**
     * Set the mode of operation
     * @param Mode
     */
    void setAutoMode(bool mode);

    /**
     * Set the controller direction - more power means hotter or colder.
     * @param Direction
     */
    void setControllerDirection(bool isDirectOperation);
    int getControllerDirection();

    /**
     *
     * @param setpoint
     */
    void setSetPoint(float setpoint, float feedForward);
    float getSetPoint() {return m_setpoint;}

    void setSetPointRange(float setpointRange);

    bool isEnabled() const
    {
        return m_enabled;
    }

    void setEnabled(bool enabled);

    float getKd() const
    {
        return m_kd;
    }

    float getKi() const
    {
        return m_ki;
    }

    float getKp() const
    {
        return m_kp;
    }

    float getKiErrorThreshold() const
    {
        return m_kiErrorThreshold;
    }

    void setKiErrorThreshold(float kiErrorThreshold)
    {
        m_kiErrorThreshold = kiErrorThreshold;
    }

    int getOutputSmoothingWindow() const
    {
        return (int)(1/(m_outputSmoothingCoeff)-1);
    }

    void setOutputSmoothingWindow(int outputSmoothingWindow)
    {
        m_outputSmoothingCoeff = 1/((float)(outputSmoothingWindow+1));
    }

    /**
     * Initialize the control loop.
     */
    void Initialize();

    void reset();

    void updateTrapezoidalRange();

    uint16_t getPssId() const
    {
        return m_pssId;
    }

    void setPssId(uint16_t pssId)
    {
        m_pssId = pssId;
    }
};


