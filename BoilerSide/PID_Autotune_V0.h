#ifndef PID_AutoTune_v0
#define PID_AutoTune_v0
#define LIBRARY_VERSION  0.0.1

enum E_PID_ControlType
{
    E_PID_ControlType_PI, E_PID_ControlType_PID
};

enum E_AutoTuneState
{
    E_AutoTuneState_Idle,
    E_AutoTuneState_Setpoint,
    E_AutoTuneState_Running
};

class PID_ATune
{

public:
    //commonly used functions **************************************************************************
    PID_ATune();                        // * Constructor.  links the Autotune to a given PID
    PID_ATune(E_PID_ControlType controlType, int lookback, float setpoint, float overshoot, float powerStart, float powerStep);
    float Compute(float input);               // * Similar to the PID Compue function, returns non 0 when done
    void Cancel();                      // * Stops the AutoTune

    void SetOutputStep(float);          // * how far above and below the starting value will the output step?
    float GetOutputStep();                  //

    void SetControlType(E_PID_ControlType type);  // * Determies if the tuning parameters returned will be PI (D=0)
    E_PID_ControlType GetControlType();                 //   or PID.  (0=PI, 1=PID)

    void SetLookbackSec(int);             // * how far back are we looking to identify peaks
    int GetLookbackSec();               //

    void SetNoiseBand(float);           // * the autotune will ignore signal chatter smaller than this value
    float GetNoiseBand();               //   this should be acurately set

    float GetKp();                    // * once autotune is complete, these functions contain the
    float GetKi();                    //   computed tuning parameters.
    float GetKd();                    //

    void StartAutoTune();

    float GetOutputStart() const
    {
        return m_outputStart;
    }

    void SetOutputStart(float outputStart)
    {
        m_outputStart = outputStart;
    }

    float GetSetpoint() const
    {
        return m_setpoint;
    }

    void SetSetpoint(float setpoint)
    {
        m_setpoint = setpoint;
    }

    bool isRunning() const
    {
        return (m_runningState != E_AutoTuneState_Idle);
    }

private:
    void FinishUp();
    bool m_isMax, m_isMin;
    //float *input, *output;
    float m_input, m_output;
    float m_setpoint;
    float m_noiseBand;
    E_PID_ControlType m_controlType;
    E_AutoTuneState m_runningState;
    unsigned long m_peak1, m_peak2, m_lastTime;
    int m_sampleTime;
    int m_nLookBack;
    int m_peakType;
    float m_lastInputs[101];
    float m_peaks[10];
    int m_peakCount;
    bool m_justchanged;
    //bool justevaled;
    float m_absMax, m_absMin;
    float m_oStep;
    float m_outputStart;
    float m_Ku, m_Pu;

};
#endif


