PROGRAM PLC_PRG
VAR
    // --- Modbus Variables ---
    Python_Velocity   : INT;
    Python_WheelSpeed : INT;
    Python_SlipRatio  : INT;
    Python_AxleLoad   : INT;
    PLC_TorqueCommand : INT;

    // --- Un-scaled Real-world Values ---
    rVehicleSpeed     : REAL;
    rWheelSpeed       : REAL;
    rSlipRatioRaw     : REAL;
    rSlipRatioFilt    : REAL := 0.0;
    rAxleLoad         : REAL;

    // --- Filtering / Smoothing ---
    rFilterTau        : REAL := 0.10;
    rAlpha            : REAL;
    rCycle            : REAL := 0.02; // 50Hz match
    
    // --- Output ---
    rTargetTorque     : REAL := 0.0;
END_VAR

// ======================================================================
// 1. READ & UN-SCALE DATA FROM PYTHON
// ======================================================================
rVehicleSpeed := INT_TO_REAL(Python_Velocity) / 100.0;
rWheelSpeed   := INT_TO_REAL(Python_WheelSpeed) / 100.0;
rSlipRatioRaw := INT_TO_REAL(Python_SlipRatio) / 1000.0;
rAxleLoad     := INT_TO_REAL(Python_AxleLoad); // Raw Newtons

// Low-pass filter the slip signal to reduce chatter from Modbus quantization
rAlpha := rCycle / (rFilterTau + rCycle);
rSlipRatioFilt := rSlipRatioFilt + rAlpha * (rSlipRatioRaw - rSlipRatioFilt);

// ======================================================================
// 3. SCALE & WRITE DATA TO PYTHON
// ======================================================================
PLC_TorqueCommand := REAL_TO_INT(rTargetTorque);
