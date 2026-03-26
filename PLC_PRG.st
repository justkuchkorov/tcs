PROGRAM PLC_PRG
VAR
    Python_Velocity   : INT;
    Python_WheelSpeed : INT;
    Python_SlipRatio  : INT;
    Python_AxleLoad   : INT;
    PLC_TorqueCommand : INT;

    rVehicleSpeed     : REAL;
    rWheelSpeed       : REAL;
    rSlipRatioRaw     : REAL;
    rSlipRatioFilt    : REAL := 0.0;
    rAxleLoad         : REAL;

    bStartTest        : BOOL := FALSE;
    rDriverRequest    : REAL := 3000.0;
    rLaunchTorqueLimit: REAL := 800.0;  // Safe open-loop torque for 0 to 3 m/s
    rTargetTorque     : REAL := 0.0;
    rTargetTorquePrev : REAL := 0.0;
    rPidTorque        : REAL := 0.0;
    rRequestedTorque  : REAL := 0.0;

    fbTractionPI      : PID_FIXCYCLE;
    rSlipSetpoint     : REAL := 0.15;
    rKp               : REAL; // Initialized in code to prevent memory lock
    rTn               : REAL; // Initialized in code
    rTv               : REAL; // Initialized in code
    rCycle            : REAL := 0.02; // 50Hz match

    // --- Filtering / Smoothing ---
    rFilterTau        : REAL := 0.10;
    rAlpha            : REAL;
    rSlipDeadband     : REAL := 0.02;

    // --- Torque slew-rate limits [Nm per 20 ms cycle] ---
    rMaxRisePerCycle  : REAL := 80.0;
    rMaxFallPerCycle  : REAL := 140.0;
END_VAR

// ======================================================================
// 1. READ & UN-SCALE DATA FROM PYTHON
// ======================================================================
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
// 2. TRACTION CONTROLLER (THE BRAIN)
// ======================================================================
rKp := 3000.0; 
rTn := 0.60;
rTv := 0.0;

IF NOT bStartTest THEN
    fbTractionPI(
        ACTUAL := rSlipRatioFilt,
        SET_POINT := rSlipSetpoint,
        KP := rKp,
        TN := rTn,
        TV := rTv,
        Y_MIN := 0.0,
        Y_MAX := rDriverRequest,
        RESET := TRUE,
        CYCLE := rCycle
    );
    rTargetTorque := 0.0;
    rTargetTorquePrev := 0.0;

ELSE
    IF rVehicleSpeed < 3.0 THEN
        rPidTorque := rLaunchTorqueLimit; 
        fbTractionPI.ACTUAL := rSlipSetpoint; 

    ELSE
        IF ABS(rSlipRatioFilt - rSlipSetpoint) <= rSlipDeadband THEN
            fbTractionPI.ACTUAL := rSlipSetpoint; 
        ELSE
            fbTractionPI.ACTUAL := rSlipRatioFilt;
        END_IF;
    END_IF;
    fbTractionPI(
        SET_POINT := rSlipSetpoint,
        KP := rKp,
        TN := rTn,
        TV := rTv,
        Y_MIN := 0.0,
        Y_MAX := rDriverRequest,
        RESET := FALSE,
        CYCLE := rCycle
    );
    IF rVehicleSpeed >= 3.0 THEN
        rPidTorque := fbTractionPI.Y;
    END_IF;
    rRequestedTorque := MIN(rDriverRequest, rPidTorque);

    IF rRequestedTorque > rTargetTorquePrev + rMaxRisePerCycle THEN
        rTargetTorque := rTargetTorquePrev + rMaxRisePerCycle;
    ELSIF rRequestedTorque < rTargetTorquePrev - rMaxFallPerCycle THEN
        rTargetTorque := rTargetTorquePrev - rMaxFallPerCycle;
    ELSE
        rTargetTorque := rRequestedTorque;
    END_IF;

    rTargetTorque := LIMIT(0.0, rTargetTorque, rDriverRequest);
    rTargetTorquePrev := rTargetTorque;
END_IF;

// ======================================================================
// 3. SCALE & WRITE DATA TO PYTHON
// ======================================================================
PLC_TorqueCommand := REAL_TO_INT(rTargetTorque);
