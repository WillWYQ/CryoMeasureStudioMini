# Temperature System

## Detailed System Description

The Temperature System is responsible for measuring temperature from a PT1000 resistance temperature detector (RTD) using the microcontroller's on-chip ADC. The system reads the analog voltage associated with the RTD measurement path, uses the known reference resistor value (`Rref`) and the bus or excitation voltage, calculates the RTD resistance, and then converts that resistance into temperature using a PT1000 RTD curve-fit model.

The purpose of this module is to provide a complete software path from raw analog measurement to a final temperature value that can be used by the rest of the application. Instead of exposing low-level ADC details to higher-level software, the Temperature System abstracts the full measurement process into a small set of initialization, acquisition, computation, and conversion functions.

During initialization, the system is configured with the ADC input pins or channels used for the RTD measurement, the reference measurement, and the bus-voltage measurement. It also stores the electrical parameters required for the resistance calculation, including the known reference resistance value and any ADC scaling information. Once initialized and requested, the module can sample the configured analog channels, compute the RTD resistance from the measured voltages, apply the PT1000 curve-fit equation, and return the resulting temperature in degrees Fer, Celsius, or Kelvin, depending on the application requirement.

The processing flow of the Temperature System is:

1. Configure the ADC channels and measurement pins.
2. Read the RTD-related analog voltage.
3. Compute the RTD resistance using the known `Rref` and measured voltage ratio.
4. Convert RTD resistance to temperature using the PT1000 curve-fit model.
5. Return the final temperature value to the caller.

This design separates hardware acquisition from temperature computation, which improves maintainability, calibration support, and future extensibility. For example, the same software structure can later support filtering, calibration offsets, fault detection, multiple RTD channels, or a different ADC implementation without changing the higher-level application logic.

## Function List

### Initialization and Configuration

#### `bool TempSystem_Init(const TempSystemConfig* cfg);`
Initializes the Temperature System with the selected ADC channels or pins, the known reference resistor value, the bus-voltage input, and the RTD model parameters. This function prepares the module for temperature measurement.

#### `void TempSystem_Deinit(void);`
Releases or resets any resources used by the Temperature System. This function is optional, but it is useful if the module needs to be shut down or reconfigured.

#### `bool TempSystem_SetReferenceResistance(float rref_ohms);`
Updates the stored reference resistor value used in RTD resistance calculation. This is useful when calibration data is available or when the hardware configuration changes.

#### `bool TempSystem_SetCurveFitCoefficients(const PT1000CurveFit* fit);`
Loads or updates the coefficients used by the PT1000 curve-fit model for resistance-to-temperature conversion.

### Data Acquisition

#### `bool TempSystem_ReadMeasurementVoltage(float* v_meas);`
Reads the ADC value for the RTD measurement path and converts it into a voltage value.

#### `bool TempSystem_ReadReferenceVoltage(float* v_ref);`
Reads the ADC value associated with the reference path, if the design uses a measured reference voltage.

#### `bool TempSystem_ReadBusVoltage(float* v_bus);`
Reads the ADC value associated with the bus or excitation voltage used in the RTD resistance calculation.

#### `bool TempSystem_ReadRawAdc(uint16_t* raw_meas, uint16_t* raw_ref, uint16_t* raw_bus);`
Returns raw ADC counts for debugging, calibration, or diagnostics before voltage conversion is applied.

### Computation

#### `bool TempSystem_ComputeRTDResistance(float v_meas, float v_ref, float v_bus, float* r_rtd);`
Computes the RTD resistance from the measured voltages and the known reference resistor value. The exact equation depends on the analog front-end topology.

#### `bool TempSystem_ResistanceToTemperature(float r_rtd, float* temperature);`
Converts RTD resistance into temperature using the PT1000 curve-fit model.

#### `bool TempSystem_ApplyCalibration(float* temperature);`
Applies calibration offset, gain correction, or compensation to the computed temperature value.

### High-Level Measurement

#### `bool TempSystem_ReadTemperature(float* temperature);`
Performs a complete measurement cycle: acquire voltages, compute RTD resistance, convert resistance to temperature, and return the final result.

#### `bool TempSystem_ReadTemperatureDetailed(TempMeasurementResult* result);`
Performs a complete measurement cycle and returns detailed intermediate values such as raw ADC counts, voltages, RTD resistance, and final temperature. This is useful for debugging and validation.

### Status and Diagnostics

#### `bool TempSystem_IsInitialized(void);`
Returns whether the module has been successfully initialized.

#### `bool TempSystem_CheckFault(void);`
Checks for common fault conditions such as invalid ADC readings, open RTD, shorted RTD, or out-of-range voltage values.

#### `TempSystemStatus TempSystem_GetLastStatus(void);`
Returns the last module status or error code for diagnostics.

#### `int pinUsed(void)`
return array of pin that is used by this system
