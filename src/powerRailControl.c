#include <std_includes.h> // actual includes TBC

/ * =========================================================================
 * ASSUMPTIONS
 * =========================================================================
 * 1. Assumes a 32-bit architecture mcu where a single write to a 32-bit variable
 * takes a single clock cycle.(Important for atomic reading of volatile vars)
 *
 * 2. Assumes the 5V and 12V are routed back to a shared ADC pin.
 *
 * 3. Assumes voltage switching is instant or calling function handles the delay timing. 
 *
 * 4. Assumes that 'Sensor_SetPowerMode' and 'Sensor_VerifyVoltage' are 
 * called asynchronously by an upper-layer scheduler. 
 *
 * NOTE: any functions with a Sensor_ prefix would be in the header and public. 
 * ========================================================================= */

/*** SYSTEM MACROS ***/

// Voltage verification targets
#define VOLTAGE_0V_TARGET       0.0f
#define VOLTAGE_5V_TARGET       5.0f
#define VOLTAGE_12V_TARGET      12.0f

// Electrical diagnostics constraints
#define VOLTAGE_TOLERANCE       0.5f  // +/- 0.5V acceptable window for electrical noise

// Mode-specific hardware scaling factors to translate 0-3V ADC readings back to rail voltages
#define SCALE_RATIO_OFF         1.0f  // no scaling needed when rail is off
#define SCALE_RATIO_5V          1.667f // scales 3.0V max ADC pin exposure to a 5.0V rail max (5/3)
#define SCALE_RATIO_12V         4.0f  // scales 3.0V max ADC pin exposure to a 12.0V rail max (12/3)

/*** ENUMS ***/

typedef enum {
    POWER_MODE_OFF = 0,
    POWER_MODE_5V,                    // Implied value of 1
    POWER_MODE_12V                    // Implied value of 2
} PowerMode_t;

typedef enum {
    SYS_STATUS_SUCCESS = 0,
    SYS_ERROR_INVALID,
    SYS_ERROR_VOLTAGE_FAULT
} SystemStatus_t;

/*** GLOBALS ***/

// current active power state, read by diagnostics and written by control interfaces
static volatile PowerMode_t power_mode = POWER_MODE_OFF; // default to off on boot

/*** PUBLIC ***/

/**
 * @brief Safely transitions the sensor hardware between power supply rails.
 *
 * @param new_power_mode The desired PowerMode_t (OFF, 5V, or 12V) to select.
 */
void Sensor_SetPowerMode(PowerMode_t new_power_mode) {
    if (new_power_mode == POWER_MODE_OFF || 
         new_power_mode == POWER_MODE_5V || 
         new_power_mode == POWER_MODE_12V) {
        
        // turn both rails off before anything else
        hw_write_gpio_5v(false);  
        hw_write_gpio_12v(false); 
        
        // select power rail
        switch (new_power_mode) {
        case POWER_MODE_5V:
            hw_write_gpio_5v(true);
            break;
            
        case POWER_MODE_12V:
            hw_write_gpio_12v(true);
            break;
            
        case POWER_MODE_OFF:
        default:
            // Do nothing, rails remain off
            break;
        }
        
        // update global state tracker
        power_mode = new_power_mode;
    }
    // else- change nothing
}

/**
 * @brief  Reads the 0-3V ADC peripheral, applies a mode-specific scaling factor,
 * and verifies that the true rail voltage sits within tolerance limits.
 * 
 * @param  measured_voltage_out  Pointer to write the calculated real-world voltage.
 * @return SystemStatus_t        SYS_STATUS_SUCCESS if within limits, SYS_ERROR_VOLTAGE_FAULT 
 * if voltage out of range and SYS_ERROR_INVALID for a caller error. 
 */
SystemStatus_t Sensor_VerifyVoltage(float *measured_voltage_out) {
    SystemStatus_t status = SYS_ERROR_INVALID; // init to failed state
    float pin_volts = 0.0f;
    float real_voltage = 0.0f;
    float target_voltage = 0.0f;
    float current_scale_ratio = 1.0f;
    float voltage_error = 0.0f;
    
    // always check for nullptr when passed in as param
    if (measured_voltage_out != NULL) {
        
        // read raw analog voltage present directly at the 0-3V physical pin
        pin_volts = hw_read_adc_pin_volts();
        
        // assign target voltage and appropriate scaling factor based on the active power mode
        switch (power_mode) {
        case POWER_MODE_OFF:
            target_voltage = VOLTAGE_0V_TARGET;
            current_scale_ratio = SCALE_RATIO_OFF;
            status = SYS_STATUS_SUCCESS;
            break;
            
        case POWER_MODE_5V:
            target_voltage = VOLTAGE_5V_TARGET;
            current_scale_ratio = SCALE_RATIO_5V;
            status = SYS_STATUS_SUCCESS;
            break;
            
        case POWER_MODE_12V:
            target_voltage = VOLTAGE_12V_TARGET;
            current_scale_ratio = SCALE_RATIO_12V;
            status = SYS_STATUS_SUCCESS;
            break;
            
        default:
            // keep at status = error if mode is somehow corrupted
            break;
        }
        
        // verify if the voltage is in range (only if we've been successful up to this point)
        if (status == SYS_STATUS_SUCCESS) {
            
            // apply scaling factor
            real_voltage = pin_volts * current_scale_ratio;
            
            // calculate delta
            voltage_error = real_voltage - target_voltage;
            
            // confirm that the error isn't greater than our tolerance (accounting for electric noise)
            if (abs(voltage_error) > VOLTAGE_TOLERANCE) {
                status = SYS_ERROR_VOLTAGE_FAULT;
            }
        }
        
        // assign output and return final status
        *measured_voltage_out = real_voltage;
    }
    
    return status;
}