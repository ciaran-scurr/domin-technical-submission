#include <std_includes.h> // actual includes TBC

/ * =========================================================================
 * ASSUMPTIONS
 * =========================================================================
 * 1. Assumes a 32-bit architecture mcu where a single write to a 32-bit variable
 * takes a single clock cycle.(Important for atomic reading of volatile vars)
 *
 * 2. Assumes 'raw_value' is updated continuously in the background by a 
 * high priority ISR. 
 *
 * 3. Assumes linear mapping of analogue/digital sensors.
 *
 * 4. Assumes 0 position is a safe 'all off' state for the actuator. 
 *
 * NOTE: any functions with a Sensor_ prefix would be in the header and public. 
 * ========================================================================= */

/*** SYSTEM MACROS ***/

#define NORM_MIN           -1.0f
#define NORM_MAX            1.0f

// Physical sensor limits matching hardware specifications
#define ANALOG_RAW_MIN      0.5f
#define ANALOG_RAW_MAX      2.5f
#define DIGITAL_RAW_MIN     10.0f
#define DIGITAL_RAW_MAX     90.0f

// clamp an output between min and max
#define CLAMP(val, min, max) ((val) > (max) ? (max) : ((val) < (min) ? (min) : (val)))

/*** ENUMS ***/

typedef enum {
    SENSOR_MODE_ANALOGUE = 0,
    SENSOR_MODE_DIGITAL = 1
} SensorMode_t;

typedef enum {
    SYS_STATUS_SUCCESS = 0,
    SYS_ERROR_SENSOR_OUT_OF_BOUNDS,
    SYS_ERROR_INVALID
} SystemStatus_t;

/*** GLOBALS ***/

// sensor state, updated asynchronously by undefined comms interface.
static volatile SensorMode_t sensor_mode = SENSOR_MODE_ANALOGUE; // default to analogue

// raw_reading. Updated in Sensor_SetMode()
static volatile float raw_reading = 0;  // updated async by the sensor

/*** PRIVATE ***/

/**
 * @brief Helper function to perform standard linear interpolation / mapping.
 * Maps an incoming raw value from its physical hardware span to a desired output span.
 *
 * @param raw_reading      The unscaled input value directly from the ADC.
 * @param input_min        The lower threshold of the hardware sensor's physical range.
 * @param input_max        The upper threshold of the hardware sensor's physical range.
 * @param output_min       The lower limit of the target normalization output (-1.0f).
 * @param output_max       The upper limit of the target normalization output (1.0f).
 * @return float           The scaled value projected onto the output range constraints.
 */
static float normalise_reading(float raw_reading, float input_min, float input_max, float output_min, float output_max) {
    float input_range = 0;
    float output_range = 0;
    float input_offset = 0;
    float scaled_reading = 0;
    float normalised_reading = 0;
    
    // would be a user error to set max < min but check for it anyway to avoid a crash 
    if (input_max <= input_min) {
        return 0.0f; // Or trigger a fault handler
    }
    
    input_range = input_max - input_min; // calculate total input range
    output_range = output_max - output_min; // calculate total output range
    
    input_offset = raw_reading - input_min;  // get raw input offset
    
    // scale the input offset to match the target output scale
    scaled_reading = (input_offset * output_range) / input_range;
    
    // shift the scaled value relative to the target minimum baseline
    normalised_reading = scaled_reading + output_min;
    
    // ensure rounding errors haven't pushed reading out of range
    normalised_reading = CLAMP(normalised_reading, output_min, output_max);
    
    return normalised_reading;
}

/**
 * @brief  Placveholder: Interface shell to process incoming packets from the Comms ISR.
 *
 * @param  packet_buffer  Pointer to the raw byte stream from the driver hardware.
 * @param  buffer_len     Length of the received data packet.
 */
void handle_incoming_packet(const uint8_t *packet_buffer, uint16_t buffer_len) {
    /* Out of scope but good to keep in mind how this would work.
     * Comms ISR dumps contents into a buffer asynchronously.
     * Here, when we have processor time, we parse that buffer and assign to relevant 
     * variables e.g. target_position or sensor_mode
     * Calls Sensor_SetMode() when packet type is sensor mode
     * Also obviously check for invalid packets etc.
     */
}

/*** PUBLIC ***/

/**
 * @brief Interface to update the global sensor mode.
 * Validates the mode before applying it to the system state.
 */
void Sensor_SetMode(SensorMode_t new_mode) {
    if (new_mode == SENSOR_MODE_ANALOGUE || new_mode == SENSOR_MODE_DIGITAL) {
        // NOTE: needs to be read atomically but can due to size should only take
        // a single instruction cycle 
        sensor_mode = new_mode;
    }
}



/**
 * @brief  Validates raw sensor data against active physical bounds
 * and normalizes the result into a strict -1.0 to +1.0 range.
 * 
 * @param  raw_value        The filtered sensor reading (Volts or PWM %). Global but pass as param for visibility.
 * @param  normalized_out   Pointer to write the calculated normalized output.
 * @return SystemStatus_t   SYS_STATUS_SUCCESS if successful, SYS_ERROR_INVALID for a
 * system error (e.g. nullptr) or SYS_ERROR_SENSOR_OUT_OF_BOUNDS for an out of
 * bounds reading.
 */
SystemStatus_t Sensor_ProcessReading(float raw_value, float *normalized_out) {
    float x_min = 0;
    float x_max = 0;
    SystemStatus_t status = SYS_STATUS_SUCCESS; // initialize to success state
    
    // always check for nullptr when passed in as param
    if (normalized_out == NULL) {
        status = SYS_ERROR_INVALID;
    } else {
        // select input range depending on sensor type
        switch (sensor_mode) {
        case SENSOR_MODE_ANALOGUE:
            x_min = ANALOG_RAW_MIN;
            x_max = ANALOG_RAW_MAX;
            break;

        case SENSOR_MODE_DIGITAL:
            x_min = DIGITAL_RAW_MIN;
            x_max = DIGITAL_RAW_MAX;
            break;

        default:
            status = SYS_ERROR_INVALID;
            break;
        }

        // only proceed with validation and math if the mode switch was valid
        if (status == SYS_STATUS_SUCCESS) {
            // check raw reading is in bounds and return error if not
            if (raw_value < x_min || raw_value > x_max) {
                *normalized_out = 0.0f; // force valve to 0 position on error for safety
                status = SYS_ERROR_SENSOR_OUT_OF_BOUNDS;
            } else {
                // normalise the reading to our specified range (in this case -1 to 1)
                *normalized_out = normalise_reading(raw_value, x_min, x_max, NORM_MIN, NORM_MAX);
            }
        }
    }

    return status; 
}

