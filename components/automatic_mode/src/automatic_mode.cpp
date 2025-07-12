// components/automatic_mode/src/automatic_mode.cpp
// ═══════════════════════════════════════════════════════════════════════════════
// AUTOMATIC_MODE.CPP - MODE 2: AUTONOMOUS 5 M/S CYCLING IMPLEMENTATION
// ═══════════════════════════════════════════════════════════════════════════════

#include "automatic_mode.h"
#include "hardware_control.h"
#include "sensor_health.h"
#include "mode_coordinator.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <cmath>
#include <cstring>

static const char* TAG = "AUTOMATIC_MODE";

// FIXED: Add missing constant definition
#define COAST_DETECTION_SPEED_MS    0.1f      // Speed threshold for coast detection
#define AUTO_MODE_MIN_WIRE_LENGTH_M 2.0f      // Minimum wire length for automatic mode

// ═══════════════════════════════════════════════════════════════════════════════
// GLOBAL AUTOMATIC MODE STATE
// ═══════════════════════════════════════════════════════════════════════════════

static automatic_mode_progress_t g_auto_progress = {
    .state = AUTO_MODE_IDLE,
    .state_start_time = 0,
    .mode_start_time = 0,
    .current_target_speed = 0.0f,
    .acceleration_rate = AUTO_MODE_ACCEL_RATE_MS2,
    .esc_auto_armed = false,
    .coasting = {0},
    .coasting_active = false,
    .coasting_start_time = 0,
    .coasting_start_rotations = 0,
    .cycle_data = {0},
    .user_interrupted = false,
    .finishing_current_run = false,
    .wire_length_m = 0.0f,
    .current_position_m = 0.0f,
    .distance_to_wire_end_m = 0.0f,
    .status_message = {0},
    .error_message = {0},
    .error_count = 0
};

static automatic_mode_results_t g_auto_results = {
    .total_cycles_completed = 0,
    .total_runs_completed = 0,
    .total_operating_time_ms = 0,
    .total_distance_traveled_m = 0.0f,
    .average_cycle_time_ms = 0.0f,
    .max_speed_achieved_ms = 0.0f,
    .coasting_data = {0},
    .interrupted_by_user = false,
    .completion_reason = {0}
};

static bool g_auto_initialized = false;

// Speed control state
static float g_current_acceleration_target = 0.0f;
static uint64_t g_acceleration_start_time = 0;
static uint32_t g_acceleration_start_rotations = 0;

// Coasting state
static bool g_coasting_in_progress = false;
static uint64_t g_coasting_start_time = 0;
static uint32_t g_coasting_start_rotations = 0;
static float g_coasting_start_speed = 0.0f;

// User interruption tracking
static bool g_user_interruption_requested = false;
static uint64_t g_interruption_request_time = 0;

// ═══════════════════════════════════════════════════════════════════════════════
// COASTING CALIBRATION AND MANAGEMENT
// ═══════════════════════════════════════════════════════════════════════════════

esp_err_t automatic_mode_start_coasting_calibration(void) {
    if (g_auto_progress.coasting.calibrated) {
        ESP_LOGI(TAG, "Coasting already calibrated - skipping");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Starting coasting calibration at %.1f m/s", AUTO_COASTING_CALIBRATION_SPEED);
    
    g_auto_progress.state = AUTO_MODE_COASTING_CALIBRATION;
    g_auto_progress.state_start_time = esp_timer_get_time();
    
    // Accelerate to calibration speed
    esp_err_t result = automatic_mode_accelerate_to_speed(AUTO_COASTING_CALIBRATION_SPEED);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start acceleration for coasting calibration");
        return result;
    }
    
    strcpy(g_auto_progress.status_message, "Coasting calibration - accelerating to 5.0 m/s");
    return ESP_OK;
}

esp_err_t automatic_mode_update_coasting_calibration(float speed, float position) {
    static bool calibration_motor_stopped = false;
    
    // Wait until we reach calibration speed
    if (speed < AUTO_COASTING_CALIBRATION_SPEED - 0.2f) {
        return ESP_OK; // Still accelerating
    }
    
    // Stop motor and start measuring coasting
    if (!calibration_motor_stopped) {
        ESP_LOGI(TAG, "Reached %.1f m/s - stopping motor for coasting measurement", speed);
        
        hardware_set_motor_speed(0.0f, g_auto_progress.cycle_data.current_direction_forward);
        
        g_coasting_start_time = esp_timer_get_time();
        g_coasting_start_rotations = hardware_get_rotation_count();
        g_coasting_start_speed = speed;
        calibration_motor_stopped = true;
        
        strcpy(g_auto_progress.status_message, "Measuring coasting distance...");
        return ESP_OK;
    }
    
    // Monitor coasting until stopped
    if (speed > COAST_DETECTION_SPEED_MS) {
        return ESP_OK; // Still coasting
    }
    
    // Coasting complete - calculate results
    uint64_t coast_end_time = esp_timer_get_time();
    uint32_t coast_end_rotations = hardware_get_rotation_count();
    
    g_auto_progress.coasting.calibrated = true;
    g_auto_progress.coasting.calibration_speed_ms = g_coasting_start_speed;
    g_auto_progress.coasting.coasting_distance_m = hardware_rotations_to_distance(
        coast_end_rotations - g_coasting_start_rotations);
    g_auto_progress.coasting.coasting_time_ms = (coast_end_time - g_coasting_start_time) / 1000;
    g_auto_progress.coasting.deceleration_rate_ms2 = g_coasting_start_speed / 
        (g_auto_progress.coasting.coasting_time_ms / 1000.0f);
    g_auto_progress.coasting.coast_start_distance_m = g_auto_progress.coasting.coasting_distance_m + 
        AUTO_COASTING_SAFETY_MARGIN_M;
    g_auto_progress.coasting.calibration_rotations = coast_end_rotations - g_coasting_start_rotations;
    g_auto_progress.coasting.calibration_successful = true;
    
    // Validate coasting data
    if (g_auto_progress.coasting.coasting_distance_m < AUTO_COASTING_MIN_DISTANCE_M ||
        g_auto_progress.coasting.coasting_distance_m > AUTO_COASTING_MAX_DISTANCE_M) {
        ESP_LOGW(TAG, "Coasting distance out of expected range: %.2f m", 
                g_auto_progress.coasting.coasting_distance_m);
        g_auto_progress.coasting.calibration_successful = false;
        return ESP_ERR_INVALID_SIZE;
    }
    
    // Save coasting data
    coasting_data_t coasting_data = {
        .calibrated = true,
        .coasting_distance_m = g_auto_progress.coasting.coasting_distance_m,
        .coast_start_distance_m = g_auto_progress.coasting.coast_start_distance_m,
        .coast_time_ms = g_auto_progress.coasting.coasting_time_ms,
        .decel_rate_ms2 = g_auto_progress.coasting.deceleration_rate_ms2
    };
    
    mode_coordinator_set_coasting_data(&coasting_data);
    
    ESP_LOGI(TAG, "=== COASTING CALIBRATION COMPLETE ===");
    ESP_LOGI(TAG, "Coasting Distance: %.2f m", g_auto_progress.coasting.coasting_distance_m);
    ESP_LOGI(TAG, "Coasting Time: %lu ms", g_auto_progress.coasting.coasting_time_ms);
    ESP_LOGI(TAG, "Deceleration Rate: %.2f m/s²", g_auto_progress.coasting.deceleration_rate_ms2);
    ESP_LOGI(TAG, "Coast Start Distance: %.2f m from wire end", g_auto_progress.coasting.coast_start_distance_m);
    
    // Reset for normal operation
    calibration_motor_stopped = false;
    strcpy(g_auto_progress.status_message, "Coasting calibration complete");
    
    return ESP_OK;
}

bool automatic_mode_is_coasting_calibrated(void) {
    return g_auto_progress.coasting.calibrated;
}

coasting_calibration_t automatic_mode_get_coasting_data(void) {
    return g_auto_progress.coasting;
}

float automatic_mode_calculate_coasting_distance(float current_position, 
                                                float wire_length, 
                                                bool direction_forward) {
    if (!g_auto_progress.coasting.calibrated) {
        return AUTO_COASTING_SAFETY_MARGIN_M; // Default safety margin
    }
    
    float distance_to_wire_end;
    if (direction_forward) {
        distance_to_wire_end = wire_length - current_position;
    } else {
        distance_to_wire_end = current_position;
    }
    
    return distance_to_wire_end - g_auto_progress.coasting.coast_start_distance_m;
}

// ═══════════════════════════════════════════════════════════════════════════════
// SPEED CONTROL IMPLEMENTATION
// ═══════════════════════════════════════════════════════════════════════════════

esp_err_t automatic_mode_accelerate_to_speed(float target_speed) {
    if (target_speed > AUTO_MODE_MAX_SPEED_MS) {
        ESP_LOGW(TAG, "Target speed %.1f m/s exceeds maximum %.1f m/s", 
                target_speed, AUTO_MODE_MAX_SPEED_MS);
        target_speed = AUTO_MODE_MAX_SPEED_MS;
    }
    
    ESP_LOGI(TAG, "Starting gradual acceleration to %.1f m/s", target_speed);
    
    g_current_acceleration_target = target_speed;
    g_acceleration_start_time = esp_timer_get_time();
    g_acceleration_start_rotations = hardware_get_rotation_count();
    
    // Start with minimum speed
    esp_err_t result = hardware_set_motor_speed(AUTO_MODE_START_SPEED_MS, 
                                               g_auto_progress.cycle_data.current_direction_forward);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start acceleration");
        return result;
    }
    
    g_auto_progress.current_target_speed = AUTO_MODE_START_SPEED_MS;
    g_auto_progress.acceleration_rate = AUTO_MODE_ACCEL_RATE_MS2;
    
    return ESP_OK;
}

esp_err_t automatic_mode_decelerate_to_speed(float target_speed) {
    ESP_LOGI(TAG, "Starting deceleration to %.1f m/s", target_speed);
    
    float current_speed = hardware_get_current_speed();
    if (current_speed <= target_speed) {
        ESP_LOGW(TAG, "Already at or below target speed");
        return ESP_OK;
    }
    
    // Calculate deceleration time
    float speed_difference = current_speed - target_speed;
    float decel_time_s = speed_difference / AUTO_MODE_DECEL_RATE_MS2;
    
    // Gradual deceleration
    uint32_t steps = (uint32_t)(decel_time_s * 10); // 10 steps per second
    float speed_step = speed_difference / steps;
    
    for (uint32_t i = 0; i < steps; i++) {
        float new_speed = current_speed - (speed_step * (i + 1));
        hardware_set_motor_speed(new_speed, g_auto_progress.cycle_data.current_direction_forward);
        vTaskDelay(pdMS_TO_TICKS(100)); // 10 Hz update rate
        
        // Check for safety conditions
        if (!automatic_mode_is_operation_safe()) {
            ESP_LOGW(TAG, "Safety check failed during deceleration");
            automatic_mode_handle_emergency("Safety failure during deceleration");
            return ESP_ERR_INVALID_STATE;
        }
    }
    
    // Set final target speed
    hardware_set_motor_speed(target_speed, g_auto_progress.cycle_data.current_direction_forward);
    g_auto_progress.current_target_speed = target_speed;
    
    ESP_LOGI(TAG, "Deceleration to %.1f m/s complete", target_speed);
    return ESP_OK;
}

esp_err_t automatic_mode_maintain_cruise_speed(void) {
    float current_speed = hardware_get_current_speed();
    float target_speed = AUTO_MODE_MAX_SPEED_MS;
    
    // Simple speed maintenance with tolerance
    if (fabs(current_speed - target_speed) > 0.5f) {
        ESP_LOGD(TAG, "Adjusting cruise speed: %.1f → %.1f m/s", current_speed, target_speed);
        hardware_set_motor_speed(target_speed, g_auto_progress.cycle_data.current_direction_forward);
    }
    
    g_auto_progress.current_target_speed = target_speed;
    return ESP_OK;
}

float automatic_mode_get_current_target_speed(void) {
    return g_auto_progress.current_target_speed;
}

bool automatic_mode_is_at_target_speed(float tolerance) {
    float current_speed = hardware_get_current_speed();
    return fabs(current_speed - g_current_acceleration_target) <= tolerance;
}

// ═══════════════════════════════════════════════════════════════════════════════
// WIRE END DETECTION AND POSITIONING
// ═══════════════════════════════════════════════════════════════════════════════

bool automatic_mode_is_at_wire_end(void) {
    // Check multiple detection methods
    sensor_health_t sensor_status = sensor_health_get_status();
    
    // Impact detection
    if (sensor_status.total_accel_g > AUTO_MODE_MAX_IMPACT_G) {
        ESP_LOGI(TAG, "Wire end detected by impact: %.2f g", sensor_status.total_accel_g);
        return true;
    }
    
    // Hall sensor timeout
    if (hardware_get_time_since_last_hall_pulse() > 2000000ULL) { // 2 seconds
        ESP_LOGI(TAG, "Wire end detected by Hall timeout");
        return true;
    }
    
    // Speed drop detection
    float current_speed = hardware_get_current_speed();
    if (g_auto_progress.current_target_speed > 0.5f && current_speed < 0.2f) {
        ESP_LOGI(TAG, "Wire end detected by speed drop: %.1f m/s", current_speed);
        return true;
    }
    
    return false;
}

// ═══════════════════════════════════════════════════════════════════════════════
// MAIN STATE MACHINE IMPLEMENTATION
// ═══════════════════════════════════════════════════════════════════════════════

static esp_err_t handle_coasting_state(void) {
    float current_speed = hardware_get_current_speed();
    
    // Check if wire end reached
    if (automatic_mode_is_at_wire_end() || current_speed < COAST_DETECTION_SPEED_MS) {
        ESP_LOGI(TAG, "Wire end reached via coasting - speed: %.2f m/s", current_speed);
        
        g_coasting_in_progress = false;
        g_auto_progress.state = AUTO_MODE_WIRE_END_APPROACH;
        
        // Final approach at low speed
        hardware_set_motor_speed(AUTO_MODE_WIRE_END_APPROACH_MS, 
                                g_auto_progress.cycle_data.current_direction_forward);
        strcpy(g_auto_progress.status_message, "Final approach to wire end");
        
        // Brief approach time
        vTaskDelay(pdMS_TO_TICKS(500));
        
        // Handle wire end reached
        automatic_mode_handle_wire_end_reached();
    }
    
    return ESP_OK;
}

// ═══════════════════════════════════════════════════════════════════════════════
// PUBLIC API IMPLEMENTATION (Core Functions Only)
// ═══════════════════════════════════════════════════════════════════════════════

esp_err_t automatic_mode_init(void) {
    ESP_LOGI(TAG, "Initializing automatic mode...");
    
    // Reset automatic mode state
    memset(&g_auto_progress, 0, sizeof(g_auto_progress));
    memset(&g_auto_results, 0, sizeof(g_auto_results));
    
    g_auto_progress.state = AUTO_MODE_IDLE;
    strcpy(g_auto_progress.status_message, "Automatic mode ready");
    strcpy(g_auto_progress.error_message, "");
    
    g_auto_initialized = true;
    
    ESP_LOGI(TAG, "Automatic mode initialized");
    return ESP_OK;
}

esp_err_t automatic_mode_start(void) {
    if (!g_auto_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Starting automatic mode...");
    
    // Validate prerequisites
    esp_err_t result = automatic_mode_validate_prerequisites();
    if (result != ESP_OK) {
        return result;
    }
    
    // Get wire learning data
    const wire_learning_results_t* wire_data = mode_coordinator_get_wire_learning_results();
    if (wire_data == NULL || !wire_data->complete) {
        ESP_LOGE(TAG, "Wire learning data not available");
        strcpy(g_auto_progress.error_message, "Wire learning required before automatic mode");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Initialize automatic mode state
    g_auto_progress.state = AUTO_MODE_INITIALIZING;
    g_auto_progress.mode_start_time = esp_timer_get_time();
    g_auto_progress.wire_length_m = wire_data->wire_length_m;
    g_auto_progress.user_interrupted = false;
    g_auto_progress.finishing_current_run = false;
    g_user_interruption_requested = false;
    
    // Auto-arm ESC
    g_auto_progress.state = AUTO_MODE_ARMING_ESC;
    strcpy(g_auto_progress.status_message, "Auto-arming ESC...");
    
    result = automatic_mode_auto_arm_esc();
    if (result != ESP_OK) {
        strcpy(g_auto_progress.error_message, "Failed to auto-arm ESC");
        g_auto_progress.state = AUTO_MODE_ERROR;
        return result;
    }
    
    ESP_LOGI(TAG, "Automatic mode started successfully");
    return ESP_OK;
}

esp_err_t automatic_mode_stop_graceful(void) {
    ESP_LOGI(TAG, "Graceful stop requested - will finish current run");
    
    g_user_interruption_requested = true;
    g_interruption_request_time = esp_timer_get_time();
    g_auto_progress.finishing_current_run = true;
    
    strcpy(g_auto_progress.status_message, "Stopping gracefully - finishing current run");
    
    return ESP_OK;
}

esp_err_t automatic_mode_interrupt(void) {
    ESP_LOGI(TAG, "Immediate interruption requested");
    
    // Stop motor immediately
    hardware_emergency_stop();
    
    // Update state
    g_auto_progress.state = AUTO_MODE_STOPPING_INTERRUPTED;
    g_auto_progress.user_interrupted = true;
    g_auto_progress.finishing_current_run = false;
    g_user_interruption_requested = true;
    
    strcpy(g_auto_progress.status_message, "Interrupted by user - stopping immediately");
    
    // Auto-disarm ESC
    automatic_mode_auto_disarm_esc();
    
    return ESP_OK;
}

esp_err_t automatic_mode_update(void) {
    if (!g_auto_initialized || g_auto_progress.state == AUTO_MODE_IDLE) {
        return ESP_OK;
    }
    
    // Main state machine (simplified)
    switch (g_auto_progress.state) {
        case AUTO_MODE_COASTING:
            handle_coasting_state();
            break;
            
        case AUTO_MODE_COASTING_CALIBRATION:
            {
                float current_speed = hardware_get_current_speed();
                automatic_mode_update_coasting_calibration(current_speed, g_auto_progress.current_position_m);
            }
            break;
            
        default:
            // Other states handled elsewhere
            break;
    }
    
    return ESP_OK;
}

bool automatic_mode_is_active(void) {
    return g_auto_progress.state > AUTO_MODE_IDLE && 
           g_auto_progress.state < AUTO_MODE_COMPLETE;
}

automatic_mode_progress_t automatic_mode_get_progress(void) {
    return g_auto_progress;
}

automatic_mode_results_t automatic_mode_get_results(void) {
    return g_auto_results;
}

// ═══════════════════════════════════════════════════════════════════════════════
// ESSENTIAL HELPER FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════════

esp_err_t automatic_mode_validate_prerequisites(void) {
    // Check sensor validation
    if (!mode_coordinator_are_sensors_validated()) {
        ESP_LOGE(TAG, "Sensors not validated - cannot start automatic mode");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Check wire learning completion
    const wire_learning_results_t* wire_data = mode_coordinator_get_wire_learning_results();
    if (wire_data == NULL || !wire_data->complete) {
        ESP_LOGE(TAG, "Wire learning not complete - cannot start automatic mode");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Validate wire length
    if (wire_data->wire_length_m < AUTO_MODE_MIN_WIRE_LENGTH_M) {
        ESP_LOGE(TAG, "Wire too short for automatic mode: %.2f m < %.2f m", 
                wire_data->wire_length_m, AUTO_MODE_MIN_WIRE_LENGTH_M);
        return ESP_ERR_INVALID_SIZE;
    }
    
    return ESP_OK;
}

bool automatic_mode_is_operation_safe(void) {
    // Check system health
    if (!mode_coordinator_is_motion_safe()) {
        return false;
    }
    
    return true;
}

esp_err_t automatic_mode_handle_emergency(const char* error_message) {
    ESP_LOGE(TAG, "EMERGENCY: %s", error_message);
    
    // Stop motor immediately
    hardware_emergency_stop();
    
    // Update state
    g_auto_progress.state = AUTO_MODE_ERROR;
    strncpy(g_auto_progress.error_message, error_message, sizeof(g_auto_progress.error_message) - 1);
    
    strcpy(g_auto_progress.status_message, "EMERGENCY STOP - Automatic mode halted");
    
    return ESP_OK;
}

esp_err_t automatic_mode_auto_arm_esc(void) {
    if (hardware_esc_is_armed()) {
        ESP_LOGI(TAG, "ESC already armed");
        g_auto_progress.esc_auto_armed = true;
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Auto-arming ESC for automatic mode");
    
    esp_err_t result = hardware_esc_arm();
    if (result == ESP_OK) {
        g_auto_progress.esc_auto_armed = true;
        ESP_LOGI(TAG, "ESC auto-armed successfully");
    } else {
        ESP_LOGE(TAG, "Failed to auto-arm ESC");
    }
    
    return result;
}

esp_err_t automatic_mode_auto_disarm_esc(void) {
    ESP_LOGI(TAG, "Auto-disarming ESC");
    
    esp_err_t result = hardware_esc_disarm();
    g_auto_progress.esc_auto_armed = false;
    
    return result;
}

esp_err_t automatic_mode_handle_wire_end_reached(void) {
    ESP_LOGI(TAG, "Wire end reached - completing current run");
    
    // Stop motor immediately
    hardware_set_motor_speed(0.0f, g_auto_progress.cycle_data.current_direction_forward);
    
    return ESP_OK;
}

const char* automatic_mode_state_to_string(automatic_mode_state_t state) {
    switch (state) {
        case AUTO_MODE_IDLE: return "Idle";
        case AUTO_MODE_INITIALIZING: return "Initializing";
        case AUTO_MODE_ARMING_ESC: return "Arming ESC";
        case AUTO_MODE_ACCELERATING: return "Accelerating";
        case AUTO_MODE_CRUISING: return "Cruising";
        case AUTO_MODE_COASTING_CALIBRATION: return "Coasting Calibration";
        case AUTO_MODE_COASTING: return "Coasting";
        case AUTO_MODE_WIRE_END_APPROACH: return "Wire End Approach";
        case AUTO_MODE_DIRECTION_CHANGE: return "Direction Change";
        case AUTO_MODE_CYCLE_COMPLETE: return "Cycle Complete";
        case AUTO_MODE_STOPPING_GRACEFUL: return "Stopping Gracefully";
        case AUTO_MODE_STOPPING_INTERRUPTED: return "Stopping Interrupted";
        case AUTO_MODE_ERROR: return "Error";
        case AUTO_MODE_COMPLETE: return "Complete";
        default: return "Unknown";
    }
}

int automatic_mode_get_progress_percentage(void) {
    switch (g_auto_progress.state) {
        case AUTO_MODE_IDLE: return 0;
        case AUTO_MODE_INITIALIZING: return 5;
        case AUTO_MODE_ARMING_ESC: return 10;
        case AUTO_MODE_COASTING_CALIBRATION: return 20;
        case AUTO_MODE_ACCELERATING: return 30;
        case AUTO_MODE_CRUISING: return 55;
        case AUTO_MODE_COASTING: return 70;
        case AUTO_MODE_WIRE_END_APPROACH: return 85;
        case AUTO_MODE_DIRECTION_CHANGE: return 90;
        case AUTO_MODE_CYCLE_COMPLETE: return 95;
        case AUTO_MODE_COMPLETE: return 100;
        case AUTO_MODE_ERROR: return -1;
        default: return 0;
    }
}