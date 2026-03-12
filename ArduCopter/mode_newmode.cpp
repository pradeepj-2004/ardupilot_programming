#include "Copter.h"

/*
 * Init and run calls for stabilize flight mode
 */

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more

bool ModeNewMode::init(bool ignore_checks)
{
    gcs().send_text(MAV_SEVERITY_INFO, "Entered NEW_MODE");
    return true;
}

void ModeNewMode::run()
{
    // gcs().send_text(MAV_SEVERITY_INFO, "NEW MODE RUNNING");
    static uint32_t last_print = 0;

    if (AP_HAL::millis() - last_print > 1000) {

        float roll  = degrees(copter.ahrs.get_roll());
        float pitch = degrees(copter.ahrs.get_pitch());
        float yaw   = degrees(copter.ahrs.get_yaw());

        gcs().send_text(MAV_SEVERITY_INFO,
                        "Roll: %.2f  Pitch: %.2f  Yaw: %.2f",
                        roll, pitch, yaw);

        last_print = AP_HAL::millis();
    }
    // Check if motors are armed
    if (!copter.motors->armed()) {
            gcs().send_text(MAV_SEVERITY_INFO, "Motors not armed, skipping motor output");

        return;
        
    }
    uint16_t pwm = 1200;
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    SRV_Channels::set_output_pwm(SRV_Channel::k_motor1, pwm);
    SRV_Channels::set_output_pwm(SRV_Channel::k_motor2, pwm);

    // Keep other motors off
    SRV_Channels::set_output_pwm(SRV_Channel::k_motor3, 1000);
    SRV_Channels::set_output_pwm(SRV_Channel::k_motor4, 1000);



}
