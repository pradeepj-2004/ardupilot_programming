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

    // Wait until armed
    if (!copter.motors->armed()) {
        gcs().send_text(MAV_SEVERITY_INFO, "Waiting for arming...");
        return;
    }

    // 🔥 Kill all internal controllers
    attitude_control->relax_attitude_controllers();
    copter.motors->set_throttle(0);

    // Read RC throttle (1000–2000)
    uint16_t pwm = channel_throttle->get_radio_in();

    // Safety clamp
    pwm = constrain_uint16(pwm, 1000, 2000);

    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // 3. Directly write PWM values to the first four channels
    // Note: SRV_Channel indices 0-3 correspond to Pixhawk Ports 1-4
    SRV_Channels::set_output_pwm_chan_timeout(0, pwm, 100); // Port 1: 1500us
    SRV_Channels::set_output_pwm_chan_timeout(1, pwm, 100); // Port 2: 1200us
    SRV_Channels::set_output_pwm_chan_timeout(2, 1000, 100); // Port 3: 1100us
    SRV_Channels::set_output_pwm_chan_timeout(3, 1000, 100); // Port 4: 1900us

    // Debug print (1 Hz)
    if (AP_HAL::millis() - last_print > 1000) {
        gcs().send_text(MAV_SEVERITY_INFO, "PWM: %u", pwm);
        last_print = AP_HAL::millis();
    }

}
