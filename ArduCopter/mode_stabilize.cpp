#include "Copter.h"

/*
 * Init and run calls for stabilize flight mode
 */

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more



void ModeStabilize::run()
{

    // apply simple mode transform to pilot inputs
    update_simple_mode();


    motors->_wheel_mid=g7._wheel_mid;
    motors->_wheel_range=g7._wheel_range;
    motors->_left_servo_mid=g7._left_servo_mid;
    motors->_left_servo_nine=g7._left_servo_nine;
    motors->_right_servo_mid=g7._right_servo_mid;
    motors->_right_servo_nine=g7._right_servo_nine;
    motors->_hover_throttle_decouple=g7._hover_throttle_decouple;
    motors->_down_range=g7._down_range;
    motors->_lift_range=g7._lift_range;
    motors->_test_mode=g7._test_mode;
    motors->_test_servo_angle=g7._test_servo_angle;
    motors->_test_thrust=g7._test_thrust;
    


    attitude_control->_sqrt_p=g7._sqrt_p;
    attitude_control->_ang_accel_limit=g7._ang_accel_limit;
    attitude_control->_PWM_limit_upper=g7._PWM_limit_upper;

    int mode_d=g7._mode_switch_decouple;
    int mode_fly=g7._mode_switch_fly;
    int tran_low=g7._tran_throttle_low;
    int tran_high=g7._tran_trottle_high;
    float hover_th=g7.thr_ho;


    //gcs().send_text(MAV_SEVERITY_CRITICAL, "param %5.3f", (double)g7.bb_pit_r_P);

    // convert pilot input to lean angles
    float target_roll, target_pitch,balance_pitch,balance_roll,uncouple_pitch;
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, copter.aparm.angle_max);
    
    get_pilot_desired_lean_angles(balance_pitch,balance_roll,10, 10);
    
    // get_pilot_desired_lean_angles_gw(uncouple_pitch,9000);
    //float hover_th=throttle_hover();
    float ground_pitch=channel_pitch->get_control_in();
    float ground_yaw=channel_yaw->get_control_in();

    // gcs().send_text(MAV_SEVERITY_CRITICAL, "pin! %5.3f", (double)ground_pitch);
    // gcs().send_text(MAV_SEVERITY_CRITICAL, "yin! %5.3f", (double)ground_yaw);

    

    motors->yaw_gw=ground_yaw/4500;
    motors->pitch_gw=ground_pitch/4500;

    float pilot_desired_throttle = get_pilot_desired_throttle();
    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());

    float bbswitch=channel_mode->get_control_in();
    uncouple_pitch=(channel_uncouple_pitch->get_control_in()-500.0)/500.0;
    uncouple_pitch=uncouple_pitch*9000.0;
    //ground_pitch 500 ==0
    //gcs().send_text(MAV_SEVERITY_CRITICAL, "uncouple! %5.3f",uncouple_pitch);
    // gcs().send_text(MAV_SEVERITY_CRITICAL, "ground_pitch! %5.3f", (double)ground_pitch);
    float rc_in=channel_throttle->get_control_in();
    //gcs().send_text(MAV_SEVERITY_CRITICAL, "bbswitch! %5.3f", (double)rc_in);
    if(rc_in>tran_high){
        motors->set_mix(0);
    }else if(rc_in<tran_low){
        motors->set_mix(1);
    }else{
        motors->set_mix((tran_high-rc_in)/(tran_high-tran_low));
    }
    motors->rc_in=rc_in;



    // mode 0:uncouple mode 1:balanced mode ground  2 balanced transition mode 3:balanced mode fly
    int last_mode_atc=attitude_control->check_mode();
    float last_roll_atc=attitude_control->check_roll_angle();//this is the roll angle deg
    if(bbswitch<=mode_d && last_mode_atc==1){
        //this is the code for uncouple mode set
        attitude_control->set_target_mode(0);
        gcs().send_text(MAV_SEVERITY_CRITICAL, "balanced ground to uncoupled");
    }else if(bbswitch>mode_d && last_mode_atc==0 && -5<last_roll_atc && last_roll_atc<5&&bbswitch<=mode_fly){
        attitude_control->set_target_mode(1);
        attitude_control->reset_yaw_target_and_rate();
        gcs().send_text(MAV_SEVERITY_CRITICAL, "uncouple to balanced ground");
    }else if(bbswitch>mode_d && last_mode_atc==1 && tran_low<rc_in && rc_in<tran_high&&bbswitch<=mode_fly){
        attitude_control->set_target_mode(2);
        attitude_control->reset_yaw_target_and_rate();
        gcs().send_text(MAV_SEVERITY_CRITICAL, "ground to transition");
    }else if(bbswitch>mode_d && last_mode_atc==2 && rc_in>tran_high&&bbswitch<=mode_fly){
        attitude_control->set_target_mode(3);
        gcs().send_text(MAV_SEVERITY_CRITICAL, "transition to fly");
    }else if(bbswitch>mode_d && last_mode_atc==3 && rc_in<=tran_high&&bbswitch<=mode_fly){
        attitude_control->set_target_mode(2);
        gcs().send_text(MAV_SEVERITY_CRITICAL, "fly to transition");
    }else if(bbswitch>mode_d && last_mode_atc==2 && rc_in<=tran_low&&bbswitch<=mode_fly){
        attitude_control->set_target_mode(1);
        gcs().send_text(MAV_SEVERITY_CRITICAL, "transition to ground");
    }else if(bbswitch>mode_d && last_mode_atc==1 &&(last_roll_atc>5||last_roll_atc<-5)&&false&&bbswitch<=mode_fly)
    {
        attitude_control->set_target_mode(0);
        gcs().send_text(MAV_SEVERITY_CRITICAL, "emergency ground to uncoupled");
    }

    
    if(bbswitch>mode_fly)
    {
        attitude_control->set_target_mode(3);
        gcs().send_text(MAV_SEVERITY_CRITICAL, "fly tuning");
    }
    
    
    
    //read mode and do some things
    int now_mode=attitude_control->check_mode();
    if(now_mode==0){
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->reset_rate_controller_I_terms_gg();
        pilot_desired_throttle=0.15;
        // //this is the code for test log and draw figure
    if(abs(uncouple_pitch)<3000){
        if(uncouple_pitch<0){
            uncouple_pitch=0;
        }
        else{
            uncouple_pitch=0;
        }

    }else if(abs(uncouple_pitch)<6000){
        if(uncouple_pitch<0){
            uncouple_pitch=-3000;
        }
        else{
            uncouple_pitch=3000;
        }
    }else if(abs(uncouple_pitch)<8000){
        if(uncouple_pitch<0){
            uncouple_pitch=-6000;
        }
        else{
            uncouple_pitch=6000;
        }

    }else{
        if(uncouple_pitch<0){
            uncouple_pitch=-9000;
        }
        else{
            uncouple_pitch=9000;
        }
    }

    }else if(now_mode==1){
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->reset_rate_controller_I_terms_gw();
        if(rc_in>200){
            attitude_control->reset_rate_controller_I_terms_gg();
        }
        pilot_desired_throttle=0.15;
    }else if(now_mode==2){
        attitude_control->reset_rate_controller_I_terms_gw();

        pilot_desired_throttle=(rc_in-tran_low)/(tran_high-tran_low)*(tran_high-100)/1000+0.1;

        if(pilot_desired_throttle<hover_th){
            attitude_control->reset_yaw_target_and_rate();
        }
        attitude_control->reset_rate_controller_I_terms_gg();
        // else{
        //     attitude_control->set_target_mode(3);
        // }
    }else if (now_mode==3){
        attitude_control->reset_rate_controller_I_terms_gg();
        attitude_control->reset_rate_controller_I_terms_gw();
    }

    if (!motors->armed()) {
        // Motors should be Stopped
        motors->rc_in=0;
        motors->yaw_gw=0;
        motors->pitch_gw=0;

        attitude_control->reset_rate_controller_I_terms_gg();
        attitude_control->reset_rate_controller_I_terms_gw();
        attitude_control->reset_rate_controller_I_terms();
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    } else if (copter.ap.throttle_zero
               || (copter.air_mode == AirMode::AIRMODE_ENABLED && motors->get_spool_state() == AP_Motors::SpoolState::SHUT_DOWN)) {
        // throttle_zero is never true in air mode, but the motors should be allowed to go through ground idle
        // in order to facilitate the spoolup block

        // Attempting to Land
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    } else {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

    //gcs().send_text(MAV_SEVERITY_CRITICAL, "throttle_before %5.3f", (double)pilot_desired_throttle);

    switch (motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
        // Motors Stopped
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms();
        pilot_desired_throttle = 0.0f;
        attitude_control->_disturbance=0;
        break;

    case AP_Motors::SpoolState::GROUND_IDLE:
        // Landed
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms_smoothly();
        pilot_desired_throttle = 0.0f;
        break;

    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        // clear landing flag above zero throttle
        if (!motors->limit.throttle_lower) {
            set_land_complete(false);
        }
        break;

    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        // do nothing
        break;
    }

    // call attitude controller
    // attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
    //attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_pitch, target_roll, target_yaw_rate,balance_roll, balance_pitch, target_yaw_rate,uncouple_pitch, 0, 0);
    
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_pitch, target_roll, target_yaw_rate,balance_roll, 0, target_yaw_rate,uncouple_pitch, 0, 0);
    // output pilot's throttle
    //gcs().send_text(MAV_SEVERITY_CRITICAL, "throttle! %5.3f", (double)pilot_desired_throttle);
    attitude_control->set_throttle_out(pilot_desired_throttle, true, g.throttle_filt);
}
