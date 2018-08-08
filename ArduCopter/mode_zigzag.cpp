#include "Copter.h"

/*
* Init and run calls for zigzag flight mode
*/

#define ZIGZAG_RANGE_AS_STATIC  3

// zigzag_init - initialise zigzag controller
bool Copter::ModeZigzag::init(bool ignore_checks)
{
    if (copter.position_ok() || ignore_checks) {
        // initialize's loiter position and velocity on xy-axes from current pos and velocity
        loiter_nav->init_target();

        // initialize vertical speed and acceleration's range
        pos_control->set_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
        // the parameters are maximum climb and descent rates
        pos_control->set_accel_z(g.pilot_accel_z);

        // initialise position_z and desired velocity_z
        if (!pos_control->is_active_z()) {
            // is_active_z - returns true if the z-axis position controller has been run very recently
            pos_control->set_alt_target_to_current_alt();
            pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
        }

        // initialise waypoint state
        zigzag_waypoint_state.A_hasbeen_defined = false;
        zigzag_waypoint_state.B_hasbeen_defined = false;
        in_zigzag_manual_control = true;
        zigzag_is_between_A_and_B = false;
        zigzag_judge_moving.is_keeping_time = false;

        return true;
    }
    else {
        return false;
    }
}

// zigzag_run - runs the zigzag controller
// should be called at 100hz or more
void Copter::ModeZigzag::run()
{
    // if not auto armed or motors not enabled set throttle to zero and exit immediately
    if (!motors->armed() || !ap.auto_armed || !motors->get_interlock() || ap.land_complete) {
        zero_throttle_and_relax_ac();
        return;
    }

    // if B has not been defined
    if (!zigzag_waypoint_state.B_hasbeen_defined) {
        // receive pilot's inputs, do position and attitude control
        zigzag_manual_control();
    }

    //if it's in manual control part
    //receive pilot's inputs, do position and attitude control
    //else
    //judge if the plane has arrived at the current destination
    //if yes, go to the manual control part by modifying parameter
    //else, fly to current destination
    else {
        if (in_zigzag_manual_control) {
            zigzag_manual_control();
        }else { //auto flight

            if (zigzag_has_arr_at_dest()) {  //if the vehicle has arrived at the current destination
                in_zigzag_manual_control = true;
                loiter_nav->init_target();
            }else {
                zigzag_auto_control();
            }
        }
    }
}

//zigzag_auto_control - guide the plane to fly to current destination
void Copter::ModeZigzag::zigzag_auto_control()
{
    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!copter.failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
        }
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // run waypoint controller to update xy
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control->update_z_controller();

    // call attitude controller
    if (auto_yaw.mode() == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), target_yaw_rate);
    }
    else if (auto_yaw.mode() == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from mavlink command or mission item
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), auto_yaw.rate_cds());
    }
    else {
        // roll, pitch from waypoint controller, yaw heading from GCS or auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), auto_yaw.yaw(), true);
    }
}

//zigzag_manual_control - process manual control
void Copter::ModeZigzag::zigzag_manual_control()
{
    float target_yaw_rate = 0.0f;
    float target_climb_rate = 0.0f;

    // initialize vertical speed and acceleration's range
    pos_control->set_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control->set_accel_z(g.pilot_accel_z);
    // process pilot inputs unless we are in radio failsafe
    if (!copter.failsafe.radio) {
        // apply SIMPLE mode transform to pilot inputs
        update_simple_mode();
        // process pilot's roll and pitch input
        loiter_nav->set_pilot_desired_acceleration(channel_roll->get_control_in(), channel_pitch->get_control_in(), G_Dt);
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

        // get pilot desired climb rate
        target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
        //make sure the climb rate is in the given range, prevent floating point errors
        target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
    }
    else {
        // clear out pilot desired acceleration in case radio failsafe event occurs and we
        //do not switch to RTL for some reason
        loiter_nav->clear_pilot_desired_acceleration();
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
    // run loiter controller
    loiter_nav->update(ekfGndSpdLimit, ekfNavVelGainScaler);

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(loiter_nav->get_roll(),
            loiter_nav->get_pitch(), target_yaw_rate);

    // adjust climb rate using rangefinder
    if (copter.rangefinder_alt_ok()) {
        // if rangefinder is ok, use surface tracking
        target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control->get_alt_target(), G_Dt);
    }

    // get avoidance adjusted climb rate
    target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

    // update altitude target and call position controller
    pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
    //adjusts target up or down using a climb rate

    pos_control->update_z_controller();
}

//zigzag_has_arr_at_next_dest - judge if the plane is within a small area around the current destination
bool Copter::ModeZigzag::zigzag_has_arr_at_dest()
{
    if (!zigzag_judge_moving.is_keeping_time) {
        zigzag_judge_moving.is_keeping_time = true;
        zigzag_judge_moving.last_judge_pos_time = AP_HAL::millis();
        zigzag_judge_moving.last_pos = inertial_nav.get_position();
    }
    else {
        if ((AP_HAL::millis() - zigzag_judge_moving.last_judge_pos_time) > 1000) {
            Vector3f cur_pos = inertial_nav.get_position();
            float dist_x = cur_pos.x - zigzag_judge_moving.last_pos.x;
            float dist_y = cur_pos.y - zigzag_judge_moving.last_pos.y;
            if ((dist_x*dist_x + dist_y*dist_y) < (ZIGZAG_RANGE_AS_STATIC * ZIGZAG_RANGE_AS_STATIC)) {
                return true;
            }
            else {
                zigzag_judge_moving.last_judge_pos_time = AP_HAL::millis();
                zigzag_judge_moving.last_pos = inertial_nav.get_position();
            }
        }
    }

    return false;
}

// zigzag_calculate_next_dest - calculate next destination according to vector A-B and current position
void Copter::ModeZigzag::zigzag_calculate_next_dest(Vector3f& next_dest, uint8_t next_A_or_B) const
{
    // calculate difference between A and B - vector AB and its direction
    Vector3f pos_diff = zigzag_waypoint_state.B_pos - zigzag_waypoint_state.A_pos;
    // get current position
    Vector3f cur_pos = inertial_nav.get_position();
    if (!zigzag_is_between_A_and_B) {
        // if the drone's position is on the side of A or B
        if (next_A_or_B == 0) {
            next_dest = cur_pos + pos_diff;
        }
        if (next_A_or_B == 2) {
            next_dest = cur_pos + pos_diff*(-1);
        }
    }
    else {
        // used to check if the drone is outside A-B scale
        int next_dir = 1;
        // if the drone's position is between A and B
        float xa = zigzag_waypoint_state.A_pos.x;
        float ya = zigzag_waypoint_state.A_pos.y;
        float xb = zigzag_waypoint_state.B_pos.x;
        float yb = zigzag_waypoint_state.B_pos.y;
        float xc = cur_pos.x;
        float yc = cur_pos.y;
        next_dest.z = cur_pos.z;
        if (next_A_or_B == 0) {
            // calculate next B
            Vector3f pos_diff_BC = cur_pos - zigzag_waypoint_state.B_pos;
            if ((pos_diff_BC.x*pos_diff.x + pos_diff_BC.y*pos_diff.y) > 0) {
                next_dir = -1;
            }
            float dis_from_AB = fabsf(xc*(yb - ya) / (xb - xa) - yc + (xb*ya - xa*yb) / (xb - xa)) / sqrtf(1 + ((yb - ya)*(yb - ya)) / ((xb - xa)*(xb - xa)));
            float dis_CB = sqrtf((xc - xb)*(xc - xb) + (yc - yb)*(yc - yb));
            float dis_BE = sqrtf(dis_CB*dis_CB - dis_from_AB*dis_from_AB);
            float dis_ratio = dis_BE / sqrtf((xa - xb)*(xa - xb) + (ya - yb)*(ya - yb));
            next_dest.x = cur_pos.x + next_dir*dis_ratio*pos_diff.x;
            next_dest.y = cur_pos.y + next_dir*dis_ratio*pos_diff.y;
        }
        if (next_A_or_B == 2) {
            // calculate next A
            Vector3f pos_diff_AC = cur_pos - zigzag_waypoint_state.A_pos;
            if ((pos_diff_AC.x*pos_diff.x + pos_diff_AC.y*pos_diff.y) < 0) {
                next_dir = -1;
            }
            float dis_from_AB = fabsf(xc*(yb - ya) / (xb - xa) - yc + (xb*ya - xa*yb) / (xb - xa)) / sqrtf(1 + ((yb - ya)*(yb - ya)) / ((xb - xa)*(xb - xa)));
            float dis_CA = sqrtf((xc - xa)*(xc - xa) + (yc - ya)*(yc - ya));
            float dis_AE = sqrtf(dis_CA*dis_CA - dis_from_AB*dis_from_AB);
            float dis_ratio = dis_AE / sqrtf((xa - xb)*(xa - xb) + (ya - yb)*(ya - yb));
            next_dest.x = cur_pos.x + (-1)*next_dir*dis_ratio*pos_diff.x;
            next_dest.y = cur_pos.y + (-1)*next_dir*dis_ratio*pos_diff.y;
        }
    }
}

// called by AUXSW_ZIGZAG_ENABLE case in switches.cpp
// used to record point A, B and give the signal to fly to next destination automatically
void Copter::ModeZigzag::zigzag_receive_signal_from_auxsw(RC_Channel::aux_switch_pos_t aux_switch_position)
{
    // define point A and B
    if (!zigzag_waypoint_state.A_hasbeen_defined || !zigzag_waypoint_state.B_hasbeen_defined) {
        if ((!zigzag_waypoint_state.A_hasbeen_defined && aux_switch_position == RC_Channel::aux_switch_pos_t::HIGH) || (!zigzag_waypoint_state.B_hasbeen_defined && aux_switch_position == RC_Channel::aux_switch_pos_t::LOW)) {
            Vector3f cur_pos = inertial_nav.get_position();
            zigzag_set_destination(cur_pos);
            return;
        }
    }
    else {
        // LOW = 0
        // MIDDLE = 1
        // HIGH = 2
        if ((aux_switch_position == RC_Channel::aux_switch_pos_t::HIGH) || (aux_switch_position == RC_Channel::aux_switch_pos_t::LOW)) {
            // calculate next point A or B
            // need to judge if the drone's position is between A and B
            Vector3f next_dest;
            zigzag_calculate_next_dest(next_dest, aux_switch_position);
            // initialise waypoint and spline controller
            wp_nav->wp_and_spline_init();
            zigzag_set_destination(next_dest);
            // initialise yaw
            auto_yaw.set_mode_to_default(false);
            in_zigzag_manual_control = false;
        }
        else {
            //regain the control
            if (!in_zigzag_manual_control) {
                in_zigzag_manual_control = true;
                loiter_nav->init_target();
                zigzag_is_between_A_and_B = true;
            }
            else {
                zigzag_is_between_A_and_B = false;
            }
        }
    }
}

// zigzag_set_destination - sets zigzag mode's target destination
// Returns true if the fence is enabled and guided waypoint is within the fence
// else return false if the waypoint is outside the fence
bool Copter::ModeZigzag::zigzag_set_destination(const Vector3f& destination)
{

#if AC_FENCE == ENABLED
    // reject destination if outside the fence
    Location_Class dest_loc(destination);
    if (!copter.fence.check_destination_within_fence(dest_loc)) {
        copter.Log_Write_Error(ERROR_SUBSYSTEM_NAVIGATION, ERROR_CODE_DEST_OUTSIDE_FENCE);
        // failure is propagated to GCS with NAK
        return false;
    }
#endif

    //define point A
    if (!zigzag_waypoint_state.A_hasbeen_defined) {
        zigzag_waypoint_state.A_pos = destination;
        zigzag_waypoint_state.A_hasbeen_defined = true;
        return true;
    }
    //define point B
    else if (!zigzag_waypoint_state.B_hasbeen_defined) {
        zigzag_waypoint_state.B_pos = destination;
        zigzag_waypoint_state.B_hasbeen_defined = true;
        return true;
    }

    // no need to check return status because terrain data is not used
    wp_nav->set_wp_destination(destination, false);
    return true;
}
