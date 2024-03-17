package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ArmRotate {
    public DcMotor motor;

    public double target = 0;

    // constants for PIDf
    public double Kp = 0.001;
    public double f = 0.2;

    //constants for motion profiling
    public double max_acceleration = 0.01;
    public double max_velocity = 6;

    // variables for motion profiling
    double distance = 0;
    double startingPosition = 0;

    ElapsedTime timer = new ElapsedTime();

    public ArmRotate(DcMotor motor) {
        this.motor = motor;
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setTarget(int target) {
        if (this.target != target) {
            startingPosition = getCurrentPosition();
            this.target = target;
            timer.reset();
            distance = target - getCurrentPosition();
        }
    }

    public void moveTowardsTarget() {
        double instantTargetPosition = startingPosition + (distance > 1 ? 1 : -1) * motion_profile(max_acceleration, max_velocity, distance, timer.milliseconds());
        double ff = Math.cos(-0.041152263 * instantTargetPosition - 4546) * f;
        double motorPower = (instantTargetPosition - getCurrentPosition()) * Kp + ff;

        motor.setPower(motorPower);
    }

    public void fastMoveToTarget() {
        double instantTargetPosition = startingPosition + (distance > 1 ? 1 : -1) * motion_profile(10, 10, distance, timer.milliseconds());
        double motorPower = (instantTargetPosition - getCurrentPosition()) * 0.01;

        motor.setPower(motorPower);
    }

    public Boolean reachedTarget() {
        return Math.abs(target-getCurrentPosition()) < 50;
    }

    public int getCurrentPosition() {
        return -motor.getCurrentPosition();
    }

    double motion_profile(double max_acceleration, double max_velocity, double distance, double elapsed_time) {

        distance = Math.abs(distance);

        // Calculate the time it takes to accelerate to max velocity
        double acceleration_dt = max_velocity / max_acceleration;

        // If we can't accelerate to max velocity in the given distance, we'll accelerate as much as possible
        double halfway_distance = distance / 2;
        double acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);

        if (acceleration_distance > halfway_distance) {
            acceleration_dt = Math.sqrt(halfway_distance / (0.5 * max_acceleration));
        }

        acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);

        // recalculate max velocity based on the time we have to accelerate and decelerate
        max_velocity = max_acceleration * acceleration_dt;

        // we decelerate at the same rate as we accelerate
        double deceleration_dt = acceleration_dt;

        // calculate the time that we're at max velocity
        double cruise_distance = distance - 2 * acceleration_distance;
        double cruise_dt = cruise_distance / max_velocity;
        double deceleration_time = acceleration_dt + cruise_dt;

        // check if we're still in the motion profile
        double entire_dt = acceleration_dt + cruise_dt + deceleration_dt;
        if (elapsed_time > entire_dt) {
            return distance;
        }

        // if we're accelerating
        if (elapsed_time < acceleration_dt) {
            // use the kinematic equation for acceleration
            return 0.5 * max_acceleration * Math.pow(elapsed_time, 2);
        }

        // if we're cruising
        else if (elapsed_time < deceleration_time) {
            acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);
            double cruise_current_dt = elapsed_time - acceleration_dt;

            // use the kinematic equation for constant velocity
            return acceleration_distance + max_velocity * cruise_current_dt;
        }

        // if we're decelerating
        else {
            acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);
            cruise_distance = max_velocity * cruise_dt;
            deceleration_time = elapsed_time - deceleration_time;

            // use the kinematic equations to calculate the instantaneous desired position
            return acceleration_distance + cruise_distance + max_velocity * deceleration_time - 0.5 * max_acceleration * Math.pow(deceleration_time, 2);
        }
    }

}

