package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class ArmRotate {
    DcMotorEx motor;
    int target;
    public ArmRotate(DcMotorEx motor) {
        this.motor = motor;
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setTarget(int target) {
        this.target = target;

        motor.setTargetPosition(target);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(1);
    }

    public int currentPosition() {
        return motor.getCurrentPosition();
    }
    public Boolean reachedTarget() {
        return Math.abs(target-currentPosition()) < 20;
    }

}
