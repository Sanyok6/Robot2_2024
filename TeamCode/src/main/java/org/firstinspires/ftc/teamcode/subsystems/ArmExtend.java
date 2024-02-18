package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class ArmExtend {
    DcMotorEx motor;
    int target;
    public ArmExtend(DcMotorEx motor) {
        this.motor = motor;
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
