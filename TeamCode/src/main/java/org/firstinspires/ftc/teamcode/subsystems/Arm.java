package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Mat;

public class Arm {
    ArmMode mode;

    Servo clawRightServo;
    Servo clawLeftServo;
    ServoImplEx clawRollServo;
    ServoImplEx clawPitchServo;

    public ArmRotatePID armRotate;
    public ArmExtend armExtend;

    Boolean clawLeftOpen = false;
    Boolean clawRightOpen = false;

    ElapsedTime timeSinceClawClose = new ElapsedTime();

    public Arm(HardwareMap hardwareMap) {
        clawRightServo = hardwareMap.get(Servo.class, "clawRight");
        clawLeftServo = hardwareMap.get(Servo.class, "clawLeft");

        clawRollServo = hardwareMap.get(ServoImplEx.class, "clawRoll");

        clawPitchServo = hardwareMap.get(ServoImplEx.class, "clawPitch");
        clawPitchServo.setPwmRange(new PwmControl.PwmRange(2500, 500));

        armRotate = new ArmRotatePID(hardwareMap.get(DcMotorEx.class, "armRotate"), hardwareMap.get(DcMotorEx.class, "armRotateEncoder"));
        armExtend = new ArmExtend(hardwareMap.get(DcMotorEx.class, "linearSlide"));

        mode = ArmMode.DRIVE;
    }

    public void setMode(ArmMode mode) {
        this.mode = mode;
    }
    public void moveAlongBackdrop() {

    }

    void checkIfIntakeMode() {
        if (mode == ArmMode.INTAKE && !clawLeftOpen && !clawRightOpen) {
            if (timeSinceClawClose.milliseconds() >= 500) {
                this.mode = ArmMode.DRIVE;
            }
        } else if (clawLeftOpen || clawRightOpen) {
            if (mode == ArmMode.DRIVE) {
                this.mode = ArmMode.INTAKE;
            } else if (mode == ArmMode.INTAKE) {
                timeSinceClawClose.reset();
            }
        }
    }
    public void setClawLeftOpen(boolean state) {
        clawLeftOpen = state;
        checkIfIntakeMode();
    }

    public void setClawRightOpen(boolean state) {
        clawRightOpen = state;
        checkIfIntakeMode();
    }

    void setRotateTarget(int target) {
        armRotate.setTarget(target);
    }
    void setExtendTarget(int target) {
        armExtend.setCompensatedTarget(target, (int) armRotate.target);
    }

    public void update() {
        if (mode == ArmMode.INTAKE) {
            clawPitchServo.setPosition(0.52);
            clawRollServo.setPosition(0);
            clawRightServo.setPosition(clawRightOpen ? 0.4 : 0);
            clawLeftServo.setPosition(clawLeftOpen ? 0.6 : 1);

            setRotateTarget(300);
            setExtendTarget(300);
        } else if (mode == ArmMode.DRIVE) {
            clawPitchServo.setPosition(0);
            clawRollServo.setPosition(0);

            clawRightServo.setPosition(0);
            clawLeftServo.setPosition(1);

            setRotateTarget(300);
            setExtendTarget(0);
        } else if (mode == ArmMode.OUTTAKE) {
            clawPitchServo.setPosition(0.8);
            clawRollServo.setPosition(0.55);

            clawRightServo.setPosition(clawRightOpen ? 0.2 : 0);
            clawLeftServo.setPosition(clawLeftOpen ? 0.8 : 1);


            setRotateTarget(4000);
            setExtendTarget(0);
        } else if (mode == ArmMode.VERTICAL) {
            setRotateTarget(1200);
            setExtendTarget(0);

            clawPitchServo.setPosition(1);
            clawRollServo.setPosition(0);
        } else if (mode == ArmMode.HANG) {
            setRotateTarget(700);
            setExtendTarget(0);
        }

        armRotate.moveTowardsTarget();
    }

}
