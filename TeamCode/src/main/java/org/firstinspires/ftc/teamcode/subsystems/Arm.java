package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Arm {
    ArmMode mode;

    Servo clawRightServo;
    Servo clawLeftServo;
    ServoImplEx clawRollServo;
    ServoImplEx clawPitchServo;

    public ArmRotate armRotate;
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

        armRotate = new ArmRotate(hardwareMap.get(DcMotorEx.class, "armRotate"));
        armExtend = new ArmExtend(hardwareMap.get(DcMotorEx.class, "linearSlide"));

        mode = ArmMode.drive;
    }

    public void setMode(ArmMode mode) {
        this.mode = mode;
    }
    public void moveAlongBackdrop() {

    }

//    public void setRotationTarget(int target) {
//        armRotate.setTarget(target);
//    }
//
//    public void setExtensionTarget(int target) {
//        armExtend.setTarget(target);
//    }
//
//    public void closeClawLeft() {
//        clawLeftServo.setPosition(0);
//    }
//    public void openClawLeft() {
//        if (mode == ArmMode.intake) {
//            clawLeftServo.setPosition(0.8);
//        } else if (mode == ArmMode.outtake) {
//            clawLeftServo.setPosition(0.3);
//        }
//    }
//
//    public void closeClawRight() {
//        clawRightServo.setPosition(1);
//    }
//    public void openClawRight() {
//        if (mode == ArmMode.intake) {
//            clawLeftServo.setPosition(0.2);
//        } else if (mode == ArmMode.outtake) {
//            clawLeftServo.setPosition(0.7);
//        }
//    }

    void checkIfIntakeMode() {
        if (mode == ArmMode.intake && !clawLeftOpen && !clawRightOpen) {
            if (timeSinceClawClose.milliseconds() >= 500) {
                this.mode = ArmMode.drive;
            }
        } else if (clawLeftOpen || clawRightOpen) {
            if (mode == ArmMode.drive) {
                this.mode = ArmMode.intake;
            } else if (mode == ArmMode.intake) {
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
        double predictedExtension = 0.0574454233 * armRotate.target - 0.7430232633;
        armExtend.setTarget(target + (int) predictedExtension);
    }

    public void update() {
        if (mode == ArmMode.intake) {
            clawPitchServo.setPosition(0.55);
            clawRollServo.setPosition(0);
            clawRightServo.setPosition(clawRightOpen ? 0.4 : 0);
            clawLeftServo.setPosition(clawLeftOpen ? 0.6 : 1);

            setRotateTarget(100);
            setExtendTarget(300);
        } else if (mode == ArmMode.drive) {
            clawPitchServo.setPosition(0);
            clawRollServo.setPosition(0);

            clawRightServo.setPosition(0);
            clawLeftServo.setPosition(1);


            if (armExtend.currentPosition() - (int) (0.0574454233 * armRotate.currentPosition() - 0.7430232633) < 25) { setRotateTarget(50); }
            setExtendTarget(0);
        } else if (mode == ArmMode.outtake) {
            clawPitchServo.setPosition(0.8);
            clawRollServo.setPosition(0.55);

            clawRightServo.setPosition(clawRightOpen ? 0.2 : 0);
            clawLeftServo.setPosition(clawLeftOpen ? 0.8 : 1);


            setRotateTarget(1800);
            setExtendTarget(0);
        }
    }

}
