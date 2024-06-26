package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Arm {
    public ArmMode mode;

    Servo clawRightServo;
    Servo clawLeftServo;
    ServoImplEx clawRollServo;
    ServoImplEx clawPitchServo;

    Servo hangServo;

    public ArmRotate armRotate;
    public ArmExtend armExtend;

    Boolean clawLeftOpen = false;
    Boolean clawRightOpen = false;

    ElapsedTime timeSinceClawClose = new ElapsedTime();

    Boolean outtakePositionMoved = false;
    double currentHeight;
    int targetRotation;
    int targetExtension;
    public double targetPitchPosition;

    public Arm(HardwareMap hardwareMap) {
        clawRightServo = hardwareMap.get(Servo.class, "clawRight");
        clawLeftServo = hardwareMap.get(Servo.class, "clawLeft");

        clawRollServo = hardwareMap.get(ServoImplEx.class, "clawRoll");

        clawPitchServo = hardwareMap.get(ServoImplEx.class, "clawPitch");
        clawPitchServo.setPwmRange(new PwmControl.PwmRange(2500, 500));

        hangServo = hardwareMap.servo.get("hang");
        hangServo.setPosition(1);

        armRotate = new ArmRotate(hardwareMap.get(DcMotorEx.class, "armRotate"));
        armExtend = new ArmExtend(hardwareMap.get(DcMotorEx.class, "linearSlide"));

        mode = ArmMode.DRIVE;
    }

    public void setMode(ArmMode mode) {
        this.mode = mode;
    }
    public void armKinematics(double x, double y, double theta) {
        //https://www.chiefdelphi.com/t/inverse-kinematics-for-a-telescoping-arm/426258

        theta = Math.toRadians(theta);

        //constant
        double r = 14;

        //outputs
        double L = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2) + Math.pow(r, 2) - 2*r*x*Math.cos(theta) - 2*r*y*Math.sin(theta));
        double phi_1 = Math.atan2(y - r*Math.sin(theta), x - r*Math.cos(theta));
        double phi_2 = theta - phi_1;

        phi_1 = Math.toDegrees(phi_1);
        phi_2 = Math.toDegrees(phi_2);

        // calculate linear slide encoder position from target extension length
        double extensionTarget = 18.32 * L - 544.31;

        // calculate arm rotation encoder position from angle
        double rotationTarget = -24.3 * phi_1 + 4546;

        // calculate pitch servo position from angle
        double clawTarget = 0.00555 * phi_2 + 0.616;

        this.targetRotation = (int) rotationTarget;
        this.targetExtension = (int) extensionTarget;
        this.targetPitchPosition = clawTarget;
    }

    public void moveAlongBackdrop(double speed) {
        outtakePositionMoved = true;
        double newHeight = currentHeight + speed;
        if (newHeight >= 20 && newHeight <= 60) {
            currentHeight = newHeight;
            armKinematics(38, currentHeight, 60);
        }
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
        armExtend.setCompensatedTarget(target, (int) armRotate.getCurrentPosition());
    }

    public void resetRotateEncoder(){
        clawPitchServo.setPosition(0.3);
        setExtendTarget(0);
        armRotate.motor.setPower(0);
        armRotate.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRotate.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void update() {
        if (mode == ArmMode.INTAKE) {
            clawPitchServo.setPosition(0.51);
            clawRollServo.setPosition(0.1);
            clawRightServo.setPosition(clawRightOpen ? 0.18 : 0.91);
            clawLeftServo.setPosition(clawLeftOpen ? 0.87 : 0.04);

            setRotateTarget(220);
            setExtendTarget(300);
        } else if (mode == ArmMode.DRIVE) {

            if (armRotate.getCurrentPosition() < 3000) {
                clawPitchServo.setPosition(0);
                clawRollServo.setPosition(0.1);
            }

            clawRightServo.setPosition(0.91);
            clawLeftServo.setPosition(0.04);

            setRotateTarget(300);
            setExtendTarget(0);
        } else if (mode == ArmMode.OUTTAKE) {

            if (!outtakePositionMoved) {
                armKinematics(38, 20, 60);
                currentHeight = 20;
            }

            if (armRotate.getCurrentPosition() > 1000) {
                clawRollServo.setPosition(0.73);
                clawPitchServo.setPosition(targetPitchPosition);
            }

            clawRightServo.setPosition(clawRightOpen ? 0.18 : 0.91);
            clawLeftServo.setPosition(clawLeftOpen ? 0.87 : 0.04);

            setRotateTarget(targetRotation);
            setExtendTarget(targetExtension);

        } else if (mode == ArmMode.VERTICAL) {
            setRotateTarget(2500);
            setExtendTarget(200);

            clawPitchServo.setPosition(1);
            clawRollServo.setPosition(0.1);
            hangServo.setPosition(0.7);
        } else if (mode == ArmMode.HANG) {
            armRotate.setTarget(1000);
            armRotate.fastMoveToTarget();
            setExtendTarget(0);

            clawPitchServo.setPosition(1);
            clawRollServo.setPosition(0.1);
        }

        if (mode != ArmMode.HANG) {
            armRotate.moveTowardsTarget();
        }
    }

}
