package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantFunction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.subsystems.ArmExtend;
import org.firstinspires.ftc.teamcode.subsystems.ArmRotatePID;

public class ScoringMechanism {
    LinearOpMode opMode;
    StartingPosition startingPosition;

    Servo clawRightServo;
    Servo clawLeftServo;
    Servo clawRollServo;
    ServoImplEx clawPitchServo;
    ArmRotatePID armRotate;
    ArmExtend armExtend;

    public ScoringMechanism(LinearOpMode opMode, StartingPosition startingPosition) {
        this.opMode = opMode;
        this.startingPosition = startingPosition;

        HardwareMap hardwareMap = opMode.hardwareMap;

        clawRightServo = hardwareMap.get(Servo.class, "clawRight");
        clawLeftServo = hardwareMap.get(Servo.class, "clawLeft");

        clawRollServo = hardwareMap.get(Servo.class, "clawRoll");

        clawPitchServo = hardwareMap.get(ServoImplEx.class, "clawPitch");
        clawPitchServo.setPwmRange(new PwmControl.PwmRange(2500, 500));

        armRotate = new ArmRotatePID(hardwareMap.get(DcMotorEx.class, "armRotate"), hardwareMap.get(DcMotorEx.class, "armRotateEncoder"));
        armExtend = new ArmExtend(hardwareMap.get(DcMotorEx.class, "linearSlide"));

        clawRightServo.setPosition(0);
        clawLeftServo.setPosition(1);

        clawRollServo.setPosition(0);
        clawPitchServo.setPosition(0.01);

        armRotate.setTarget(150);
        armExtend.setTarget(0 + (int) (0.0574454233 * 150 - 0.7430232633));
    }

    public InstantFunction lowerOuttake = () -> {
        clawPitchServo.setPosition(0.5);
    };

    public InstantFunction placePurplePixel = () -> {
        movePurplePixelClaw(true);

        opMode.sleep(500);

        movePurplePixelClaw(false);
        clawPitchServo.setPosition(0);
    };

    public InstantFunction prepareToOuttakeYellowPixel = () -> {
        clawPitchServo.setPosition(0.86);
        clawRollServo.setPosition(0.55);

        armRotate.setTarget(2000);
        armExtend.setTarget(0 + (int) (0.0574454233 * 2000 - 0.7430232633));
    };

    public InstantFunction releaseYellowPixel = () -> moveYellowPixelClaw(true);

    public Action armToDriveMode = (t) -> {
        armRotate.setTarget(400);
        armExtend.setTarget(0);

        if (armRotate.getCurrentPosition() < 1500) {
            moveYellowPixelClaw(false);

            clawRollServo.setPosition(0);
            clawPitchServo.setPosition(0);

            return false;
        }

        return true;
    };

    public InstantFunction test = () -> {
        clawPitchServo.setPosition(0.86);
        clawRollServo.setPosition(0.55);

        armRotate.setTarget(2000);
        armExtend.setTarget(0 + (int) (0.0574454233 * 2000 - 0.7430232633));

        opMode.sleep(1000);

        armExtend.setTarget(325 + (int) (0.0574454233 * 2000 - 0.7430232633), 1);

        opMode.sleep(500);

        moveYellowPixelClaw(true);

        armExtend.setTarget(0 + (int) (0.0574454233 * 2000 - 0.7430232633), 0.5);

        opMode.sleep(1000);

    };

    private void movePurplePixelClaw(boolean open) {
        if (startingPosition.color == StartingColor.RED) {
            clawRightServo.setPosition(open ? 0.1 : 0);
        } else {
            clawLeftServo.setPosition(open ? 0.9 : 1);
        }
    }

    private void moveYellowPixelClaw(boolean open) {
        if (startingPosition.color == StartingColor.RED) {
            clawLeftServo.setPosition(open ? 0.95 : 1);
        } else {
            clawRightServo.setPosition(open ? 0.05 : 0);
        }
    }

}
