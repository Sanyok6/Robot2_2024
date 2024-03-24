package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantFunction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.ArmExtend;
import org.firstinspires.ftc.teamcode.subsystems.ArmRotate;

public class ScoringMechanism {
    LinearOpMode opMode;
    StartingPosition startingPosition;

    public ArmRotate armRotate;
    ArmExtend armExtend;

    Servo clawRightServo;
    Servo clawLeftServo;
    Servo clawRollServo;
    ServoImplEx clawPitchServo;

    public ScoringMechanism(LinearOpMode opMode, StartingPosition startingPosition) {
        this.opMode = opMode;
        this.startingPosition = startingPosition;

        HardwareMap hardwareMap = opMode.hardwareMap;

        clawRightServo = hardwareMap.get(Servo.class, "clawRight");
        clawLeftServo = hardwareMap.get(Servo.class, "clawLeft");

        clawRollServo = hardwareMap.get(Servo.class, "clawRoll");

        clawPitchServo = hardwareMap.get(ServoImplEx.class, "clawPitch");
        clawPitchServo.setPwmRange(new PwmControl.PwmRange(2500, 500));

        armRotate = new ArmRotate(hardwareMap.get(DcMotorEx.class, "armRotate"));
        armExtend = new ArmExtend(hardwareMap.get(DcMotorEx.class, "linearSlide"));

        clawRightServo.setPosition(0);
        clawLeftServo.setPosition(1);

        clawRollServo.setPosition(0);
        clawPitchServo.setPosition(0.01);

        armRotate.setTarget(1200);
        armExtend.setCompensatedTarget(0, 1200);
    }

    public InstantFunction lowerOuttake = () -> {
        armRotate.setTarget(175);
        armExtend.setCompensatedTarget(0, 175);
        clawPitchServo.setPosition(0.5);
    };

    public InstantFunction placePurplePixel = () -> {
        movePurplePixelClaw(true);

        opMode.sleep(250);

        movePurplePixelClaw(false);
        clawPitchServo.setPosition(0);
    };

    public Action armToDriveMode = (t) -> {
        armRotate.setTarget(400);
        armExtend.setCompensatedTarget(0, 400);

        if (armRotate.getCurrentPosition() < 3000) {
            moveYellowPixelClaw(false);

            clawRollServo.setPosition(0);
            clawPitchServo.setPosition(0);

            return false;
        }

        return true;
    };

    private class PrepareToOuttakePixel implements Action {
        int target;
        boolean armRotated = false;
        boolean clawMoved = false;
        public PrepareToOuttakePixel(int target) {this.target = target;}
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!armRotated) {
                armRotate.setTarget(target);
                armExtend.setCompensatedTarget(0, target);
                armRotated = true;
            } else if (armRotate.getCurrentPosition() > 1000 && !clawMoved) {
                clawPitchServo.setPosition(0.86);
                clawRollServo.setPosition(0.55);
                clawMoved = true;
            } else return !armRotate.reachedTarget();
            return true;
        }
    }
    public Action prepareToOuttakeYellowPixel() {return new PrepareToOuttakePixel(4300);}
    public Action prepareToOuttakeWhitePixel() {return new PrepareToOuttakePixel(3600);}


    private class PlaceYellowPixel implements Action {
        ElapsedTime timer = new ElapsedTime();
        boolean started = false;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!started) {timer.reset(); started = true;}
            if (timer.milliseconds() > 1500) {
                return false;
            } else if (timer.milliseconds() > 1000) {
                moveYellowPixelClaw(true);
                armExtend.setCompensatedTarget(0, 4300, 0.5);
            } else {
                armExtend.setCompensatedTarget(300, armRotate.getCurrentPosition());
            }
            return true;
        }
    }
    public Action placeYellowPixel() {return new PlaceYellowPixel();}


    public InstantFunction prepareToIntakeWhitePixel = () -> {
        armRotate.setTarget(335);
        armExtend.setCompensatedTarget(0, 340);
        clawPitchServo.setPosition(0.55);
        clawLeftServo.setPosition(0.6);
        clawRightServo.setPosition(0.4);
    };

    public InstantFunction intakeWhitePixel = () -> {
        clawLeftServo.setPosition(1);
        clawRightServo.setPosition(0);
    };


    private void movePurplePixelClaw(boolean open) {
        if (startingPosition.color == StartingColor.RED) {
            clawRightServo.setPosition(open ? 0.15 : 0);
        } else {
            clawLeftServo.setPosition(open ? 0.85 : 1);
        }
    }

    private void moveYellowPixelClaw(boolean open) {
        if (startingPosition.color == StartingColor.RED) {
            clawLeftServo.setPosition(open ? 0.85 : 1);
        } else {
            clawRightServo.setPosition(open ? 0.15 : 0);
        }
    }

}
