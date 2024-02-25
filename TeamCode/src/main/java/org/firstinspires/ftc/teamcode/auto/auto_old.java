package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ArmExtend;
import org.firstinspires.ftc.teamcode.subsystems.ArmRotate;

public class auto_old {
    public auto_old(LinearOpMode opMode, StartingPosition startingPosition) {
        HardwareMap hardwareMap = opMode.hardwareMap;

        Servo clawRightServo = hardwareMap.get(Servo.class, "clawRight");
        Servo clawLeftServo = hardwareMap.get(Servo.class, "clawLeft");

        Servo clawRollServo = hardwareMap.get(Servo.class, "clawRoll");

        ServoImplEx clawPitchServo = hardwareMap.get(ServoImplEx.class, "clawPitch");
        clawPitchServo.setPwmRange(new PwmControl.PwmRange(2500, 500));

        ArmRotate armRotate = new ArmRotate(hardwareMap.get(DcMotorEx.class, "armRotate"));
        ArmExtend armExtend = new ArmExtend(hardwareMap.get(DcMotorEx.class, "linearSlide"));

        clawRightServo.setPosition(0);
        clawLeftServo.setPosition(1);

        clawRollServo.setPosition(0);
        clawPitchServo.setPosition(0.01);

        armRotate.setTarget(150);
        armExtend.setTarget(0 + (int) (0.0574454233 * 150 - 0.7430232633));

        Trajectories trajectories = new Trajectories(hardwareMap, startingPosition);

        opMode.waitForStart();

        Actions.runBlocking(
                new SequentialAction(
//                                trajectories.toTeamPropOddPositions(),
                        trajectories.toTeamPropPos2Pt1(),
                        telemetryPacket -> {

                            // pos 3
                            //armExtend.setTarget(860);
                            //armRotate.setTarget(300);
                            //opMode.sleep(1000);

                            armRotate.setTarget(250);
                            armExtend.setTarget(500 + (int) (0.0574454233 * 150 - 0.7430232633));

                            clawPitchServo.setPosition(0.5);
                            opMode.sleep(500);

                            // Yellow pixel closer to backdrop when aligning

                            if (startingPosition.color == StartingColor.RED) {
                                clawRightServo.setPosition(0.1);
                            } else {
                                clawLeftServo.setPosition(0.9);
                            }

                            opMode.sleep(500);
                            clawPitchServo.setPosition(0);


                            if (startingPosition.color == StartingColor.RED) {
                                clawRightServo.setPosition(0);
                            } else {
                                clawLeftServo.setPosition(1);
                            }

                            armExtend.setTarget(0);
                            opMode.sleep(1000);

                            clawPitchServo.setPosition(0.8);
                            clawRollServo.setPosition(0.55);

                            armRotate.setTarget(1800);
                            armRotate.setTarget(1800 + (int) (0.0574454233 * 150 - 0.7430232633));

                            opMode.sleep(1000);

                            return false;
                        },
                        trajectories.toTeamPropPos2Pt2(),
                        trajectories.alignToBackdrop(),
                        telemetryPacket -> {
                            opMode.sleep(500);

                            if (startingPosition.color == StartingColor.RED) {
                                clawLeftServo.setPosition(0.9);
                            } else {
                                clawRightServo.setPosition(0.1);
                            }

                            opMode.sleep(500);
                            return false;
                        },
                        trajectories.awayFromBackdrop(),
                        telemetryPacket -> {
                            armRotate.setTarget(300);
                            armRotate.setTarget(300 + (int) (0.0574454233 * 150 - 0.7430232633));

                            opMode.sleep(500);

                            clawPitchServo.setPosition(0);
                            clawRollServo.setPosition(0);

                            if (startingPosition.color == StartingColor.RED) {
                                clawLeftServo.setPosition(1);
                            } else {
                                clawRightServo.setPosition(0);
                            }

                            armRotate.setTarget(300);
                            armRotate.setTarget(300 + (int) (0.0574454233 * 150 - 0.7430232633));

                            opMode.sleep(1000);
                            return false;
                        }

                )
        );
    }

}
