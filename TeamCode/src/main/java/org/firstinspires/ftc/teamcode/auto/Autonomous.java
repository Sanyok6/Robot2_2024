package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
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

public class Autonomous {
        StartingPosition startingPosition;

        public Autonomous(LinearOpMode opMode, StartingPosition startingPosition) {
                HardwareMap hardwareMap = opMode.hardwareMap;
                this.startingPosition = startingPosition;

                MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(12, yCoordinate(-60), angle(270)));

                ScoringMechanism scoringMechanism = new ScoringMechanism(opMode, startingPosition);

                Action leftPosTraj = drive.actionBuilder(new Pose2d(12, yCoordinate(-60), angle(270)))
                                .afterTime(1, scoringMechanism.lowerOuttake)
                                .strafeToLinearHeading(new Vector2d(34, yCoordinate(-28)), angle(180))
                                .stopAndAdd(scoringMechanism.placePurplePixel)
                                .afterTime(0, scoringMechanism.prepareToOuttakeYellowPixel)
                                .strafeTo(new Vector2d(38, yCoordinate(-30)))
                                .strafeTo(new Vector2d(44, yCoordinate(-30)), null, new ProfileAccelConstraint(-5, 50))
                                .stopAndAdd(scoringMechanism.releaseYellowPixel)
                                .waitSeconds(0.5)
                                .strafeTo(new Vector2d(40, yCoordinate(-30)), null, new ProfileAccelConstraint(-5, 50))
                                .stopAndAdd(scoringMechanism.armToDriveMode)
                                .strafeTo(new Vector2d(45, yCoordinate(-30)))
                                .build();

                Action betterLeftPosTraj = drive.actionBuilder(new Pose2d(12, yCoordinate(-60), angle(270)))
                        .afterTime(1, scoringMechanism.lowerOuttake)
                        .strafeToLinearHeading(new Vector2d(34, yCoordinate(-28)), angle(180))
                        .stopAndAdd(scoringMechanism.placePurplePixel)
                        .strafeTo(new Vector2d(38, yCoordinate(-36)))
                        .stopAndAdd(scoringMechanism.test)
                        .build();

                Action middlePosTraj = drive.actionBuilder(new Pose2d(12, yCoordinate(-60), angle(270)))
                        .afterTime(1, scoringMechanism.lowerOuttake)
                        .strafeToLinearHeading(new Vector2d(25, yCoordinate(-21)), angle(180))
                        .stopAndAdd(scoringMechanism.placePurplePixel)
                        .strafeTo(new Vector2d(40, yCoordinate(-30)))
                        .strafeTo(new Vector2d(45, yCoordinate(-30)), null, new ProfileAccelConstraint(-5, 50))
                        .build();

                Action betterMiddlePosTraj = drive.actionBuilder(new Pose2d(12, yCoordinate(-60), angle(270)))
                        .afterTime(1, scoringMechanism.lowerOuttake)
                        .strafeToLinearHeading(new Vector2d(25, yCoordinate(-21)), angle(180))
                        .stopAndAdd(scoringMechanism.placePurplePixel)
                        .strafeTo(new Vector2d(38, yCoordinate(-30)))
                        .stopAndAdd(scoringMechanism.test)
                        .build();

                Action rightPosTraj = drive.actionBuilder(new Pose2d(12, yCoordinate(-60), angle(270)))
                        .afterTime(1.5, scoringMechanism.lowerOuttake)
                        .strafeToLinearHeading(new Vector2d(15, yCoordinate(-40)), angle(180))
                        .strafeTo(new Vector2d(10, yCoordinate(-30)))
                        .stopAndAdd(scoringMechanism.placePurplePixel)
                        .strafeTo(new Vector2d(40, yCoordinate(-30)))
                        .strafeTo(new Vector2d(45, yCoordinate(-30)), null, new ProfileAccelConstraint(-5, 50))
                        .build();

            Action betterRightPosTraj = drive.actionBuilder(new Pose2d(12, yCoordinate(-60), angle(270)))
                    .afterTime(1.5, scoringMechanism.lowerOuttake)
                    .strafeToLinearHeading(new Vector2d(18, yCoordinate(-40)), angle(180))
                    .strafeTo(new Vector2d(10, yCoordinate(-30)))
                    .stopAndAdd(scoringMechanism.placePurplePixel)
                    .strafeTo(new Vector2d(38, yCoordinate(-22)))
                    .stopAndAdd(scoringMechanism.test)
                    .build();


            opMode.waitForStart();

                Actions.runBlocking(
                        new SequentialAction(
                                betterRightPosTraj
                        )
//                    new SequentialAction(
//                            toOddGroundPositions
//                                telemetryPacket -> {
//
//                                        // pos 3
//                                        //armExtend.setTarget(860);
//                                        //armRotate.setTarget(300);
//                                        //opMode.sleep(1000);
//
//                                        armRotate.setTarget(250);
//                                        armExtend.setTarget(500 + (int) (0.0574454233 * 150 - 0.7430232633));
//
//                                        clawPitchServo.setPosition(0.5);
//                                        opMode.sleep(500);
//
//                                        // Yellow pixel closer to backdrop when aligning
//
//                                        if (startingPosition.color == StartingColor.RED) {
//                                            clawRightServo.setPosition(0.1);
//                                        } else {
//                                            clawLeftServo.setPosition(0.9);
//                                        }
//
//                                        opMode.sleep(500);
//                                        clawPitchServo.setPosition(0);
//
//
//                                        if (startingPosition.color == StartingColor.RED) {
//                                            clawRightServo.setPosition(0);
//                                        } else {
//                                            clawLeftServo.setPosition(1);
//                                        }
//
//                                        armExtend.setTarget(0);
//                                        opMode.sleep(1000);
//
//                                        clawPitchServo.setPosition(0.8);
//                                        clawRollServo.setPosition(0.55);
//
//                                        armRotate.setTarget(1800);
//                                        armRotate.setTarget(1800 + (int) (0.0574454233 * 150 - 0.7430232633));
//
//                                        opMode.sleep(1000);
//
//                                        return false;
//                                },
//                                trajectories.toTeamPropPos2Pt2(),
//                                trajectories.alignToBackdrop(),
//                                telemetryPacket -> {
//                                    opMode.sleep(500);
//
//                                    if (startingPosition.color == StartingColor.RED) {
//                                        clawLeftServo.setPosition(0.9);
//                                    } else {
//                                        clawRightServo.setPosition(0.1);
//                                    }
//
//                                    opMode.sleep(500);
//                                    return false;
//                                },
//                                trajectories.awayFromBackdrop(),
//                                telemetryPacket -> {
//                                    armRotate.setTarget(300);
//                                    armRotate.setTarget(300 + (int) (0.0574454233 * 150 - 0.7430232633));
//
//                                    opMode.sleep(500);
//
//                                    clawPitchServo.setPosition(0);
//                                    clawRollServo.setPosition(0);
//
//                                    if (startingPosition.color == StartingColor.RED) {
//                                        clawLeftServo.setPosition(1);
//                                    } else {
//                                        clawRightServo.setPosition(0);
//                                    }
//
//                                    armRotate.setTarget(300);
//                                    armRotate.setTarget(300 + (int) (0.0574454233 * 150 - 0.7430232633));
//
//                                    opMode.sleep(1000);
//                                    return false;
//                                }

//                        )
                );
        }

        private double yCoordinate(double coordinate) {
            if (startingPosition.color == StartingColor.RED) {
                return coordinate;
            } else {
                return -coordinate;
            }
        }

        private double angle(double angle) {
            if (startingPosition.color == StartingColor.RED) {
                return Math.toRadians(angle);
            } else {
                return Math.toRadians(-angle);
            }
        }

}
