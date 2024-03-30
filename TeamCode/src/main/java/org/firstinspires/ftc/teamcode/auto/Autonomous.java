package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;

public class Autonomous {
        StartingPosition startingPosition;

        public Autonomous(LinearOpMode opMode, StartingPosition startingPosition) {
                HardwareMap hardwareMap = opMode.hardwareMap;
                this.startingPosition = startingPosition;

                MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(12, yCoordinate(-59), angle(270)));

                ScoringMechanism scoringMechanism = new ScoringMechanism(opMode, startingPosition);

                DistanceSensor leftDist = hardwareMap.get(DistanceSensor.class, "leftDist");
                DistanceSensor rightDist = hardwareMap.get(DistanceSensor.class, "rightDist");
                DistanceSensor centerDist = hardwareMap.get(DistanceSensor.class, "centerDist");

                Action BackdropSideClosePosPreloadTraj = drive.actionBuilder(new Pose2d(12, yCoordinate(-59), angle(270)))
                        .afterTime(1, scoringMechanism.lowerOuttake)
                        .strafeToLinearHeading(new Vector2d(34, yCoordinate(-28)), angle(180))
                        .stopAndAdd(scoringMechanism.placePurplePixel)
                        .strafeTo(new Vector2d(38, yCoordinate(-36)))
                        .stopAndAdd(scoringMechanism.prepareToOuttakeYellowPixel())
                        .waitSeconds(0.25)
                        .stopAndAdd(scoringMechanism.placeYellowPixel())
                        .stopAndAdd(scoringMechanism.armToDriveMode)
                        .strafeTo(new Vector2d(45, yCoordinate(-60)))
                        .strafeTo(new Vector2d(60, yCoordinate(-60)))
                        .build();

                Action BackdropSideCenterPosPreloadTraj = drive.actionBuilder(new Pose2d(12, yCoordinate(-59), angle(270)))
                        .afterTime(1, scoringMechanism.lowerOuttake)
                        .strafeToLinearHeading(new Vector2d(25, yCoordinate(-21)), angle(180))
                        .stopAndAdd(scoringMechanism.placePurplePixel)
                        .strafeTo(new Vector2d(38, yCoordinate(-30)))
                        .stopAndAdd(scoringMechanism.prepareToOuttakeYellowPixel())
                        .waitSeconds(0.5)
                        .stopAndAdd(scoringMechanism.placeYellowPixel())
                        .stopAndAdd(scoringMechanism.armToDriveMode)
                        .strafeTo(new Vector2d(45, yCoordinate(-60)))
                        .strafeTo(new Vector2d(60, yCoordinate(-60)))
                        .build();

                Action BackdropSideFarPosPreloadTraj = drive.actionBuilder(new Pose2d(12, yCoordinate(-59), angle(270)))
                        .afterTime(1.5, scoringMechanism.lowerOuttake)
                        .strafeToLinearHeading(new Vector2d(18, yCoordinate(-40)), angle(180))
                        .strafeTo(new Vector2d(10, yCoordinate(-30)))
                        .stopAndAdd(scoringMechanism.placePurplePixel)
                        .strafeTo(new Vector2d(38, yCoordinate(-23)))
                        .stopAndAdd(scoringMechanism.prepareToOuttakeYellowPixel())
                        .waitSeconds(0.5)
                        .stopAndAdd(scoringMechanism.placeYellowPixel())
                        .stopAndAdd(scoringMechanism.armToDriveMode)
                        .strafeTo(new Vector2d(45, yCoordinate(-60)))
                        .strafeTo(new Vector2d(60, yCoordinate(-60)))
                        .build();


                Action BackdropSideClosePosOneCycleTraj = drive.actionBuilder(new Pose2d(12, yCoordinate(-59), angle(270)))
                        .afterTime(1, scoringMechanism.lowerOuttake)
                        .strafeToLinearHeading(new Vector2d(32, yCoordinate(-28)), angle(180))
                        .stopAndAdd(scoringMechanism.placePurplePixel)
                        .stopAndAdd(scoringMechanism.prepareToOuttakeYellowPixel())
                        .strafeTo(new Vector2d(38, yCoordinate(-36)))
                        .stopAndAdd(scoringMechanism.placeYellowPixel())
                        .afterTime(0, scoringMechanism.armToDriveMode)

                        .strafeTo(new Vector2d(35, yCoordinate(-6)))
                        .afterTime(0.5, scoringMechanism.prepareToIntakeWhitePixel)
                        .strafeTo(new Vector2d(-60, yCoordinate(-6)))
                        .stopAndAdd(scoringMechanism.intakeWhitePixel)

                        .strafeTo(new Vector2d(-59, yCoordinate(-6)), null, new ProfileAccelConstraint(-50, 5))
                        .strafeTo(new Vector2d(35, yCoordinate(-6)))
                        .stopAndAdd(scoringMechanism.prepareToOuttakeWhitePixel())
                        .strafeTo(new Vector2d(41, yCoordinate(-30)))
                        .stopAndAdd(scoringMechanism.placeYellowPixel())
                        .waitSeconds(0.5)
                        .stopAndAdd(scoringMechanism.armToDriveMode)

                        .build();

                Action updateArm = (t) -> {
                    scoringMechanism.armRotate.moveTowardsTarget();
                    return true;
                };

                Action autoTrajectory = BackdropSideClosePosOneCycleTraj;

                while (!opMode.isStarted()) {
                    scoringMechanism.armRotate.moveTowardsTarget();

//                    TeamPropPosition teamPropPosition = determineTeamPropLocation(leftDist, rightDist);
//
//                    if (teamPropPosition == TeamPropPosition.FAR) {
//                        autoTrajectory = BackdropSideClosePosPreloadTraj;
//                    } else if (teamPropPosition == TeamPropPosition.CLOSE) {
//                        autoTrajectory = BackdropSideFarPosPreloadTraj;
//                    } else {
//                        autoTrajectory = BackdropSideCenterPosPreloadTraj;
//                    }
//
//                    opMode.telemetry.addData("Team prop location", teamPropPosition);
//                    opMode.telemetry.update();
                }


                Actions.runBlocking(
                        new ParallelAction(
                                autoTrajectory,
                                updateArm
                        )
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

        public TeamPropPosition determineTeamPropLocation(DistanceSensor leftDist, DistanceSensor rightDist) {
            if (startingPosition.color == StartingColor.BLUE) {
                    if (startingPosition.side == StartingSide.BACKDROP) {
                        if (centerDist.getDistance(DistanceUnit.CM) < 90) {
                            return TeamPropPosition.CENTER;
                        } else if (leftDist.getDistance(DistanceUnit.CM) < 90) {
                            return TeamPropPosition.CLOSE;
                        } else {
                            return TeamPropPosition.FAR;
                        }
                        } else { 
                            if (centerDist.getDistance(DistanceUnit.CM) < 90) {
                            return TeamPropPosition.CENTER;
                        } else if (rightDist.getDistance(DistanceUnit.CM) < 90) {
                            return TeamPropPosition.FAR;
                        } else {
                            return TeamPropPosition.CLOSE;
                        }
                    }
            } else {
                    if (startingPosition.side == StartingSide.BACKDROP) {
                        if (centerDist.getDistance(DistanceUnit.CM) < 90) {
                            return TeamPropPosition.CENTER;
                        } else if (rightDist.getDistance(DistanceUnit.CM) < 90) {
                            return TeamPropPosition.CLOSE;
                        } else {
                            return TeamPropPosition.FAR;
                        }
                        } else { 
                            if (centerDist.getDistance(DistanceUnit.CM) < 90) {
                            return TeamPropPosition.CENTER;
                        } else if (leftDist.getDistance(DistanceUnit.CM) < 90) {
                            return TeamPropPosition.FAR;
                        } else {
                            return TeamPropPosition.CLOSE;
                        }
                    }
                }
}
