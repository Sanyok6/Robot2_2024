package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class Trajectories {

    public MecanumDrive drive;
    public StartingPosition startingPosition;

    public Trajectories(HardwareMap hardwareMap, StartingPosition startingPosition) {
        this.startingPosition = startingPosition;
        this.drive = new MecanumDrive(hardwareMap, new Pose2d(12, yCoordinate(-60), angle(270)));
    }

    public Action toTeamPropOddPositions() {
        return drive.actionBuilder(new Pose2d(12, yCoordinate(-60), angle(270)))
                        .strafeToLinearHeading(new Vector2d(34, yCoordinate(-28)), angle(180))
                        .build();
    }

    public Action toTeamPropPos2Pt1() {
        return drive.actionBuilder(new Pose2d(12, yCoordinate(-60), angle(270)))
                .strafeToLinearHeading(new Vector2d(34, yCoordinate(-28)), angle(160))
                .build();
    }

    public Action toTeamPropPos2Pt2() {
        return drive.actionBuilder(new Pose2d(34, yCoordinate(-28), angle(160)))
                .turnTo(angle(180))
                .build();
    }



    public Action simplePark() {
        return drive.actionBuilder(new Pose2d(34, yCoordinate(-28), angle(180)))
                .setTangent(angle(180))
                .lineToX(45)
                .build();
    }

    public Action alignToBackdrop() {
        return drive.actionBuilder(new Pose2d(34, yCoordinate(-28), angle(180)))
                .setTangent(angle(90))
                .lineToY(yCoordinate(-24))

                .setTangent(angle(180))
                .lineToX(45, new TranslationalVelConstraint(5), new ProfileAccelConstraint(-5, 5))

                .build();
    }

    public Action awayFromBackdrop() {
        return drive.actionBuilder(new Pose2d(45, yCoordinate(-24), angle(180)))
                .lineToX(42, new TranslationalVelConstraint(5))
                .build();
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
