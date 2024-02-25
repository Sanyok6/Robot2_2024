package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.auto.Autonomous;
import org.firstinspires.ftc.teamcode.auto.StartingColor;
import org.firstinspires.ftc.teamcode.auto.StartingPosition;
import org.firstinspires.ftc.teamcode.auto.StartingSide;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class TESTING extends LinearOpMode {

    public MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        Action traj = drive.actionBuilder(new Pose2d(0,0,0))
                .lineToX(30)
                .afterTime(0.5, telemetryPacket -> {
                    telemetry.addLine("test");
                    telemetry.update();
                    return false;
                })
                .lineToX(0)
                .build();

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        traj
                )
        );
    }

    public Pose2d pose() {
        return drive.pose;
    }
}
