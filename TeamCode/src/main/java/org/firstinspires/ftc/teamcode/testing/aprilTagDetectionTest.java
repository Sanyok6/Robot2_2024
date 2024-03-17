package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.lang.reflect.Array;
import java.util.List;

@Config
@TeleOp(name="april tag detection test", group="Tuning")
public class aprilTagDetectionTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        AprilTagProcessor processor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();

        VisionPortal myVisionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), processor);

        while (!isStarted() && !isStopRequested()) {
            List<AprilTagDetection> detections = processor.getDetections();

            if (!detections.isEmpty()) {
                telemetry.addData("x", detections.get(0).ftcPose.x);
                telemetry.addData("z", detections.get(0).ftcPose.y);
            }
            telemetry.update();
        }


        while (opModeIsActive()) {
            List<AprilTagDetection> detections = processor.getDetections();

            if (!detections.isEmpty()) {
                telemetry.addData("x", detections.get(0).ftcPose.x);
                telemetry.addData("z", detections.get(0).ftcPose.y);

                Actions.runBlocking(
                        new SequentialAction(
                                drive.actionBuilder(new Pose2d(0,0,0))
                                        .strafeToConstantHeading(new Vector2d(-(detections.get(0).ftcPose.y - 16), detections.get(0).ftcPose.x - 5))
                                        .build(),
                                (t) -> {requestOpModeStop(); return false;}
                        )
                );
            }
            telemetry.update();

        }
    }

}
