package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ArmMode;

@TeleOp
public class TELEOP extends LinearOpMode {
    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        Arm arm = new Arm(hardwareMap);


        waitForStart();

        while (opModeIsActive()) {

            double x = -gamepad1.left_stick_y;
            double y = -gamepad1.left_stick_x;
            double h = drive.imu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double multiplier = gamepad1.left_trigger > 0.9 ? 0.3 : (gamepad1.right_trigger > 0.9 ? 1 : 0.55);

            double rotatedX = x * Math.cos(-h) - y * Math.sin(-h);
            double rotatedY = x * Math.sin(-h) + y * Math.cos(-h);

            drive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(rotatedX * multiplier, rotatedY * multiplier),
                            -gamepad1.right_stick_x / 2
                    )
            );

            if (gamepad1.y) {drive.imu.get().resetYaw();}


            if (gamepad2.a) {
                arm.setMode(ArmMode.drive);
            } else if (gamepad2.b) {
                arm.setMode(ArmMode.outtake);
            }

            arm.setClawLeftOpen(gamepad2.left_trigger > 0);
            arm.setClawRightOpen(gamepad2.right_trigger > 0);

            arm.update();

            telemetry.addData("arm rotation", arm.armRotate.currentPosition());
            telemetry.update();
        }
    }

}