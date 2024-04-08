package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ArmMode;

@Config
@TeleOp
public class TELEOP extends LinearOpMode {
    public static double Kp = 1;
    public static double Ki = 0;
    public static double Kd = 0.15;
    double integralSum = 0;
    double lastError = 0;

    public static double s = 8;

    @Override
    public void runOpMode() {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        Arm arm = new Arm(hardwareMap);

        boolean armMoveUpPressed = false;
        boolean armMoveDownPressed = false;

        Servo planeServo = hardwareMap.servo.get("plane");
        planeServo.setPosition(0.97);

        double targetHeading = drive.imu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        waitForStart();

        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive()) {

            double currentHeading = drive.imu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double x = -gamepad1.left_stick_y;
            double y = -gamepad1.left_stick_x;

            double multiplier = gamepad1.left_trigger > 0.9 ? 0.3 : (gamepad1.right_trigger > 0.9 ? 1 : 0.55);

            double rotatedX = x * Math.cos(-currentHeading) - y * Math.sin(-currentHeading);
            double rotatedY = x * Math.sin(-currentHeading) + y * Math.cos(-currentHeading);

            targetHeading += -gamepad1.right_stick_x / s;

            if (targetHeading > Math.PI) {
                targetHeading -= Math.PI * 2;
            } else if (targetHeading < -Math.PI) {
                targetHeading += Math.PI * 2;
            }

            double error;
            if (Math.abs(targetHeading + 2*Math.PI - currentHeading) < Math.PI) {
                error = targetHeading + 2*Math.PI - currentHeading;
            } else if (Math.abs( targetHeading - 2*Math.PI - currentHeading) < Math.PI) {
                error = targetHeading - 2*Math.PI - currentHeading;
            } else {
                error = targetHeading - currentHeading;
            }

            double derivative = (error - lastError) / timer.seconds();
            integralSum = integralSum + (error * timer.seconds());

            lastError = error;
            timer.reset();

            double turnPower = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

            drive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(rotatedX * multiplier, rotatedY * multiplier),
                            turnPower
                    )
            );

            if (gamepad1.y) {
                drive.imu.get().resetYaw();
                targetHeading = 0;
            }

            if (gamepad2.right_bumper && gamepad2.a && arm.mode == ArmMode.DRIVE) {
                arm.resetRotateEncoder();
            } else {
                if (gamepad2.a) {
                    arm.setMode(ArmMode.DRIVE);
                } else if (gamepad2.b) {
                    arm.setMode(ArmMode.OUTTAKE);
                } else if (gamepad2.y) {
                    if (gamepad2.right_bumper) {
                        arm.setMode(ArmMode.VERTICAL);
                    } else {
                        arm.setMode(ArmMode.HANG);
                    }
                } else if (!gamepad2.dpad_up && armMoveUpPressed) {
                    arm.moveAlongBackdrop(6);
                    armMoveUpPressed = false;
                } else if (!gamepad2.dpad_down && armMoveDownPressed) {
                    arm.moveAlongBackdrop(-6);
                    armMoveDownPressed = false;
                } else if (gamepad2.dpad_up) {
                    armMoveUpPressed = true;
                } else if (gamepad2.dpad_down) {
                    armMoveDownPressed = true;
                } else if (gamepad1.dpad_left && gamepad2.dpad_left) {
                    planeServo.setPosition(0.4);
                }

                arm.setClawLeftOpen(gamepad2.left_trigger > 0);
                arm.setClawRightOpen(gamepad2.right_trigger > 0);

                arm.update();
            }

            telemetry.addData("arm rotation", arm.armRotate.getCurrentPosition());
            telemetry.addData("claw pos", arm.targetPitchPosition);
            telemetry.update();
        }
    }

}
