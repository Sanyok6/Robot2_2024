package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp(name="full arm test", group="Tuning")
public class fullArmTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        DcMotorEx armRotate = hardwareMap.get(DcMotorEx.class, "armRotate");
        DcMotorEx linearSlide = hardwareMap.get(DcMotorEx.class, "linearSlide");

        armRotate.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        // Min position: 0, Max position: 2250

        // Linear slide: 1, 1000

        // equation: extension = -0.0570170042x + 3.261793482
        // new equation: 0.0574454233x - 0.7430232633

        int targetRotation = 500;

        int targetExtension = 0;

        double predictedExtension = -0.0570170042 * armRotate.getCurrentPosition() + 3.261793482;

        while (opModeIsActive()) {

            predictedExtension = 0.0574454233 * armRotate.getCurrentPosition() - 0.7430232633;
//
//            armRotate.setTargetPosition(targetRotation);
//            armRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            armRotate.setPower(1);
//
//            linearSlide.setTargetPosition(targetExtension + (int) predictedExtension);
//            linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            linearSlide.setPower(1);
//
//            if (gamepad1.dpad_left) {
//                targetRotation = 2250;
//            } else if (gamepad1.dpad_up) {
//                targetRotation = 1200;
//            } else if (gamepad1.dpad_right) {
//                targetRotation = 500;
//            }
//
//            if (gamepad1.y) {
//                targetExtension = 1000;
//            } else if (gamepad1.x) {
//                targetExtension = 500;
//            } else if (gamepad1.a) {
//                targetExtension = 0;
//            }

            telemetry.addData("Arm rotation: ", armRotate.getCurrentPosition());
            telemetry.addData("Arm extension: ", linearSlide.getCurrentPosition());
            telemetry.addData("Predicted arm extension: ", predictedExtension);
            telemetry.update();

        }
    }

}
