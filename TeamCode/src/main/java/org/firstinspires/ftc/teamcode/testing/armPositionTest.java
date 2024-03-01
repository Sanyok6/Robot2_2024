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
@TeleOp(name="arm position test", group="Tuning")
public class armPositionTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        DcMotorEx armRotate = hardwareMap.get(DcMotorEx.class, "armRotate");
        DcMotorEx linearSlide = hardwareMap.get(DcMotorEx.class, "linearSlide");

        armRotate.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        int targetRotation = 500;

        int targetExtension = 0;

        double predictedExtension = 0;

        while (opModeIsActive()) {

            predictedExtension = 0.0574454233 * -armRotate.getCurrentPosition() - 0.7430232633;

            telemetry.addData("Arm rotation: ", -armRotate.getCurrentPosition());
            telemetry.addData("Arm extension: ", linearSlide.getCurrentPosition());
            telemetry.addData("Predicted arm extension: ", predictedExtension);
            telemetry.update();

        }
    }

}
