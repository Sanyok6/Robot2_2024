package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
@TeleOp(name="arm rotate test", group="Tuning")
public class armRotateTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "armRotate");

        waitForStart();

        // Min position: 0, Max position: 2250

        int target = 500;

        while (opModeIsActive()) {
            motor.setTargetPosition(target);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(1);

            if (gamepad1.dpad_left) {
                target = 2250;
            } else if (gamepad1.dpad_up) {
                target = 1200;
            } else if (gamepad1.dpad_right) {
                target = 500;
            }

            telemetry.addData("Arm position: ", motor.getCurrentPosition());
            telemetry.addData("Current draw: ", motor.getCurrent(CurrentUnit.AMPS));


            telemetry.update();

        }
    }

}
