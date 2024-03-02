package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@TeleOp(name="distance sensor test", group="Tuning")
public class distanceSensorTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        DistanceSensor leftDist = hardwareMap.get(DistanceSensor.class, "leftDist");
        DistanceSensor rightDist = hardwareMap.get(DistanceSensor.class, "rightDist");

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("Left dist", leftDist.getDistance(DistanceUnit.CM));
            telemetry.addData("Right dist", rightDist.getDistance(DistanceUnit.CM));
            telemetry.update();

        }
    }

}
