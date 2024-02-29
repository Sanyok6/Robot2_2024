package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.ArmExtend;
import org.firstinspires.ftc.teamcode.subsystems.ArmRotatePID;

@Config
@TeleOp(name="Arm Tune", group="Tuning")
public class ArmTune extends LinearOpMode {
    public static double clawRightPos = 0; // Drive: 0, Intake: 0.4, Outtake: 0.1
    public static double clawLeftPos = 1; // Drive: 1, Intake: 0.6, Outtake: 0.9

    public static double clawRollPos = 0; // Drive: 0, Intake: 0, Outtake:
    public static double clawPitchPos = 0.5; // Drive: 0, Intake: 0.52, Outtake:

    public static int rotationTarget = 0; // Drive: 300, Intake: 250, Outtake:
    public static int extensionTarget = 0; // Drive: 0, Intake: 300, Outtake:

    public static double Kp = 0.001;

    public static double max_acceleration = 0.01;
    public static double max_velocity = 6;


    @Override
    public void runOpMode() {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        Servo clawRightServo = hardwareMap.get(Servo.class, "clawRight");
        Servo clawLeftServo = hardwareMap.get(Servo.class, "clawLeft");

        Servo clawRollServo = hardwareMap.get(Servo.class, "clawRoll");

        ServoImplEx clawPitchServo = hardwareMap.get(ServoImplEx.class, "clawPitch");
        clawPitchServo.setPwmRange(new PwmControl.PwmRange(2500, 500));

        ArmRotatePID armRotate = new ArmRotatePID(hardwareMap.get(DcMotorEx.class, "armRotate"), hardwareMap.get(DcMotorEx.class, "armRotateEncoder"));
        ArmExtend armExtend = new ArmExtend(hardwareMap.get(DcMotorEx.class, "linearSlide"));

        waitForStart();

        while (opModeIsActive()) {

            clawRightServo.setPosition(clawRightPos);
            clawLeftServo.setPosition(clawLeftPos);

            clawRollServo.setPosition(clawRollPos);

            clawPitchServo.setPosition(clawPitchPos);


            armRotate.Kp = Kp;

            armRotate.max_acceleration = max_acceleration;
            armRotate.max_velocity = max_velocity;

            armRotate.setTarget(rotationTarget);
            armRotate.moveTowardsTarget();

            armExtend.setCompensatedTarget(extensionTarget, rotationTarget);


            telemetry.addData("Arm rotation", armRotate.getCurrentPosition());
            telemetry.addData("Arm extension", armExtend.currentPosition());
            telemetry.addData("Predicted extension", (int) (0.027015 * armRotate.getCurrentPosition() - 10.1046));
            telemetry.update();
        }
    }

}