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
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ArmExtend;
import org.firstinspires.ftc.teamcode.subsystems.ArmMode;
import org.firstinspires.ftc.teamcode.subsystems.ArmRotate;

@Config
@TeleOp(name="Arm Tune", group="Tuning")
public class ArmTune extends LinearOpMode {
    public static double clawRightPos = 0; // Drive: 0, Intake: 0.4, Outtake: 0.1
    public static double clawLeftPos = 1; // Drive: 1, Intake: 0.6, Outtake: 0.9

    public static double clawRollPos = 0; // Drive: 0, Intake: 0, Outtake:
    public static double clawPitchPos = 0.5; // Drive: 0, Intake: 0.55, Outtake:

    public static int rotationTarget = 0; // Drive: 50, Intake: 100, Outtake:
    public static int extensionTarget = 0; // Drive: 0, Intake: 300, Outtake:


    @Override
    public void runOpMode() {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        Servo clawRightServo = hardwareMap.get(Servo.class, "clawRight");
        Servo clawLeftServo = hardwareMap.get(Servo.class, "clawLeft");

        Servo clawRollServo = hardwareMap.get(Servo.class, "clawRoll");

        ServoImplEx clawPitchServo = hardwareMap.get(ServoImplEx.class, "clawPitch");
        clawPitchServo.setPwmRange(new PwmControl.PwmRange(2500, 500));

        ArmRotate armRotate = new ArmRotate(hardwareMap.get(DcMotorEx.class, "armRotate"));
        ArmExtend armExtend = new ArmExtend(hardwareMap.get(DcMotorEx.class, "linearSlide"));

        waitForStart();

        while (opModeIsActive()) {

            clawRightServo.setPosition(clawRightPos);
            clawLeftServo.setPosition(clawLeftPos);

            clawRollServo.setPosition(clawRollPos);

            clawPitchServo.setPosition(clawPitchPos);

            armRotate.setTarget(rotationTarget);

            double predictedExtension = 0.0574454233 * armRotate.currentPosition() - 0.7430232633;
            armExtend.setTarget(extensionTarget + (int) predictedExtension);


            telemetry.addData("Arm rotation", armRotate.currentPosition());
            telemetry.addData("Arm extension", armExtend.currentPosition());
            telemetry.update();
        }
    }

}