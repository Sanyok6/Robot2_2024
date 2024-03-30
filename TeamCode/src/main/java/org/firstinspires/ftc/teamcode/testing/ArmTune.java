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
import org.firstinspires.ftc.teamcode.subsystems.ArmRotate;

@Config
@TeleOp(name="Arm Tune", group="Tuning")
public class ArmTune extends LinearOpMode {
    public static double clawRightPos = 0; // Drive: 0, Intake: 0.4, Outtake: 0.1
    public static double clawLeftPos = 1; // Drive: 1, Intake: 0.6, Outtake: 0.9

    public static double clawRollPos = 0; // Drive: 0.1, Intake: 0.1, Outtake: 0.37
    public static double clawPitchPos = 0.5; // Drive: 0, Intake: 0.52, Outtake:

    public static int rotationTarget = 0; // Drive: 300, Intake: 250, Outtake:
    public static int extensionTarget = 0; // Drive: 0, Intake: 300, Outtake:

    public static double Kp = 0.002;
    public static double f = 0;

    public static double max_acceleration = 0.01;
    public static double max_velocity = 4;


    public static double x = 0;
    public static double y = 0;
    public static double θ = 0;

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

        Arm arm = new Arm(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            clawRightServo.setPosition(clawRightPos);
            clawLeftServo.setPosition(clawLeftPos);

            clawRollServo.setPosition(clawRollPos);

            clawPitchServo.setPosition(clawPitchPos);


            armRotate.Kp = Kp;
            armRotate.f = f;

            armRotate.max_acceleration = max_acceleration;
            armRotate.max_velocity = max_velocity;

            armRotate.setTarget(rotationTarget);
            armRotate.moveTowardsTarget();

            armExtend.setCompensatedTarget(extensionTarget, rotationTarget);


            telemetry.addData("Arm rotation", armRotate.getCurrentPosition());
            telemetry.addData("Arm extension", armExtend.currentPosition());
            telemetry.addData("Predicted extension", (int) (0.027015 * armRotate.getCurrentPosition() - 10.1046));

            double theta = Math.toRadians(θ);

            //constant
            double r = 14;

            //outputs
            double L = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2) + Math.pow(r, 2) - 2*r*x*Math.cos(theta) - 2*r*y*Math.sin(theta));
            double phi_1 = Math.atan2(y - r*Math.sin(theta), x - r*Math.cos(theta));
            double phi_2 = theta - phi_1;

            phi_1 = Math.toDegrees(phi_1);
            phi_2 = Math.toDegrees(phi_2);

            // calculate linear slide encoder position from target extension length
            double extensionTarget = 18.32 * L - 544.31;

            // calculate arm rotation encoder position from angle
            double rotationTarget = -24.3 * phi_1 + 4546;

            // calculate pitch servo position from angle
            double clawTarget = 0.00555 * phi_2 + 0.616;

            telemetry.addData("raw extension target", L);
            telemetry.addData("raw rotation target", phi_1);
            telemetry.addData("raw claw target", phi_2);

            telemetry.addData("extension target", extensionTarget);
            telemetry.addData("rotation target", rotationTarget);
            telemetry.addData("claw target", clawTarget);

            telemetry.update();
        }
    }

}