package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ArmMode;

@TeleOp
public class TELEOP extends LinearOpMode {
    @Override
    public void runOpMode() {

        Arm arm = new Arm(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad2.a) {
                arm.setMode(ArmMode.drive);
            } else if (gamepad2.b) {
                arm.setMode(ArmMode.outtake);
            }

            arm.setClawLeftOpen(gamepad2.left_trigger > 0);
            arm.setClawRightOpen(gamepad2.right_trigger > 0);

            arm.update();
        }
    }

}