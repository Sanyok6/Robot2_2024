package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.Autonomous;
import org.firstinspires.ftc.teamcode.auto.StartingColor;
import org.firstinspires.ftc.teamcode.auto.StartingPosition;
import org.firstinspires.ftc.teamcode.auto.StartingSide;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class AUTO_RED_CLOSE extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Autonomous autonomous = new Autonomous(this,
                new StartingPosition(StartingColor.RED, StartingSide.CLOSE));

    }
}
