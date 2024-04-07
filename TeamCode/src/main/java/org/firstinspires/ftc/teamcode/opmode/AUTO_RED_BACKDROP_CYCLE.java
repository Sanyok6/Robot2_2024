package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.Autonomous;
import org.firstinspires.ftc.teamcode.auto.StartingColor;
import org.firstinspires.ftc.teamcode.auto.AutoPath;
import org.firstinspires.ftc.teamcode.auto.PathType;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group = "BACKDROP CYCLE", name = "RED BACKDROP CYCLE")
public class AUTO_RED_BACKDROP_CYCLE extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        new Autonomous(this,
                new AutoPath(StartingColor.RED, PathType.BACKDROP_CYCLE_MIDLINE));

    }
}
