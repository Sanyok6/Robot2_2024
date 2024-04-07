package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.Autonomous;
import org.firstinspires.ftc.teamcode.auto.StartingColor;
import org.firstinspires.ftc.teamcode.auto.AutoPath;
import org.firstinspires.ftc.teamcode.auto.PathType;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group = "BACKDROP PRELOAD", name = "BLUE BACKDROP PRELOAD")
public class AUTO_BLUE_BACKDROP_PRELOAD extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        new Autonomous(this,
                new AutoPath(StartingColor.BLUE, PathType.BACKDROP_PRELOAD_MIDLINE));

    }
}
