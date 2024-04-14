package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.Autonomous;
import org.firstinspires.ftc.teamcode.auto.StartingColor;
import org.firstinspires.ftc.teamcode.auto.AutoPath;
import org.firstinspires.ftc.teamcode.auto.PathType;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group = "AUDIENCE PRELOAD", name = "BLUE AUDIENCE TRUSS")
public class AUTO_BLUE_AUDIENCE_TRUSS extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        new Autonomous(this,
                new AutoPath(StartingColor.BLUE, PathType.AUDIENCE_PRELOAD_TRUSS));

    }
}
