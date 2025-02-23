package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "autotesting")
//@Disabled
public class autotesting extends LinearOpMode {

    Hardware2025 robot = new Hardware2025(this);

    public void runOpMode() {
        robot.init();
        robot.configureOtos();
        waitForStart();
        telemetry.update();
        robot.driveByOtos(8,8,0,10);
        //robot.driveByOtos(55,0,0,5);
        //robot.driveByOtos(3,0,0,5);
        //robot.driveByOtos(0,-43,0,5);

    }
}