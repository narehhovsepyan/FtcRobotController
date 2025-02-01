package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "autotesting")
//@Disabled
public class autotesting extends LinearOpMode {

    Hardware2025 robot = new Hardware2025(this);

    public void runOpMode() {
        robot.init();

        robot.closeBeak();
        robot.closeClaw();
        robot.holdArmEncoder();
        robot.resetYaw();

        waitForStart();
        telemetry.update();

        //get to submersible
        robot.startSlideByEncoder(1, 26.5,30);
        robot.driveByOtos(-20.0,27.0,0);

    }
}