package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "touchSensorTesting")
//@Disabled
public class touchSensorTesting extends LinearOpMode {

    Hardware2025 robot = new Hardware2025(this);

    public void runOpMode() {
        robot.init();

        waitForStart();
        telemetry.update();
        robot.resetYaw();

        robot.startSlideByEncoder(1, 26, 30);
        robot.driveByOtos(-20.0, 15, 0, 100);
        robot.waitForSlide(1, 26, 30);

        robot.driveUntilTouch(.2);
        robot.waitForSlide(1, 18, 30);

    }
}