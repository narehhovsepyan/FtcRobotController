package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous(name = "DemoAuto")
//@Disabled
public class DemoAuto extends LinearOpMode {

    Hardware2026 robot = new Hardware2026(this);

    public void runOpMode() {
        robot.init();

        waitForStart();
        telemetry.update();
        robot.resetYaw();

        robot.shootBallControlled(1);
        sleep(500);
        robot.shootBallControlled(.8);
        sleep(500);
        robot.shootBallControlled(.5);
        sleep(500);
        robot.shootBallControlled(.2);
        sleep(500);
        robot.shootBallControlled(.8);
        sleep(500);
    }

    }