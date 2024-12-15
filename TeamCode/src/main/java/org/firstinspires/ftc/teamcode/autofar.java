package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "autofar")
//@Disabled
public class autofar extends LinearOpMode {

    Hardware2025 robot = new Hardware2025(this);

    public void runOpMode() {
        robot.init();

        robot.clawServo.setPosition(.2);

        waitForStart();
        telemetry.update();
        robot.closeClaw();

        //get to submersible
        robot.strafeByEncoder(1, 8.2, 10);
        robot.straightByEncoder(1, 6, 15);

        //score specimen
        robot.scoreSpecimen();
        robot.strafeByEncoder(.5, -4, 10);

        //push sample 1
        robot.straightByEncoder(1, -10, 15); //test
        robot.strafeByEncoder(1, 12, 15); //test
        robot.straightByEncoder(.5, -2.25, 15); //test
        robot.strafeByEncoder(1, -21, 15); //test

        robot.strafeByEncoder(1, 21, 15);
        robot.pushSampleFar();
        //ROTATE 180 DEGREES
        robot.strafeByEncoder(.5, 1, 15); //TEST- THIS NEEDS TO GO UP TO WALL
        robot.straightByEncoder(.5, -5, 15);
        robot.closeClaw();
        robot.relativeSlideByEncoder(.5, 2, 15);
        while(!robot.isSlideDone()) {
        }
        robot.strafeByEncoder(.5, -10, 15);
        //ROTATE 180 DEGREES

        robot.straightByEncoder(.5, 10, 15);
        robot.strafeByEncoder(.5, 3, 15);

        robot.scoreSpecimen();
        robot.strafeByEncoder(.5, -3, 10);

        robot.straightByEncoder(.5, -13, 15); //test
        robot.strafeByEncoder(.5, 12, 15); //test
        robot.straightByEncoder(.5, -4.5, 15); //test

        robot.pushSampleFar();
        //ROTATE 180 DEGREES
        robot.strafeByEncoder(.5, 1, 15); //TEST- THIS NEEDS TO GO UP TO WALL
        robot.straightByEncoder(.5, -10, 15);
        robot.closeClaw();
        robot.relativeSlideByEncoder(.5, 2, 15);
        while(!robot.isSlideDone()) {
        }
        robot.strafeByEncoder(.5, -10, 15);
        //ROTATE 180 DEGREES
        robot.straightByEncoder(.5, 5, 15);

        robot.scoreSpecimen();

    }
    }
