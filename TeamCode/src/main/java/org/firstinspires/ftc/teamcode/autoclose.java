package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "autoclose")
//@Disabled
public class autoclose extends LinearOpMode {

    Hardware2025 robot = new Hardware2025(this);
    private final ElapsedTime runtime = new ElapsedTime();

    //@Override
    public void runOpMode() {
        //Init methods
        robot.init();
        robot.configureOtos();
        robot.closeBeak();
        robot.closeClaw();
        robot.holdArmEncoder();


        waitForStart();
        telemetry.update();
        robot.resetYaw();

        //score sample 1
        robot.startSlideByEncoder(1, 27.5,30);
        robot.strafeByEncoder(1,-1,2);
        robot.straightByEncoder(.6,-6.05,30);
        robot.waitForSlide(1, 27.5, 30);
        robot.turnToHeading(.5,10);
        robot.straightByEncoder(.5, -2.6,10);
        robot.clawServo.setPosition(.3);

        //get to second sample

        robot.turnToHeading(.6,-90);
        robot.startSlideByEncoder(1, 0,30);
        robot.closeClaw();
        robot.strafeByEncoder(.55,-2.08,10);
        robot.straightByEncoder(.7,-4.9,20);
        robot.waitForSlide(1, 0, 30);

        //pick up sample 2
        robot.openClaw();
        robot.straightByEncoder(.55,-1.3,30);
        robot.clawServo.setPosition(0.01);

        //score sample 2
        robot.startSlideByEncoder(1, 27.5,30);
        robot.straightByEncoder(.7,5.86,20);
        robot.waitForSlide(1, 27.5, 30);
        robot.turnToHeading(.7,22);

        robot.straightByEncoder(.6,-3,10);
        robot.clawServo.setPosition(.01);







        //robot.strafeByEncoder(.5, 10.9, 10);
        //robot.straightByEncoder(.5, -7, 15);

        //robot.scoreSpecimen();

        //pushes first sampl

        //robot.strafeByEncoder(1, -3, 10);
//        robot.straightByEncoder(.5, 8.2, 15); //test
//        robot.strafeByEncoder(1, 14, 15); //test
//        robot.straightByEncoder(.5, 2.34, 15); //test
//        robot.strafeByEncoder(1, -20, 15);

        //robot.pushSampleClose();

        //pushes last sample, aligns w wall first
//        robot.strafeByEncoder(.5, 21, 15);
//        robot.straightByEncoder(.5, 7, 15);
//        robot.straightByEncoder(.5, -.8, 15);
//        robot.strafeByEncoder(.5, -21, 15);

//        robot.relativeSlideByEncoder(.5, 2, 15);
//        while (!robot.isSlideDone()){
//        }
    }
}
