package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "autofarNew")
//@Disabled
public class autofarNew extends LinearOpMode {

    Hardware2025 robot = new Hardware2025(this);

    public void runOpMode() {
        robot.init();
        robot.configureOtos();
        //Check this
        robot.holdArmEncoder();
        robot.closeBeak();
        robot.clawServo.setPosition(0.0);
        robot.stopArm();

        waitForStart();
        telemetry.update();
        robot.resetYaw();

        //Get to bar and bring slide up
        robot.startSlideByEncoder(1, 26,30);
        robot.driveByOtos(-20.0,15,0,100);
        robot.waitForSlide(1, 26, 30);
       // robot.goToDistance(robot.ROBOT_AT_BAR);


        //score first specimen
        robot.goToDistance(19);
        sleep(500);
        robot.startSlideByEncoder(1, 18,30);
        robot.waitForSlide(1,18,30);
        robot.openClaw();

        //collect next sample
        robot.stopArm(); //why

        robot.driveByOtos(0,-7,0,100);
        robot.startSlideByEncoder(1, 8.7,30);
       // robot.driveByOtos(28,0,0);
        robot.strafeByEncoder(.8, -11.0, 30);
        robot.driveByOtos(0,27.0,0,100);
       // robot.straightByEncoder(.5,-12.5,10);
        robot.driveByOtos(7.0,0,0,100);
        //robot.strafeByEncoder(.5, -1.54, 10);
        robot.stopArm();
        //robot.strafeByEncoder(.5,5,10);
        robot.straightByEncoder(.8,16,10);
        robot.straightByEncoder(.8,-16,10);
        robot.strafeByEncoder(.8,-7.0,10);
        robot.straightByEncoder(.8,16.2,10);
        robot.turnToHeading(.5,179);
        robot.waitForSlide(1, 8.7,30);
        robot.straightByEncoder(.1,-1.2,10);
        robot.clawServo.setPosition(0.0);
        robot.startSlideByEncoder(1,11,30);
        robot.waitForSlide(1,11,30);
        robot.turnToHeading(.5,-179);
        //robot.strafeByEncoder(.8,-6,10);


        //second specimen
        robot.startSlideByEncoder(1, 26,30);
        robot.driveByOtos(-45.0,20,0,100);
        robot.waitForSlide(1, 26, 30);
        robot.goToDistance(15);
        robot.startSlideByEncoder(1, 18,30);
        robot.waitForSlide(1,18,30);
        robot.openClaw();








  /*      //move sample to human player
        robot.straightByEncoder(.8,16,10);
        robot.straightByEncoder(.8,-4.5 ,10);

        //pickup second speciman
        robot.turnToHeading(.5,179);
        //robot.driveByOtos(-1.5,1.5,0.01);
        robot.startSlideByEncoder(1,9,30);
        robot.waitForSlide(1,9,30);
        robot.openClaw();
        robot.straightByEncoder(.4,-8,10);
        sleep(250);
        robot.closeClaw();
        robot.stopArm();
       // robot.straightByEncoder(.8,.8,10);
        robot.turnToHeading(.8,180);
        sleep(200);
        //return to inital postition
//        robot.strafeByEncoder(.8,-2.5,10);
//        robot.straightByEncoder(0.5,.9,10);

        //Get to bar and bring slide up
      //  robot.startSlideByEncoder(1, 25.3,30);
        robot.driveByOtos(-15.0,19,0);
        robot.goToDistance(5);
        robot.strafeByEncoder(.8,6,10);
        robot.waitForSlide(1, 25.3, 30);

        //score first specimen
        robot.straightByEncoder(.3, -4, 10);
       // robot.straightByEncoder(.5,1.2,4);//doesnt work

        robot.waitForSlide(1,17.5,30);

*/
        //robot.driveByOtos(0, -20, -179.0);
        //robot.openClaw();
    }
}
