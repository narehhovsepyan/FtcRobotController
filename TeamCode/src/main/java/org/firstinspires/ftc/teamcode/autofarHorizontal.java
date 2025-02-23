package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "autofarHorizontal")
//@Disabled
public class autofarHorizontal extends LinearOpMode {

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
            robot.waitForSlide(1,10,10);
            robot.startSlideByEncoder(1, 26.5, 30);
            robot.driveByOtos(-21.0, 26, 0, 100);
            robot.waitForSlide(1, 26.5, 30);
            robot.straightByEncoder(.5, 2, 30);
            robot.openClaw();

//            robot.startSlideByEncoder(1, 8.7,30);
//            // robot.driveByOtos(28,0,0);
//            robot.strafeByEncoder(.8, -11.0, 30);
//            robot.driveByOtos(0,27.0,0,100);
//            // robot.straightByEncoder(.5,-12.5,10);
//            robot.driveByOtos(7.0,0,0,100);
//            //robot.strafeByEncoder(.5, -1.54, 10);
//            robot.stopArm();
//            //robot.strafeByEncoder(.5,5,10);
//            robot.straightByEncoder(.8,16,10);
//            robot.straightByEncoder(.8,-16,10);
//            robot.strafeByEncoder(.8,-7.0,10);
//            robot.straightByEncoder(.8,16.2,10);
//            robot.turnToHeading(.5,179);
//            robot.waitForSlide(1, 8.7,30);
//            robot.straightByEncoder(.1,-1.2,10);
//            robot.clawServo.setPosition(0.0);
//            robot.startSlideByEncoder(1,11,30);
//            robot.waitForSlide(1,11,30);
//            robot.turnToHeading(.5,-179);

        }
}
