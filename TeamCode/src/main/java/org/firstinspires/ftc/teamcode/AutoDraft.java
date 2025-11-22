package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AutoDraft")
//@Disabled
public class AutoDraft extends LinearOpMode {

    Hardware2026 robot = new Hardware2026(this);

    public void runOpMode() {
        robot.init();
        robot.configureOtos();

        waitForStart();
        telemetry.update();
        robot.resetYaw();

        int pattern = robot.autoDecision();
        robot.driveByOtos(12,-18,0,30);
        robot.spinTake(1.0);
        robot.turnToHeading(.5, 90);
//
//        if (pattern == 1) {
//            robot.autoGPP();
//        } else if (pattern == 2) {
//            robot.autoGPG();
//        } else if (pattern == 3) {
//            robot.autoPPG();
//        } else {
//            telemetry.addLine("NO APRILTAG FOUND!");
//        }
    }



//            robot.startSlideByEncoder(1, 8.7,30);
            // robot.driveByOtos(28,0,0);
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

