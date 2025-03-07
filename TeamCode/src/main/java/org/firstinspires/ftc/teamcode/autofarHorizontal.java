package org.firstinspires.ftc.teamcode;
import android.util.Log;

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
            robot.arm.setPower(-0.01);

            waitForStart();
            telemetry.update();
            robot.resetYaw();


            //Get to bar and bring slide up
            robot.closeClaw();
            robot.waitForSlide(1,10,10);
            robot.startSlideByEncoder(1, 26, 30);
            Log.i("FTC18 autofarhorizontal", "Move 1 start -------------------------------");
            robot.driveByOtos(-17, 22, 0, 10);
            Log.i("FTC18 autofarhorizontal", "Move 2 start -------------------------------");
            robot.straightByEncoder(.5,-3,.5);
            robot.waitForSlide(1, 26.5, 30);
            Log.i("FTC18 autofarhorizontal", "Move 3 start -------------------------------");
            robot.straightByEncoder(.5, 3, 30);
            robot.openClaw();
            robot.startSlideByEncoder(1,10,20);
            Log.i("FTC18 autofarhorizontal", "Move 4 start -------------------------------");
            robot.driveByOtos(28,-10,-179,3.5);
            robot.startSlideByEncoder(1,10,20);
            Log.i("FTC18 autofarhorizontal", "Move 5 start -------------------------------");
            robot.straightByEncoder(.4,-6,5);
            robot.waitForSlide(1,9.5,10);
            robot.closeClaw();
            robot.waitForSlide(1,18,20);
            Log.i("FTC18 autofarhorizontal", "Move 6 start -------------------------------");
            robot.straightByEncoder(.5,3,10);
            robot.startSlideByEncoder(1,30,10);
            Log.i("FTC18 autofarhorizontal", "Move 7 start -------------------------------");
            robot.driveByOtos(-10,10,0,2);
            Log.i("FTC18 autofarhorizontal", "Move 8 start -------------------------------");
            robot.driveByOtos(-20,0,0,2);
            Log.i("FTC18 autofarhorizontal", "Move 9 start -------------------------------");
            robot.waitForSlide(1,30,2);
            robot.straightByEncoder(.6,-3,2);
            robot.waitForSlide(1,25,3);
            Log.i("FTC18 autofarhorizontal", "Move 10 start -------------------------------");
            robot.straightByEncoder(.5,3,2);
            robot.openClaw();
            Log.i("FTC18 autofarhorizontal", "Move 11 start -------------------------------");
            robot.strafeByEncoder(1,-14.3,10);
            Log.i("FTC18 autofarhorizontal", "Move 12 start -------------------------------");
            robot.straightByEncoder(1,-9.8,10);
            Log.i("FTC18 autofarhorizontal", "Move 13 start -------------------------------");
            robot.strafeByEncoder(1,-3.5,10);
            Log.i("FTC18 autofarhorizontal", "Move 14 start -------------------------------");
            robot.straightByEncoder(1,16,10);
        }
}
