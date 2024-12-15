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
        robot.init();

        robot.clawServo.setPosition(.2);

        waitForStart();
        runtime.reset();

        telemetry.addData("Status", "Ready to go");
        telemetry.update();
        robot.closeClaw();

//        robot.driveDiagonalForTime(-.4, .5, 1);
//        robot.driveDiagonalByEncoder(0.4, 0.5, 6.4, 1);

        robot.strafeByEncoder(1, 10.2, 10);
        robot.straightByEncoder(-1, -6, 15);

        robot.scoreSpecimen();

        //pushes first sample
        robot.strafeByEncoder(1, -3, 10);
        robot.straightByEncoder(.5, 10, 15); //test
        robot.strafeByEncoder(.5, 14, 15); //test
        robot.straightByEncoder(-.5, 2.25, 15); //test
        robot.strafeByEncoder(.5, -24, 15);
        robot.strafeByEncoder(.5, 2, 15);

        robot.pushSampleClose();
        robot.pushSampleClose();

        robot.strafeByEncoder(.5,-22,15);
        robot.relativeSlideByEncoder(.5, 2, 15);
        while (!robot.isSlideDone()){
        }
        robot.strafeByEncoder(.5,-22,15);
    }
}
