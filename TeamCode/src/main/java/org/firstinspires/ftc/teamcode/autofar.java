package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "autofar")
//@Disabled
public class autofar extends LinearOpMode {

    Hardware2025 robot = new Hardware2025(this);

    @Override
    public void runOpMode() {
        robot.init();

        waitForStart();

        robot.driveDiagonalForTime(-.4, .5, 1.2);

        robot.moveSlideTimed(1, 1.5); //move slide up//robot.openClaw();
        robot.strafeTimed(-.5, .5);
        robot.moveSlideTimed(-1, 1); //move slide back down
        //robot.closeClaw();
        robot.straightTimed(-.5, 4.5);
        robot.strafeTimed(.5,1.5);
        robot.straightTimed(-.5, 1.5);
        robot.strafeTimed(-.5, 4.1);

        }
    }
