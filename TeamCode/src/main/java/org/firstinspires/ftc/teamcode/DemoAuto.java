package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//@Disabled
@Autonomous(name = "DemoAuto")
//@Disabled
public class DemoAuto extends LinearOpMode {

    Hardware2026 robot = new Hardware2026(this);

    public void runOpMode() {
        robot.init();

        waitForStart();
        telemetry.update();
        robot.resetYaw();
        robot.setTurntableAngle(90);
        sleep(1000);
        robot.setTurntableAngle(180);
        sleep(1000);
        robot.setTurntableAngle(-90);
        sleep(1000);
        robot.setTurntableAngle(30);
        sleep(1000);
        robot.setTurntableAngle(48);
        sleep(1000);
        robot.setTurntableAngle(-50);
        sleep(1000);
        robot.setTurntableAngle(0, .8);
        sleep(1000);
//        robot.driveDiagonalAndTurn(1, 0, 20, 0, 30);
//        sleep(1000);
//        robot.driveDiagonalAndTurn(1, -20, -20, -90, 30);
//        sleep(1000);
//        robot.driveDiagonalAndTurn(1, 20,20, 90, 30);

    }

    }