package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Disabled
@Autonomous(name = "DemoAuto")
//@Disabled
public class DemoAuto extends LinearOpMode {

    Hardware2026 robot = new Hardware2026(this);

    public void runOpMode() {
        robot.init();

        waitForStart();
        telemetry.update();
        robot.resetYaw();
        robot.setTurntableAngleHeavy(45);
        sleep(1000);
        robot.setTurntableAngleHeavy(0);
        sleep(1000);
        robot.setTurntableAngleHeavy(30);
        sleep(1000);
//        robot.driveDiagonalAndTurn(1, 0, 20, 0, 30);
//        sleep(1000);
//        robot.driveDiagonalAndTurn(1, -20, -20, -90, 30);
//        sleep(1000);
//        robot.driveDiagonalAndTurn(1, 20,20, 90, 30);

    }

    }