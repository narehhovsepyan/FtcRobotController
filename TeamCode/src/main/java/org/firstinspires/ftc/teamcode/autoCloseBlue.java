package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "autoCloseBlue")
//@Disabled
public class autoCloseBlue extends LinearOpMode {

    Hardware2026 robot = new Hardware2026(this);

    public void runOpMode() {

        robot.init();
        robot.resetYaw();

        waitForStart();
        telemetry.update();
        robot.resetYaw();


        robot.driveVectorByEncoder(1, .8, 1.3, 13, 30);
        if (robot.patternDecision().equals("GPP")) {
            robot.autoGPP(.7);
        } else if (robot.patternDecision().equals("PGP")) {
            robot.autoPGP(.7);
        } else if (robot.patternDecision().equals("PPG")) {
            robot.autoPPG(.7);
        }

//        robot.turntableMotor.setPower(0);
//        robot.shootArtifact(.8);
//        sleep(500);
//        robot.daisySpin(1 ,5);
//        robot.shootArtifact(0);

        robot.driveVectorByEncoder(1, -16, 18, 0, 30);
    }

    }