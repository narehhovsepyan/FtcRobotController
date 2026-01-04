package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "autoFarRed")
//@Disabled
public class autoFarRed extends LinearOpMode {

    Hardware2026 robot = new Hardware2026(this);

    public void runOpMode() {
        robot.init();
        robot.resetYaw();

        waitForStart();
        telemetry.update();
        robot.resetYaw();

        robot.driveVectorByEncoder(1, -5.3, 0, 20, 30);
        if (robot.autoDecision().equals("GPP")) {
            robot.autoGPP();
        } else if (robot.autoDecision().equals("PGP")) {
            robot.autoPGP();
        } else if (robot.autoDecision().equals("PPG")) {
            robot.autoPPG();
        }

//        robot.shootArtifact(1);
//        sleep(500);
//        robot.daisySpin(1 ,5);
//        sleep(500);
//        robot.shootArtifact(0);

        robot.driveVectorByEncoder(1, -3, 0, 0, 30);

    }

}