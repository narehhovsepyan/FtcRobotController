package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "autoFarBlue")
//@Disabled
public class autoFarBlue extends LinearOpMode {

    Hardware2026 robot = new Hardware2026(this);

    public void runOpMode() {
        robot.init();
        robot.resetYaw();

        waitForStart();
        telemetry.update();
        robot.resetYaw();

        robot.driveVectorByEncoder(1, 5.4, 0, -30, 30);

        robot.setTurntableAngle(-53, 1);
        robot.turntableMotor.setPower(0);
        if (robot.patternDecision().equals("GPP")) {
            robot.autoGPP(1);
            telemetry.addLine("Pattern: GPP");
        } else if (robot.patternDecision().equals("PGP")) {
            robot.autoPGP(1);
            telemetry.addLine("Pattern: PGP");
        } else if (robot.patternDecision().equals("PPG")) {
            robot.autoPPG(1);
            telemetry.addLine("Pattern: PPG");
        } else if (robot.patternDecision().equals("")){
            robot.shootBalls();
            telemetry.addLine("Pattern: NONE");
        }


//        robot.shootArtifact(1);
//        sleep(500);
//        robot.daisySpin(1 ,5);
//        sleep(500);
//        robot.shootArtifact(0);

        robot.driveVectorByEncoder(1, 3, 0, -37, 30);

    }

}