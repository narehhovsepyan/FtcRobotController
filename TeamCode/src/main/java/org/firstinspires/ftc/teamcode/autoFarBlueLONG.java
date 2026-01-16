package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "autoFarBlueLONG")
//@Disabled
public class autoFarBlueLONG extends LinearOpMode {

    Hardware2026 robot = new Hardware2026(this);

    public void runOpMode() {
        robot.init();
        robot.resetYaw();

        waitForStart();
        telemetry.update();
        robot.resetYaw();

        robot.driveVectorByEncoder(1, 31, 0, -48, 30);

        robot.setTurntableAngle(-58, 1);
        robot.turntableMotor.setPower(0);
        if (robot.patternDecision().equals("GPP")) {
            robot.autoGPP(.5);
            telemetry.addLine("Pattern: GPP");
        } else if (robot.patternDecision().equals("PGP")) {
            robot.autoPGP(.5);
            telemetry.addLine("Pattern: PGP");
        } else if (robot.patternDecision().equals("PPG")) {
            robot.autoPPG(.5);
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

        robot.driveVectorByEncoder(1, -4, 0, -42, 30);

    }

}