package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.AprilTagWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
@Autonomous
@Disabled

public class AprilTagWebcamExample extends OpMode {
    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    @Override
    public void init() {
        aprilTagWebcam.init(hardwareMap, telemetry);
    }
    @Override
    public void loop() {
        aprilTagWebcam.update();
        AprilTagDetection id20 = aprilTagWebcam.getTagBySpecificID(24);

        if (id20 != null) {
            telemetry.addLine("Tag 20 detected!");
            aprilTagWebcam.displayDetectionTelemetry(id20);
        } else {
            telemetry.addLine("Tag 20 not found");
        }

        telemetry.update();
    }

}
