package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "WebcamTurnToAprilTag")
public class WebcamTurnToAprilTag extends LinearOpMode {

    private AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    private DcMotor cameraMotor;
    private static final int TARGET_TAG_ID = 24;
    private final double BEARING_TOLERANCE = 0.1;  // degrees — how close to zero counts as “aligned”

    public void runOpMode() {
        // Initialize hardware and webcam
        cameraMotor = hardwareMap.get(DcMotor.class, "camera_motor");
        aprilTagWebcam.init(hardwareMap, telemetry);
//        cameraServo.setPosition(servoPos);

        telemetry.addLine("AprilTag Yaw Alignment Ready");
        telemetry.update();

        // Wait for play button
        waitForStart();

        // Main loop: run until stop or yaw alignment complete
        while (opModeIsActive()) {
            aprilTagWebcam.update();

            AprilTagDetection tag = aprilTagWebcam.getTagBySpecificID(TARGET_TAG_ID);

            double power = 0.0;

            if (tag != null && tag.ftcPose != null) {
                double bearing = tag.ftcPose.bearing;

                telemetry.addData("Detected Tag", tag.id);
                telemetry.addData("Bearing (deg)", bearing);

                if (Math.abs(bearing) > BEARING_TOLERANCE) {
                    if (bearing < 0) {
                        power = 0.1;  // turn right
                    } else {
                        power = -0.1; // turn left
                    }
                } else {
                    power = 0.0;     // aligned
                }
            } else {
                // tag not found — optional slow scan
                power = 0.00;
            }


            cameraMotor.setPower(power);

            telemetry.update();

            // Small delay to avoid camera overload
//            sleep(10);
        }

        // Stop vision when OpMode ends
        aprilTagWebcam.stop();
    }
}

