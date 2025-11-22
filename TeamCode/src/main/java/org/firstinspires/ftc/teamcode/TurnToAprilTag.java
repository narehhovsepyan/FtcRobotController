//package org.firstinspires.ftc.teamcode;
//
//import android.annotation.SuppressLint;
//import android.graphics.Color;
//import android.util.Size;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.util.Range;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import org.firstinspires.ftc.vision.opencv.Circle;
//import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
//import org.firstinspires.ftc.vision.opencv.ColorRange;
//import org.firstinspires.ftc.vision.opencv.ImageRegion;
//
//import java.util.List;
//
//@Autonomous(name = "Auto: Turn to Artifact (Mecanum Centered)", group = "Concept")
//public class TurnToAprilTag extends LinearOpMode {
//
//    private DcMotor cameraMotor;
//
//    @SuppressLint("DefaultLocale")
//    @Override
//    public void runOpMode() {
//
//        // Initialize Mecanum motors
//        cameraMotor  = hardwareMap.get(DcMotor.class, "camera_motor");
//        cameraMotor.setDirection(DcMotor.Direction.FORWARD);
//        cameraMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        telemetry.setMsTransmissionInterval(100);
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//
//            // Get blobs
//            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//
//
//            if (!currentDetections.isEmpty()) {
//
//                // Find the largest blob (closest artifact)
//                ColorBlobLocatorProcessor.Blob target = currentDetections.get(0);
//                double maxRadius = target.getCircle().getRadius();
//                for (ColorBlobLocatorProcessor.Blob b : currentDetections) {
//                    if (b.getCircle().getRadius() > maxRadius) {
//                        target = b;
//                        maxRadius = b.getCircle().getRadius();
//                    }
//                }
//
//                Circle c = target.getCircle();
//                double offsetX = c.getX() - 160; // camera width = 320, center = 160
//
//                // Telemetry
//                String color = purpleBlobs.contains(target) ? "PURPLE" : "GREEN (FREE)";
//                telemetry.addLine("Closest Artifact: " + color);
//                telemetry.addLine(String.format("Offset X: %.2f pixels, Radius: %.2f", offsetX, c.getRadius()));
//
//                // Dead zone: stop turning if close to center
//                double deadZone = 10.0; // pixels
//                double turnPower = 0;
//
//                if (Math.abs(offsetX) > deadZone) {
//                    double k = 0.005; // proportional constant
//                    turnPower = Range.clip(k * offsetX, -0.3, 0.3);
//                }
//
//                // Apply turning power
//                frontLeft.setPower(turnPower);
//                backLeft.setPower(turnPower);
//                frontRight.setPower(turnPower);
//                backRight.setPower(turnPower);
//
//            } else {
//                telemetry.addLine("No Artifact Detected");
//
//                // Stop motors
//                frontLeft.setPower(0);
//                backLeft.setPower(0);
//                frontRight.setPower(0);
//                backRight.setPower(0);
//            }
//
//            telemetry.update();
//            sleep(50);
//        }
//    }
//}