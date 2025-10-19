//package org.firstinspires.ftc.teamcode; // Make sure this is your team's package
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//
//import java.util.List;
//
//@Autonomous(name = "Autonomous Drive To AprilTag", group = "Concept")
//// @Disabled
//public class AutoDriveToPositionAprilTag extends LinearOpMode {
//
//    // --- TUNING CONSTANTS ---
//    // Adjust these numbers to suit your robot and desired behavior.
//
//    // How close the camera should get to the target (inches).
//    final double DESIRED_DISTANCE = 8.0;
//
//    // The ID of the AprilTag you want to drive to.
//    // Set to -1 to drive to any tag found.
//    private static final int DESIRED_TAG_ID = 21;
//
//    // Proportional gain constants for motion correction.
//    final double SPEED_GAIN = 0.03;   // Forward/backward speed control
//    final double STRAFE_GAIN = 0.02;  // Strafe speed control
//    final double TURN_GAIN = 0.02;    // Turn speed control
//
//    // Maximum power levels for autonomous movement.
//    final double MAX_AUTO_SPEED = 0.6;
//    final double MAX_AUTO_STRAFE = 0.6;
//    final double MAX_AUTO_TURN = 0.4;
//
//    // Safety timeout to prevent the robot from running forever.
//    final double RUNTIME_LIMIT_SEC = 20.0;
//
//    // --- STATE & HARDWARE ---
//    private enum DriveState {
//
//        SEARCHING,
//        DRIVING_TO_TAG,
//        DONE
//    }
//    private static final boolean USE_WEBCAM = true; // Set to true for webcam, false for phone camera
//    private DriveState currentState = DriveState.SEARCHING;
//    private ElapsedTime runtime = new ElapsedTime();
//
//    // Hardware
//    private DcMotor frontLeftDrive = null;
//    private DcMotor frontRightDrive = null;
//    private DcMotor backLeftDrive = null;
//    private DcMotor backRightDrive = null;
//
//    // Vision
//    private VisionPortal visionPortal;
//    private AprilTagProcessor aprilTag;
//    private AprilTagDetection desiredTag = null;
//
//    @Override
//    public void runOpMode() {
//        // --- INITIALIZATION ---
//
//        initAprilTag();
//
//        // Initialize hardware
//        frontLeftDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
//        frontRightDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
//        backLeftDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
//        backRightDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
//
//        // *** IMPORTANT: SET YOUR MOTOR DIRECTIONS HERE ***
//        // This example assumes REVERSE for left motors and FORWARD for right.
//        // Adjust these based on your robot's motor configuration.
//        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
//        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
//        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
//        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
//
//        // Set ZERO POWER BEHAVIOR
//        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        telemetry.addData("Status", "Initialized");
//        telemetry.addData("Target", "AprilTag ID %d", DESIRED_TAG_ID);
//        telemetry.addData(">", "Press START to begin.");
//        telemetry.update();
//
//        waitForStart();
//
//        // --- MAIN AUTONOMOUS LOOP ---
//        runtime.reset();
//        boolean targetFound = false; // Declare the variable once, before the loop.
//        while (opModeIsActive() && currentState != DriveState.DONE && runtime.seconds() < RUNTIME_LIMIT_SEC) {
//
//            desiredTag = null;
//            targetFound = findDesiredTag(); // Re-assign the value, don't re-declare with "boolean".
//
//
//            // State machine logic
//            switch (currentState) {
//                case SEARCHING:
//                    telemetry.addData("State", "SEARCHING");
//                    if (targetFound) {
//                        // Target found, switch to driving mode
//                        currentState = DriveState.DRIVING_TO_TAG;
//                    } else {
//                        // If you want the robot to turn to find the tag, add that logic here.
//                        // For this example, it will just wait.
//                        moveRobot(0, 0, 0); // Stop moving
//                    }
//                    break;
//
//                case DRIVING_TO_TAG:
//                    telemetry.addData("State", "DRIVING_TO_TAG");
//                    if (targetFound) {
//                        // Calculate errors
//                        double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
//                        double headingError = desiredTag.ftcPose.bearing;
//                        double yawError = desiredTag.ftcPose.yaw;
//                        // Use gains to calculate powers
//                        double drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
//                        double turn = Range.clip(-headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
//                        double strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
//
//                        // Apply powers
//                        moveRobot(drive, strafe, turn);
//
//                        telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f", drive, strafe, turn);
//                        telemetry.addData("Pose", "Range: %3.1f, Bearing: %3.1f, Yaw: %3.1f", desiredTag.ftcPose.range, desiredTag.ftcPose.bearing, desiredTag.ftcPose.yaw);
//
//                    } else {
//                        // Target lost, go back to searching
//                        currentState = DriveState.SEARCHING;
//                        moveRobot(0, 0, 0);
//                    }
//                    break;
//
//                case DONE:
//                    // Should not happen in this loop, but good practice.
//                    moveRobot(0, 0, 0);
//                    break;
//            }
//
//            telemetry.update();
//        }
//
//        // --- CLEANUP ---
//        moveRobot(0, 0, 0); // Ensure robot is stopped
//        telemetry.addData("Status", "OpMode Finished.");
//        telemetry.addData("Final State", currentState.toString());
//        telemetry.addData("Runtime", runtime.toString());
//        telemetry.update();
//    }
//
//    /**
//     * Finds the desired AprilTag from the list of current detections.
//     * @return true if the desired tag is found, false otherwise.
//     */
//    private boolean findDesiredTag() {
//        boolean targetFound = false;
//        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//
//        for (AprilTagDetection detection : currentDetections) {
//            // If we're looking for any tag OR this is the specific tag we're looking for
//            if (detection.metadata != null && (DESIRED_TAG_ID < 0 || detection.id == DESIRED_TAG_ID)) {
//                desiredTag = detection;
//                targetFound = true;
//                break; // Found the tag, no need to look further
//            }
//        }
//        return targetFound;
//    }
//
//    /**
//     * Initialize the AprilTag processor and VisionPortal.
//     */
//    /**
//     * Initialize the AprilTag processor.
//     */
//    private void initAprilTag() {
//        // Create the AprilTag processor by using a builder.
//        aprilTag = new AprilTagProcessor.Builder()
//
//                // These values are the camera's lens calibration parameters (fx, fy, cx, cy).
//                // Using correct lens intrinsics is crucial for accurate pose estimation.
//                .setLensIntrinsics(686.02605736, 686.02605736, 341.26208637, 208.068001489)
//
//                .build();
//
//
//        // Adjust Image Decimation to trade-off detection-range for detection-rate.
//        // e.g. Some typical detection data using a Logitech C920 WebCam
//        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
//        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
//        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
//        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
//        // Note: Decimation can be changed on-the-fly to adapt during a match.
//        aprilTag.setDecimation(2);
//
//        VisionPortal.Builder builder = new VisionPortal.Builder();
//
//        if (USE_WEBCAM) {
//            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
//        } else {
//            builder.setCamera(BuiltinCameraDirection.BACK);
//        }
//    }
//
//
//    /**
//     * Commands the robot to move based on desired drive, strafe, and turn inputs.
//     * @param drive  Forward/backward power (-1.0 to 1.0). Positive is forward.
//     * @param strafe Left/right power (-1.0 to 1.0). Positive is strafe LEFT.
//     * @param turn   Rotation power (-1.0 to 1.0). Positive is counter-clockwise.
//     */
//    public void moveRobot(double drive, double strafe, double turn) {
//        // Mecanum drive calculations
//        double frontLeftPower = drive + strafe + turn;
//        double frontRightPower = drive - strafe - turn;
//        double backLeftPower = drive - strafe + turn;
//        double backRightPower = drive + strafe - turn;
//
//        // Normalize wheel powers to be within [-1.0, 1.0]
//        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
//        max = Math.max(max, Math.abs(backLeftPower));
//        max = Math.max(max, Math.abs(backRightPower));
//
//        if (max > 1.0) {
//            frontLeftPower /= max;
//            frontRightPower /= max;
//            backLeftPower /= max;
//            backRightPower /= max;
//        }
//
//        // Send calculated powers to the motors
//        frontLeftDrive.setPower(frontLeftPower);
//        frontRightDrive.setPower(frontRightPower);
//        backLeftDrive.setPower(backLeftPower);
//        backRightDrive.setPower(backRightPower);
//    }
//
//    public void driveRobot(double drive, double strafe, double turn) {
//        // Mecanum drive calculations
//        double frontLeftPower = drive + strafe + turn;
//        double frontRightPower = drive - strafe - turn;
//        double backLeftPower = drive - strafe + turn;
//        double backRightPower = drive + strafe - turn;
//
//        // Normalize wheel powers to be within [-1.0, 1.0]
//        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
//        max = Math.max(max, Math.abs(backLeftPower));
//        max = Math.max(max, Math.abs(backRightPower));
//
//        if (max > 1.0) {
//            frontLeftPower /= max;
//            frontRightPower /= max;
//            backLeftPower /= max;
//            backRightPower /= max;
//        }
//
//        // Send calculated powers to the motors
//        frontLeftDrive.setPower(frontLeftPower);
//        frontRightDrive.setPower(frontRightPower);
//        backLeftDrive.setPower(backLeftPower);
//        backRightDrive.setPower(backRightPower);
//    }
//}
//}
