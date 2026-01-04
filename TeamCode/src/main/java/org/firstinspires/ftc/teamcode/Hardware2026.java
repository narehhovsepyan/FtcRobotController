package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.util.Log;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

public class Hardware2026 {
    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    public DcMotor turntableMotor;
    public DcMotor shooterRight;
    public DcMotor shooterLeft;
    public CRServo spinTakeLeft;
    public CRServo spinTakeRight;
    public CRServo daisy;
    public AnalogInput axlePot;
    public NormalizedColorSensor colorSensor;


    // Define IMU object and headings (Make it private so it can't be accessed externally)
    public IMU imu = null;

    // Define some other variables for turning
    private double robotHeading = 0;
    private double headingOffset = 0;
    private double headingError = 0;
    private double targetHeading = 0;

    // Define camera things
    private AprilTagProcessor aprilTag;
    private AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    private VisionPortal visionPortal;
    private AprilTagDetection lastDetection = null;
    public static final boolean USE_WEBCAM = true;

    // Run time (public)
    private final ElapsedTime runtime = new ElapsedTime();
    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: our Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 100.0 / 25.4;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
//    static final double COUNTS_PER_DEGREE = 39.0/90.0;
    private double turnSpeed = 0;
    static final double P_TURN_GAIN = 0.02;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_GAIN = 0.02;     // Larger is more responsive, but also less stable
    static final double HEADING_THRESHOLD = 5.0;
    private static final int TARGET_TAG_ID = 24;
    private final double BEARING_TOLERANCE = 0.1;

    // Sorter constants
    static final double TOLERANCE = 0.08;  // Increased slightly for one-way reliability
    static final double SERVO_POWER = -0.7; // Speed for one-way travel
    static final double POSITION_ONE = 0.0;
    static final double POSITION_TWO = 0.83;
    static final double POSITION_THREE = 1.64;
    static final double[] POSITIONS = {POSITION_ONE, POSITION_TWO, POSITION_THREE};
    public static final int COLOR_NONE = 0, COLOR_GREEN = 1, COLOR_PURPLE = 2;
    private static final float GREEN_HUE_MIN = 85, GREEN_HUE_MAX = 165;
    private static final float PURPLE_HUE_MIN = 225, PURPLE_HUE_MAX = 365;
    int[] spotColors = {COLOR_NONE, COLOR_NONE, COLOR_NONE};
    boolean[] spotLocked = {false, false, false};
    int[] shootingOrder = {COLOR_NONE, COLOR_NONE, COLOR_NONE};

    int greenCount = 0, purpleCount = 0;
    final int STABLE_NEEDED = 5;

    // Create an instance of the otos sensor
    SparkFunOTOS myOtos;

    // daisy things
    ElapsedTime daisyTimer = new ElapsedTime();
    boolean daisyIsSpinning = false;
    int daisySpinDuration = 0;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public Hardware2026(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */

    public void init() {

        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        leftFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "left_back_drive");//port2
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_front_drive");//port0
        rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_back_drive");//port1
        turntableMotor = myOpMode.hardwareMap.get(DcMotor.class, "camera_motor");//port2
        shooterRight = myOpMode.hardwareMap.get(DcMotor.class, "shooter_right");//port1
        shooterLeft = myOpMode.hardwareMap.get(DcMotor.class, "shooter_left");//port0
        spinTakeRight = myOpMode.hardwareMap.get(CRServo.class, "spin_take_right");
        spinTakeLeft = myOpMode.hardwareMap.get(CRServo.class, "spin_take_left");
        daisy = myOpMode.hardwareMap.get(CRServo.class, "turnServo");
        axlePot = myOpMode.hardwareMap.get(AnalogInput.class, "axlePot");
        colorSensor = myOpMode.hardwareMap.get(NormalizedColorSensor.class, "sensor_color");


        turntableMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turntableMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turntableMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turntableMotor.setPower(0);

        myOtos = myOpMode.hardwareMap.get(SparkFunOTOS.class, "sensor_otos"); //Otos sensor
        configureOtos();

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        turntableMotor.setDirection(DcMotor.Direction.FORWARD);
        shooterRight.setDirection(DcMotor.Direction.REVERSE);
        shooterLeft.setDirection(DcMotor.Direction.FORWARD);

        // Retrieve the IMU from the hardware map
        imu = myOpMode.hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.DOWN));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

//        if (colorSensor instanceof SwitchableLight) ((SwitchableLight) colorSensor).enableLight(true);

        initAprilTag();

        resetHeading();
        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    public void initAprilTag() {

        aprilTag = new AprilTagProcessor.Builder()
                .setLensIntrinsics(
                        686.02605736, 686.02605736,
                        341.26208637, 208.068001489
                )
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(myOpMode.hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(aprilTag);

//        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
//        visionPortal = VisionPortal.easyCreateWithDefaults(
//                myOpMode.hardwareMap.get(WebcamName.class, "Webcam 1"),
//                aprilTag
//        );

        visionPortal = builder.build();

        // FIX: you forgot this!
        aprilTagWebcam = new AprilTagWebcam();

    }

    public AprilTagDetection getLatestDetection() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (detections.size() > 0)
            lastDetection = detections.get(0);
        return lastDetection;
    }

    public boolean checkIfDetected(int tagID) {
        boolean tagFound = false; // This boolean will be true if the specific tag is detected

        List<AprilTagDetection> detections = aprilTag.getDetections();

        // Step through the list of detections and check for the desired tag ID
        for (AprilTagDetection detection : detections) {
            if (detection.id == tagID) {
                // The specific tag was found
                tagFound = true;
                break; // No need to continue the loop once found
            }
        }
        return tagFound;
    }

    public double getTagBearing() {
        AprilTagDetection tag = getLatestDetection();
        if (tag != null) {
            return tag.ftcPose.bearing;
        }
        return Double.NaN;
    }

    public void stopCamera() {
        if (visionPortal != null) {
            visionPortal.stopStreaming();   // stop sending frames to processors
            visionPortal.close();           // releases webcam and processors
        }
    }

    // OTOS!
    public void configureOtos() {
        myOpMode.telemetry.addLine("Configuring OTOS...");
        myOpMode.telemetry.update();

        // Set the desired units for linear and angular measurements- currently set to the default inches and degrees
        myOtos.setLinearUnit(DistanceUnit.INCH);
        myOtos.setAngularUnit(AngleUnit.DEGREES);

        //Specify the offset for the sensor relative to the center of the robot.- chnsge
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(.25, 4.5, 0);
        myOtos.setOffset(offset);

        // Set the linear and angular scalars, to compensate for scaling issues with the sensor measurements.
        myOtos.setLinearScalar(1.0);
        myOtos.setAngularScalar(1.0);
        myOtos.calibrateImu();

        // Reset the tracking algorithm
        myOtos.resetTracking();

        // After resetting the tracking, the OTOS reports the robot is at origin. If you do not start at the origin, set the OTOS location to match.
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        myOtos.getVersionInfo(hwVersion, fwVersion);

        myOpMode.telemetry.addLine("OTOS configured! Press start to get position data!");
        myOpMode.telemetry.addLine();
        myOpMode.telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
        myOpMode.telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
        myOpMode.telemetry.update();
    }


    private double calculatePID(double error, double prevError, double integral, double kP, double kI, double kD, double loopDelay) {
        double derivative = (error - prevError) / loopDelay; //this is in seconds- should it be
        // Clip derivative to prevent spikes
        //derivative = Range.clip(derivative, -0.1, 0.1);
        double output = (kP * error) + (kI * integral) + (kD * derivative);
        Log.i("FTC18 calculatePID", String.format("PID output- error %.2f, integral: %.2f, derivative: %.2f, output: %.2f", error, integral, derivative, output));
        return output;
    }


    public void driveByOtos(double distanceToTravelX, double distanceToTravelY, double targetHeadingDegrees, double timeout) {
        runtime.reset();
        //distanceToTravelX *= 0.32;
        //distanceToTravelY *= 1.1;
        // Get initial position from OTOS sensor
        SparkFunOTOS.Pose2D pos = myOtos.getPosition();
        Translation2d currentT = new Translation2d(pos.x, pos.y);
        Rotation2d currentR = Rotation2d.fromDegrees(pos.h);
        Pose2d current = new Pose2d(currentT, currentR);

        // Compute target position
        Translation2d targetT = currentT.plus(new Translation2d(distanceToTravelX, distanceToTravelY));
        Rotation2d targetR = Rotation2d.fromDegrees(targetHeadingDegrees);
        Pose2d target = new Pose2d(targetT, targetR);

        // Use smaller thresholds for a smoother preciser stop
        double thresholdDistance = 1.0;
        double angleThreshold = 0.5; //radians should be smaller
        double correctionX = 0, correctionY = 0, correctionH = 0;

        double integralX = 0, prevErrorX = 0;
        double integralY = 0, prevErrorY = 0;
        double integralH = 0, prevErrorH = 0;

        // PID constants (tweak these as needed)
        //not sure if integeral works
        double kP_X = 0.25, kI_X = 0, kD_X = 0.02;
        double kP_Y = 0.25, kI_Y = 0, kD_Y = 0.02;
        double kP_H = .25, kI_H = 0, kD_H = 0.0;

        // So the robot can overcome friction
        double minPower = 0.2;

        double lastTime = runtime.seconds();
        double toTravelPrevA = 0;
        double toTravelPrevB = 0;
        double toTravelPrevC = 0;

        while (runtime.seconds() < timeout && myOpMode.opModeIsActive()) {
            double now = runtime.seconds();

            double loopDelay = (now - lastTime);  // seconds
            lastTime = now;
            if (loopDelay < 0.001) {
                loopDelay = 0.04;  //to prevent it being so small there is a NaN error- maybe not needed now
            }
            pos = myOtos.getPosition();


            currentT = new Translation2d(pos.x, pos.y);
            currentR = Rotation2d.fromDegrees(pos.h);
            current = new Pose2d(currentT, currentR);

            Translation2d toTravelT = target.getTranslation().minus(current.getTranslation());
            Rotation2d toTravelR = target.getRotation().minus(current.getRotation());

            double toTravelD = current.getTranslation().getDistance(target.getTranslation());


            // If within thresholds, exit the loop
            //if (toTravelAvg < thresholdDistance && toTravelD < thresholdDistance && Math.abs(toTravelR.getRadians()) < angleThreshold) {

            if (Math.abs(toTravelPrevC) < thresholdDistance && Math.abs(toTravelPrevB) < thresholdDistance && Math.abs(toTravelPrevA) < thresholdDistance && Math.abs(toTravelD) < thresholdDistance && Math.abs(toTravelR.getRadians()) < angleThreshold) {

                Log.i("FTC18 driveByOtos", String.format("Leaving loop - X: %.2f, Y: %.2f, H: %.2f", pos.x, pos.y, pos.h));
                break;
            }

            toTravelPrevC = toTravelPrevB;
            toTravelPrevB = toTravelPrevA;
            toTravelPrevA = toTravelD;




            double errorX = toTravelT.getX();
            double errorY = toTravelT.getY();
            double errorH = toTravelR.getRadians();

            // Anti-windup: only accumulate integral if error is significant (deadband)
            if (Math.abs(errorX) > 0.05) {
                integralX += errorX * loopDelay;
            } else {
                integralX = 0;
            }
            if (Math.abs(errorY) > 0.05) {
                integralY += errorY * loopDelay;
            } else {
                integralY = 0;
            }
            if (Math.abs(errorH) > 0.05) {
                integralH += errorH * loopDelay;
            } else {
                integralH = 0;
            }

            correctionX = calculatePID(errorX, prevErrorX, integralX, kP_X, kI_X, kD_X, loopDelay);
            correctionY = calculatePID(errorY, prevErrorY, integralY, kP_Y, kI_Y, kD_Y, loopDelay);
            correctionH = calculatePID(errorH, prevErrorH, integralH, kP_H, kI_H, kD_H, loopDelay);

            // Normalize X-Y correction if needed
            double magnitude = Math.hypot(correctionX, correctionY);
            if (magnitude > 1.0) {
                correctionX /= magnitude;
                correctionY /= magnitude;
            }

            double scale = Math.min(1.0, toTravelD / (thresholdDistance * 2.0));
            correctionX *= scale;
            correctionY *= scale;

            // Apply a deadband to small correction outputs
            // if (Math.abs(correctionX) < 0.5) correctionX = 0;
            //if (Math.abs(correctionY) < 0.5) correctionY = 0;
            //if (Math.abs(correctionH) < 0.25) correctionH = 0;


            correctionX = Math.signum(correctionX) * Math.max(Math.abs(correctionX), minPower);
            correctionY = Math.signum(correctionY) * Math.max(Math.abs(correctionY), minPower);

            correctionX = Range.clip(correctionX, -1.0, 1.0);
            correctionY = Range.clip(correctionY, -1.0, 1.0);
            correctionH = Range.clip(correctionH, -1.0, 1.0);

            Log.i("FTC18 driveByOtos[loop]", String.format("Drive: X=%.2f, Y=%.2f, H=%.2f", correctionX, correctionY, correctionH));
            //negative h and it now goes in the right direction
            driveRobotFC(-correctionY, -correctionX, -correctionH);

            prevErrorX = errorX;
            prevErrorY = errorY;
            prevErrorH = errorH;



            Log.i("FTC18 driveByOtos", String.format("Current Pos - X: %.2f, Y: %.2f, H: %.2f", pos.x, pos.y, pos.h));
            Log.i("FTC18 driveByOtos", String.format("Target Pos  - X: %.2f, Y: %.2f, H: %.2f", target.getTranslation().getX(), target.getTranslation().getY(), target.getRotation().getDegrees()));
            Log.i("FTC18 driveByOtos", String.format("Distance to Target: %.2f, Heading Error: %.2f", toTravelD, errorH));

            myOpMode.telemetry.addData("Current Position", "X=%.2f Y=%.2f H=%.2f", pos.x, pos.y, pos.h);
            myOpMode.telemetry.addData("Target Position", "X=%.2f Y=%.2f H=%.2f", target.getTranslation().getX(), target.getTranslation().getY(), target.getRotation().getDegrees());
            myOpMode.telemetry.addData("Distance to Target", "%.2f", toTravelD);
            myOpMode.telemetry.addData("Heading Error", "%.2f", errorH);
            myOpMode.telemetry.addData("Corrections", "X=%.2f Y=%.2f H=%.2f", correctionX, correctionY, correctionH);
            myOpMode.telemetry.update();

            // myOpMode.sleep(20);

        }
        stop();

        myOpMode.sleep(200);
    }

    public void driveWithOtos() {
        SparkFunOTOS.Pose2D pos = myOtos.getPosition(); // Get position

        myOpMode.telemetry.addData("X coordinate", pos.x); // Log the position to the telemetry
        myOpMode.telemetry.addData("Y coordinate", pos.y);
        myOpMode.telemetry.addData("Heading angle", pos.h);

        //myOpMode.telemetry.update();  Update the telemetry on the driver station
    }
    //Goes straight by encoder (takes distance)

    public void straightByEncoder(double speed, double distance, double timeout) {
        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;

        if (myOpMode.opModeIsActive()) {
            DcMotor.RunMode oldMotorMode = leftFrontDrive.getMode();

            setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = leftFrontDrive.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newLeftBackTarget = leftBackDrive.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newRightFrontTarget = rightFrontDrive.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newRightBackTarget = rightFrontDrive.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            leftFrontDrive.setTargetPosition(newLeftFrontTarget);
            leftBackDrive.setTargetPosition(newLeftBackTarget);
            rightFrontDrive.setTargetPosition(newRightFrontTarget);
            rightBackDrive.setTargetPosition(newRightBackTarget);

            setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            leftFrontDrive.setPower(Math.abs(speed));
            leftBackDrive.setPower(Math.abs(speed));
            rightFrontDrive.setPower(Math.abs(speed));
            rightBackDrive.setPower(Math.abs(speed));

            while (myOpMode.opModeIsActive() &&
                    (runtime.seconds() < timeout) &&
                    (leftFrontDrive.isBusy() && leftBackDrive.isBusy() && rightFrontDrive.isBusy() && rightBackDrive.isBusy())) {

                // Display the data for the driver.
                myOpMode.telemetry.addData("Running to", " lf:%7d lb:%7d rf:%7d rb:%7d", newLeftFrontTarget, newLeftBackTarget, newRightFrontTarget, newRightBackTarget);
                myOpMode.telemetry.addData("Currently at", " at lf:%7d lb:%7d rf:%7d rb:%7d",
                        leftFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
                //myOpMode.telemetry.update();
            }

            stop();
            setMotorMode(oldMotorMode);
            myOpMode.sleep(500);
        }
    }

    public void strafeByEncoder(double speed, double distance, double timeout) {
        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;

        if (myOpMode.opModeIsActive()) {
            DcMotor.RunMode oldMotorMode = leftFrontDrive.getMode();

            setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Determine new target position, and pass to motor controller
            // For strafing, left side motors move in opposite direction to right side motors
            newLeftFrontTarget = leftFrontDrive.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newLeftBackTarget = leftBackDrive.getCurrentPosition() - (int) (distance * COUNTS_PER_INCH); // Reversed
            newRightFrontTarget = rightFrontDrive.getCurrentPosition() - (int) (distance * COUNTS_PER_INCH); // Reversed
            newRightBackTarget = rightBackDrive.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);

            leftFrontDrive.setTargetPosition(newLeftFrontTarget);
            leftBackDrive.setTargetPosition(newLeftBackTarget);
            rightFrontDrive.setTargetPosition(newRightFrontTarget);
            rightBackDrive.setTargetPosition(newRightBackTarget);

            setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            leftFrontDrive.setPower(Math.abs(speed));
            leftBackDrive.setPower(Math.abs(speed));
            rightFrontDrive.setPower(Math.abs(speed));
            rightBackDrive.setPower(Math.abs(speed));

            while (myOpMode.opModeIsActive() &&
                    (runtime.seconds() < timeout) &&
                    (leftFrontDrive.isBusy() && leftBackDrive.isBusy() && rightFrontDrive.isBusy() && rightBackDrive.isBusy())) {

                // Display the data for the driver.
                myOpMode.telemetry.addData("Running to", " lf:%7d lb:%7d rf:%7d rb:%7d", newLeftFrontTarget, newLeftBackTarget, newRightFrontTarget, newRightBackTarget);
                myOpMode.telemetry.addData("Currently at", " at lf:%7d lb:%7d rf:%7d rb:%7d",
                        leftFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
                //myOpMode.telemetry.update();
            }

            stop();
            setMotorMode(oldMotorMode);
            myOpMode.sleep(500);
        }
    }

    public void driveVectorByEncoder(double speed, double forwardInches, double strafeInches, double turnDegrees, double timeout) {
        if (!myOpMode.opModeIsActive()) return;

        DcMotor.RunMode oldMotorMode = leftFrontDrive.getMode();

        // Reset encoders
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Convert turn degrees into wheel "inches"
        // TUNE this constant for your robot radius
        final double INCHES_PER_DEGREE = 0.10;
        double turnInches = turnDegrees * INCHES_PER_DEGREE;

        // Convert translational motion to wheel distances
        // Standard mecanum forward/strafe equations
        double lfDist =  forwardInches + strafeInches + turnInches;
        double rfDist =  forwardInches - strafeInches - turnInches;
        double lbDist =  forwardInches - strafeInches + turnInches;
        double rbDist =  forwardInches + strafeInches - turnInches;

        // Convert to encoder counts
        int lfTarget = leftFrontDrive.getCurrentPosition() + (int)(lfDist * COUNTS_PER_INCH);
        int rfTarget = rightFrontDrive.getCurrentPosition() + (int)(rfDist * COUNTS_PER_INCH);
        int lbTarget = leftBackDrive.getCurrentPosition()  + (int)(lbDist * COUNTS_PER_INCH);
        int rbTarget = rightBackDrive.getCurrentPosition() + (int)(rbDist * COUNTS_PER_INCH);

        // Apply target positions
        leftFrontDrive.setTargetPosition(lfTarget);
        rightFrontDrive.setTargetPosition(rfTarget);
        leftBackDrive.setTargetPosition(lbTarget);
        rightBackDrive.setTargetPosition(rbTarget);

        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Normalize power so no motor exceeds speed
        double max = Math.max(Math.max(Math.abs(lfDist), Math.abs(rfDist)), Math.max(Math.abs(lbDist), Math.abs(rbDist)));
        double scale = 1.0;

        leftFrontDrive.setPower(Math.abs(speed * scale));
        leftBackDrive.setPower(Math.abs(speed * scale));
        rightFrontDrive.setPower(Math.abs(speed * scale));
        rightBackDrive.setPower(Math.abs(speed * scale));

        runtime.reset();
        while (myOpMode.opModeIsActive() &&
                runtime.seconds() < timeout &&
                (leftFrontDrive.isBusy() || rightFrontDrive.isBusy() ||
                        leftBackDrive.isBusy()  || rightBackDrive.isBusy())) {

            myOpMode.telemetry.addData("Targets", "LF:%d RF:%d LB:%d RB:%d",
                    lfTarget, rfTarget, lbTarget, rbTarget);
            myOpMode.telemetry.addData("Current", "LF:%d RF:%d LB:%d RB:%d",
                    leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition(),
                    leftBackDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
            // myOpMode.telemetry.update();
        }

        stop();
        setMotorMode(oldMotorMode);
        myOpMode.sleep(300);
    }

    //Drives for a set amount of time (takes time)
    public void driveTimed(double axial, double lateral, double yaw, double time) {
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveRobot(axial, lateral, yaw);
        runtime.reset();
        while (myOpMode.opModeIsActive() && (runtime.seconds() < time)) {
            myOpMode.telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            //myOpMode.telemetry.update();
        }
        stop();
    }

    //strafe
    public void strafe(double strafe_power) {
        driveRobot(0.0, strafe_power, 0.0);
    }

    //strafe for a set amount of time (takes time)
    public void strafeTimed(double lateral, double time) {
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveRobot(0, lateral, 0);
        runtime.reset();
        while (myOpMode.opModeIsActive() && (runtime.seconds() < time)) {
            myOpMode.telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            myOpMode.telemetry.update();
        }
        stop();
    }

    //drives robot
    public void driveRobot(double axial, double lateral, double yaw) {
        double max;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
        myOpMode.telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        myOpMode.telemetry.addData("Back left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
    }

    //field centric
    public void driveRobotFC(double axial, double lateral, double yaw) {
        double y = axial;
        double x = lateral;
        double rx = yaw;
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS); //radians vs degrees
        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        rotX = rotX * 1.1;  // Counteract imperfect strafing
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;
        Log.i("FTC18 driveRobotFC", String.format("fl=%f fr=%f", frontLeftPower, frontRightPower));
        Log.i("FTC18 driveRobotFC", String.format("bl=%f br=%f", backLeftPower, backRightPower));
        leftFrontDrive.setPower(frontLeftPower);
        leftBackDrive.setPower(backLeftPower);
        rightFrontDrive.setPower(frontRightPower);
        rightBackDrive.setPower(backRightPower);
    }
    public void setMotorMode(DcMotor.RunMode motorMode) {
        leftFrontDrive.setMode(motorMode);
        leftBackDrive.setMode(motorMode);
        rightFrontDrive.setMode(motorMode);
        rightBackDrive.setMode(motorMode);
    }

    public double getRawHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void resetHeading() {
        // Save a new heading offset equal to the current raw heading.
        headingOffset = getRawHeading();
        robotHeading = 0;
    }

    public void resetYaw() {
        imu.resetYaw();
    }
    public void turnToHeading(double maxTurnSpeed, double heading) {
        getSteeringCorrection(heading, P_DRIVE_GAIN);
        while (myOpMode.opModeIsActive() && (((Math.abs(headingError - 180)) > HEADING_THRESHOLD))) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);
            myOpMode.telemetry.addData("Heading- Target : Current", "%5.2f : %5.0f", targetHeading, (getRawHeading() - headingOffset)); // Clip the speed to the maximum permitted value.
            myOpMode.telemetry.update();
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);
            // Pivot in place by applying the turning correction
            driveRobot(0, 0, turnSpeed);
        }
        stop();
    }

    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry
        // Get the robot heading by applying an offset to the IMU heading
        robotHeading = getRawHeading() - headingOffset;
        // Determine the heading current error
        headingError = targetHeading - robotHeading;
        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;
        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    public void stop() {
        driveRobot(0, 0, 0);
    }

    public void straight(double power) {
        driveRobot(power, 0, 0);
    }

    public void straightTimed(double power, double time) {
        driveTimed(power, 0, 0, time);
    }

    public void driveDiagonalForTime(double forwardPower, double strafePower, double time) {
        driveTimed(forwardPower, strafePower, 0, time);
    }
    public void stopRobot(){
        leftFrontDrive.setPower(0.01);
        leftBackDrive.setPower(0.01);
        rightFrontDrive.setPower(0.01);
        rightBackDrive.setPower(0.01);
    }

    // Decode 2026 ----------------------------------------------------------------------------
    public static class PID {
        private double kP, kI, kD;
        private double integral = 0.0;
        private double lastError = 0.0;
        private double derivativeFilter = 0.0;
        private final double derivTau = 0.02; // seconds

        public PID(double kP, double kI, double kD) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
        }

        public double update(double error, double dt) {
            if (dt <= 0) return 0.0;

            integral += error * dt;

            double rawDeriv = (error - lastError) / dt;
            double alpha = dt / (derivTau + dt);

            derivativeFilter += alpha * (rawDeriv - derivativeFilter);
            lastError = error;

            return (kP * error) + (kI * integral) + (kD * derivativeFilter);
        }

        public void reset() {
            integral = 0.0;
            lastError = 0.0;
            derivativeFilter = 0.0;
        }

        public void clampIntegral(double min, double max) {
            if (integral > max) integral = max;
            if (integral < min) integral = min;
        }
    }
    private PID turretPID = new PID(0.006, 0.00005, 0.00045);

    private static final double MAX_POWER = 0.20;
    private static final double DEADBAND_DEG = 4.0;
    private static final double LPF_ALPHA = 0.6;
    private double filteredBearing = 0.0;
    private long lastNs = System.nanoTime();
    public boolean updateTurretToAprilTag(int tagId) {

        long nowNs = System.nanoTime();
        double dt = (nowNs - lastNs) / 1e9;
        lastNs = nowNs;

        if (dt > 0.1) {
            turretPID.reset();
            return false;
        }

        List<AprilTagDetection> detections = aprilTag.getDetections();
        AprilTagDetection target = null;

        for (AprilTagDetection d : detections) {
            if (d.id == tagId) {
                target = d;
                break;
            }
        }

        if (target == null || target.ftcPose == null) {
            turntableMotor.setPower(0.0);
            turretPID.reset();
            return false;
        }

        double rawBearing = target.ftcPose.bearing;

        filteredBearing =
                (LPF_ALPHA * filteredBearing)
                        + ((1.0 - LPF_ALPHA) * rawBearing);

        double error = filteredBearing;

        if (Math.abs(error) <= DEADBAND_DEG) {
            turntableMotor.setPower(0.0);
            turretPID.reset();
            return true;
        }

        double power = turretPID.update(error, dt);
        turretPID.clampIntegral(-100.0, 100.0);

        power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));
        if (Math.abs(power) < 0.02) power = 0.0;

        turntableMotor.setPower(power);
        return false;
    }
    public void updateTurntableToFaceTarget() {
        double TARGET_FIELD_X = 5.5*24;   // your target X in field coordinates (inches or units)
        double TARGET_FIELD_Y = 5.5*24;   // your target Y
        double ROBOT_START_FIELD_X = 2.5*24; // robot start X
        double ROBOT_START_FIELD_Y = 1;  // robot start Y

        SparkFunOTOS.Pose2D pos = myOtos.getPosition();
        if (pos == null) return;

        double robotX = pos.x + ROBOT_START_FIELD_X;
        double robotY = pos.y + ROBOT_START_FIELD_Y;
        double robotHeading = ((pos.h % 360) + 360) % 360;

        double targetX = TARGET_FIELD_X - robotX;
        double targetY = TARGET_FIELD_Y - robotY;

        double angleToTarget = Math.toDegrees(Math.atan2(targetY, targetX));

        double error = angleToTarget - robotHeading;

        if (Math.hypot(targetX, targetY) < 2.0) return;
        if (Math.abs(error) < 1.5) error = 0.0;

        error = ((error + 540) % 360) - 180;

        myOpMode.telemetry.addData("Robot Pos", "X=%.2f Y=%.2f H=%.2f", robotX, robotY, robotHeading);
        myOpMode.telemetry.addData("Target OTOS", "X=%.2f Y=%.2f", targetX, targetY);
        myOpMode.telemetry.addData("Angle Error", "%.2f", error);

        setTurntableAngle(error,.8);

    }
    public void setTurntableAngle(double desiredAngleDegrees) {

        double countsPerDegree = 39.0 / 90.0;
        double toleranceDegrees = 2.0; // ±2 degrees wiggle room
        int toleranceCounts = (int)Math.round(toleranceDegrees * countsPerDegree);

        int targetCounts = (int)Math.round(desiredAngleDegrees * countsPerDegree);
        int currentCounts = turntableMotor.getCurrentPosition();

        double power = 1.0;

        if (Math.abs(targetCounts - currentCounts) > toleranceCounts) {
            turntableMotor.setTargetPosition(targetCounts);
            turntableMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            turntableMotor.setPower(power);
        } else {
            turntableMotor.setPower(0); // stop if within tolerance
        }

        myOpMode.telemetry.addData("Turret Angle (deg)", desiredAngleDegrees);
        myOpMode.telemetry.addData("Turret Target Counts", targetCounts);
        myOpMode.telemetry.addData("Turret Current Counts", currentCounts);
        myOpMode.telemetry.addData("Within Tolerance", Math.abs(targetCounts - currentCounts) <= toleranceCounts);
        myOpMode.telemetry.update();
    }

    public static final double MOTOR_ENCODER_PPR = 7.0;
    public static final double GEAR_RATIO = 90.0 / 15.0; // 6.0
    public static final double COUNTS_PER_OUTPUT_REV = MOTOR_ENCODER_PPR * GEAR_RATIO; // 42
    public static final double COUNTS_PER_DEGREE = COUNTS_PER_OUTPUT_REV / 360.0; // 0.1166667

    public void setTurntableAngle(double targetDegrees, double maxPower) {

        // Convert degrees to encoder counts
        int targetCounts = (int) Math.round(targetDegrees * .42);

        turntableMotor.setTargetPosition(targetCounts);
        turntableMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Ensure power is always positive for RUN_TO_POSITION
        turntableMotor.setPower(Math.abs(maxPower));
    }
    public void spinTake(double intake_power) {
        spinTakeRight.setPower(-intake_power);
        spinTakeLeft.setPower(intake_power);
    }
    public void stopSpinTake() {
        spinTakeRight.setPower(0);
        spinTakeLeft.setPower(0);
    }

    public void daisySpin(double daisy_power, int artifacts) {
        daisy.setPower(daisy_power);
        int time = 450 * artifacts;
        myOpMode.sleep(time);
        daisy.setPower(0);
    }

    public void startDaisySpin(double daisy_power, int artifacts) {
        daisySpinDuration = 450 * artifacts; // how long to run
        daisy.setPower(daisy_power);
        daisyTimer.reset();
        daisyIsSpinning = true;
    }

    public void updateDaisySpin() {
        if (daisyIsSpinning) {
            if (daisyTimer.milliseconds() >= daisySpinDuration) {
//                daisy.setPower(0);
                daisyIsSpinning = false;
            }
        }
    }

    public void turntable(double intake_power) {
        turntableMotor.setPower(intake_power);
    }

    public void moveToVoltage(double target) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (myOpMode.opModeIsActive() && timer.seconds() < 3.0) { // 3-second safety timeout
            double current = axlePot.getVoltage();

            // Check if we reached target
            if (Math.abs(target - current) < TOLERANCE) {
                break;
            }

            // ALWAYS POSITIVE POWER - Only moves forward
            daisy.setPower(SERVO_POWER);

            if (myOpMode.gamepad1.left_bumper) break; // Manual emergency stop

            myOpMode.telemetry.addData("Seeking Target", target);
            myOpMode.telemetry.addData("Current Volt", "%.3f", current);
            myOpMode.telemetry.update();
        }
        daisy.setPower(0);
    }

    public int getClosestPosition(double voltage) {
        for (int i = 0; i < POSITIONS.length; i++) {
            if (Math.abs(voltage - POSITIONS[i]) < TOLERANCE) return i;
        }
        return -1;
    }

    public void scanCurrentSpot(int index) {
        NormalizedRGBA c = colorSensor.getNormalizedColors();
        float[] hsv = new float[3];
        Color.colorToHSV(c.toColor(), hsv);
        float hue = hsv[0];

        if (hue >= GREEN_HUE_MIN && hue <= GREEN_HUE_MAX) {
            greenCount++; purpleCount = 0;
            if (greenCount >= STABLE_NEEDED) {
                spotColors[index] = COLOR_GREEN;
                spotLocked[index] = true;
            }
        } else if (hue >= PURPLE_HUE_MIN && hue <= PURPLE_HUE_MAX) {
            purpleCount++; greenCount = 0;
            if (purpleCount >= STABLE_NEEDED) {
                spotColors[index] = COLOR_PURPLE;
                spotLocked[index] = true;
            }
        } else {
            greenCount = 0; purpleCount = 0;
        }
    }

    public void runAutoLaunch() {
        if (shootingOrder[0] == COLOR_NONE) return;

        for (int targetColor : shootingOrder) {
            for (int i = 0; i < 3; i++) {
                if (spotColors[i] == targetColor && spotLocked[i]) {
                    moveToVoltage(POSITIONS[i]);
                    shootBall();
                    myOpMode.sleep(800);
                    spotColors[i] = COLOR_NONE;
                    spotLocked[i] = false;
                    break;
                }
            }
        }
    }
    public void shootBall() {
        ///open lancher door
        startLauncher();
        myOpMode.sleep(500);
        double currentVoltage = axlePot.getVoltage();
        int currentPosIndex = getClosestPosition(currentVoltage);

        // If we are between spots, find the mathematically closest index
        if (currentPosIndex == -1) {
            double minDiff = Double.MAX_VALUE;
            for (int i = 0; i < POSITIONS.length; i++) {
                double diff = Math.abs(currentVoltage - POSITIONS[i]);
                if (diff < minDiff) {
                    minDiff = diff;
                    currentPosIndex = i;
                }
            }
        }


        // Determine the next position in the sequence (0 -> 1 -> 2 -> 0)
        int nextIndex = (currentPosIndex + 1) % POSITIONS.length;

        // Execute the movement using your one-way logic
        moveToVoltage(POSITIONS[nextIndex]);
        myOpMode.sleep(500);

        stopLauncher();
        /// close launcher door
    }

    public void shootArtifact(double shooterPower) {
        shooterRight.setPower(shooterPower);
        shooterLeft.setPower(shooterPower);
    }
    public void startLauncher(){
        shooterRight.setPower(.7);
        shooterLeft.setPower(.7);
    }
    public void stopLauncher(){
        shooterRight.setPower(0);
        shooterRight.setPower(0);
    }

    public void resetSystem() {
        for (int i = 0; i < 3; i++) {
            spotColors[i] = COLOR_NONE;
            spotLocked[i] = false;
        }
        greenCount = 0; purpleCount = 0;
    }

    public void updateAprilTagOrder() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == 21) {
                shootingOrder[0] = COLOR_GREEN; shootingOrder[1] = COLOR_PURPLE; shootingOrder[2] = COLOR_PURPLE;

            } else if (detection.id == 22) {
                shootingOrder[0] = COLOR_PURPLE; shootingOrder[1] = COLOR_GREEN; shootingOrder[2] = COLOR_PURPLE;

            } else if (detection.id == 23) {
                shootingOrder[0] = COLOR_PURPLE; shootingOrder[1] = COLOR_PURPLE; shootingOrder[2] = COLOR_GREEN;

            }
        }
        myOpMode.telemetry.update();
    }

    public void updateTelemetry(double volt, int pos) {
        myOpMode.telemetry.addData("Voltage", "%.2f", volt);
        myOpMode.telemetry.addData("Position", pos == -1 ? "MOVING" : (pos + 1));
        myOpMode.telemetry.addLine("--- Spots ---");
        for (int i=0; i<3; i++) {
            myOpMode.telemetry.addData("Spot " + (i+1), spotLocked[i] ? colorName(spotColors[i]) : "EMPTY");
        }
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == 21) {
                myOpMode.telemetry.addData("Pattern", "GPP");
            } else if (detection.id == 22) {
                myOpMode.telemetry.addData("Pattern", "PGP");
            } else if (detection.id == 23) {
                myOpMode.telemetry.addData("Pattern", "PPG");
            }
        }
        myOpMode.telemetry.update();
        myOpMode.telemetry.update();

    }

    public String colorName(int c) {
        if (c == COLOR_GREEN) return "GREEN";
        if (c == COLOR_PURPLE) return "PURPLE";
        return "NONE";
    }

    public void waitAndSpin() {
        int nextIndex = (getClosestPosition(axlePot.getVoltage()) + 1) % POSITIONS.length;
        moveToVoltage(POSITIONS[nextIndex]);
    }

    public void shootPurple() {
        for (int i = 0; i < POSITIONS.length; i++) {
            if (spotLocked[i] && spotColors[i] == COLOR_PURPLE) {

                // Move to the purple artifact
                moveToVoltage(POSITIONS[i]);

                // Shoot it
                shootBall();

                // Clear that spot so it doesn't get reused
                spotColors[i] = COLOR_NONE;
                spotLocked[i] = false;

                return;
            }
        }
    }

    public void shootGreen() {
        for (int i = 0; i < POSITIONS.length; i++) {
            if (spotLocked[i] && spotColors[i] == COLOR_GREEN) {

                // Move to the purple artifact
                moveToVoltage(POSITIONS[i]);

                // Shoot it
                shootBall();

                // Clear that spot so it doesn't get reused
                spotColors[i] = COLOR_NONE;
                spotLocked[i] = false;

                return;
            }
        }
    }

    public String autoDecision() {
        String pattern = "";
        List<AprilTagDetection> detections = aprilTag.getDetections();

        for (AprilTagDetection detection : detections) {

            // Look for a specific tag ID
            if (detection.id == 21) {
                pattern = "GPP"; //GPP
                return pattern;
            } else if (detection.id == 22) {
                pattern = "PGP"; //PGP
                return pattern;
            } else if (detection.id == 23) {
                pattern = "PPG"; //PPG
                return pattern;
            } else {
                return pattern;
            }
        }   // end method telemetryAprilTag()
        return pattern;
    }

    //autonomous stuff!
    public void autoGPP() {
        //just shoot all three
        shootBall();
        shootBall();
        shootBall();
    }

    public void autoPGP() {
        waitAndSpin();
        shootBall();
        waitAndSpin();
        shootBall();
        waitAndSpin();
        shootBall();
        //wait and spin G, shoot P, wait and spin P, shoot G, empty spot spin, shoot P
    }

    public void autoPPG() {
        waitAndSpin();
        shootBall();
        shootBall();
        shootBall();
        //wait and spin G, shoot P and P and G
    }

}

