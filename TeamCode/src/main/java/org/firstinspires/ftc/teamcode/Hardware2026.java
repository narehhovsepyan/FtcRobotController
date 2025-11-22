package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    public DcMotor cameraMotor;
    public CRServo spinTake;

    // Define IMU object and headings (Make it private so it can't be accessed externally)
    public IMU imu = null;
    private double robotHeading = 0;
    private double headingOffset = 0;
    private double headingError = 0;
    private double targetHeading = 0;

    // define camera things
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
    private double turnSpeed = 0;
    static final double P_TURN_GAIN = 0.02;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_GAIN = 0.02;     // Larger is more responsive, but also less stable
    static final double HEADING_THRESHOLD = 5.0;
    private static final int TARGET_TAG_ID = 24;
    private final double BEARING_TOLERANCE = 0.1;

    // Create an instance of the otos sensor
     SparkFunOTOS myOtos;

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
        leftBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_back_drive");
        cameraMotor = myOpMode.hardwareMap.get(DcMotor.class, "camera_motor");
        //cameraServo = myOpMode.hardwareMap.get(CRServo.class, "cameraServo");
        spinTake = myOpMode.hardwareMap.get(CRServo.class, "spin_take");
        myOtos = myOpMode.hardwareMap.get(SparkFunOTOS.class, "sensor_otos"); //Otos sensor
        configureOtos();

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        // 18-D-RC: ALL REVERSE
        // 18-C-RC: (demo bot) F
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        cameraMotor.setDirection(DcMotor.Direction.FORWARD);

        // Retrieve the IMU from the hardware map
        imu = myOpMode.hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        initAprilTag();

        resetHeading();
        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    //CAMERA STUFF!
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

    public double getTagBearing() {
        AprilTagDetection tag = getLatestDetection();
        if (tag != null) {
            return tag.ftcPose.bearing;
        }
        return Double.NaN;
    }
    public void turnToAprilTag(int targetId) {
        AprilTagDetection tag = null;
        List<AprilTagDetection> detections = aprilTag.getDetections();

        for (AprilTagDetection d : detections) {
            if (d.id == targetId) {
                tag = d;
                break;
            }
        }

        double power = 0.0;

        if (tag != null && tag.ftcPose != null) {
            double bearing = tag.ftcPose.bearing;

            myOpMode.telemetry.addData("Detected Tag", tag.id);
            myOpMode.telemetry.addData("Bearing (deg)", bearing);

            if (Math.abs(bearing) > BEARING_TOLERANCE) {
                if (bearing < 0) {
                    power = 0.2;  // turn right
                } else {
                    power = -0.2; // turn left
                }
            } else {
                power = 0.0;     // aligned
            }
        } else {
            // tag not found — optional slow scan
            power = 0.1;
        }


        cameraMotor.setPower(power);

        myOpMode.telemetry.update();

        // Small delay to avoid camera overload
//            sleep(10);
    }

    public void stabilizeCameraFieldForward() {
        // ---------- TUNING / CONFIG ----------
        final double kP = 1.0; // proportional gain, tune this for smooth movement
        final double MAX_POWER = 1.0; // max motor power
        final double MIN_POWER = -1.0; // min motor power

        // ---------- GET ROBOT HEADING ----------
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // ---------- CALCULATE CAMERA TARGET ----------
        // Field-forward is 0 radians, so we want the camera to always face -botHeading
        double cameraTargetAngle = -botHeading;

        // ---------- READ CURRENT CAMERA ANGLE ----------
        // You need to know how to convert your motor encoder ticks to radians
        // Example: anglePerTick = 2 * Math.PI / ticksPerRevolution;
        double anglePerTick = 2 * Math.PI / COUNTS_PER_MOTOR_REV;
        double currentCameraAngle = cameraMotor.getCurrentPosition() * anglePerTick;

        // ---------- CALCULATE ERROR ----------
        double error = cameraTargetAngle - currentCameraAngle;

        // Normalize error to [-PI, PI] so camera takes shortest path
        while (error > Math.PI) error -= 2 * Math.PI;
        while (error < -Math.PI) error += 2 * Math.PI;

        // ---------- CALCULATE MOTOR POWER ----------
        double power = kP * error;

        // Clip to motor limits
        power = Math.max(MIN_POWER, Math.min(MAX_POWER, power));

        // ---------- SET MOTOR POWER ----------
        cameraMotor.setPower(power);

        // ---------- OPTIONAL LOGGING ----------
        Log.i("Camera Stabilizer", String.format("botHeading=%.2f, cameraAngle=%.2f, error=%.2f, power=%.2f",
                botHeading, currentCameraAngle, error, power));
    }


    public void stopCamera() {
        if (visionPortal != null) {
            visionPortal.stopStreaming();   // stop sending frames to processors
            visionPortal.close();           // releases webcam and processors
        }
    }

    //Configuring the otos sensor- for more detailed comments, refer to the SensorSparkFunOTOS.java file

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
    public void driveByOtosTesting(double distanceToTravelX, double distanceToTravelY, double targetHeadingDegrees, double timeout) {
        runtime.reset();

        // ---------- TUNING / CONFIG ----------
        final double DIST_TOLERANCE = 0.75;         // inches - position tolerance to consider arrived
        final double HEADING_TOLERANCE = Math.toRadians(2.0); // radians - heading tolerance
        final double LPF_ALPHA = 0.12;              // low-pass filter for OTOS (smaller = smoother)
        final double DERIVATIVE_LPF_ALPHA = 0.25;   // smoother derivative
        final double SLOW_DIST = 12.0;              // distance (in) where slowing begins
        final double MAX_LOOP_DT = 0.1;             // max dt cap (s)
        final double MIN_LOOP_DT = 0.005;           // min dt guard (s)

        // PID gains (conservative for X/Y - heading set to effective value so commanded large turns actually occur)
        double kP_X = 0.12, kI_X = 0.0, kD_X = 0.0;
        double kP_Y = 0.12, kI_Y = 0.0, kD_Y = 0.0;

        // IMPORTANT: heading PID uses radians (matching Rotation2d.getRadians()) and sign convention
        double kP_H = 0.25, kI_H = 0.0, kD_H = 0.02;

        final double INTEGRAL_MAX = 10.0; // anti-windup cap

        // ---------- state ----------
        SparkFunOTOS.Pose2D pos = myOtos.getPosition();
        // filtered OTOS values
        double fx = pos.x, fy = pos.y, fh = pos.h;

        // compute field-space target from current reading
        Translation2d currentT = new Translation2d(fx, fy);
        Rotation2d currentR = Rotation2d.fromDegrees(fh);
        Pose2d current = new Pose2d(currentT, currentR);

        Translation2d targetT = currentT.plus(new Translation2d(distanceToTravelX, distanceToTravelY));
        Rotation2d targetR = Rotation2d.fromDegrees(targetHeadingDegrees);
        Pose2d target = new Pose2d(targetT, targetR);

        // PID integrals/previouss
        double integralX = 0.0, prevErrorX = 0.0, prevDerivX = 0.0;
        double integralY = 0.0, prevErrorY = 0.0, prevDerivY = 0.0;
        double integralH = 0.0, prevErrorH = 0.0, prevDerivH = 0.0;

        double lastTime = runtime.seconds();

        while (runtime.seconds() < timeout && myOpMode.opModeIsActive()) {
            // ----- timing -----
            double now = runtime.seconds();
            double dt = now - lastTime;
            lastTime = now;
            if (dt <= 0 || Double.isNaN(dt)) dt = 0.02;
            dt = Math.max(MIN_LOOP_DT, Math.min(MAX_LOOP_DT, dt));

            // ----- read & smooth OTOS -----
            pos = myOtos.getPosition();
            // simple exponential smoothing to reduce noise (helps derivative)
            fx = (1.0 - LPF_ALPHA) * fx + LPF_ALPHA * pos.x;
            fy = (1.0 - LPF_ALPHA) * fy + LPF_ALPHA * pos.y;
            fh = (1.0 - LPF_ALPHA) * fh + LPF_ALPHA * pos.h;

            // current & target translations
            Translation2d curT = new Translation2d(fx, fy);
            Translation2d diffField = target.getTranslation().minus(curT); // field frame error vector
            double toTravelD = curT.getDistance(target.getTranslation());

            // ----- convert field error to robot frame -----
            // robot heading (degrees -> radians for trig)
            double robotTheta = Math.toRadians(fh);
            // field->robot rotation: [ cos(theta)  sin(theta) ; -sin(theta)  cos(theta) ]
            double errorRobotX =  diffField.getX() * Math.cos(robotTheta) + diffField.getY() * Math.sin(robotTheta); // strafe (robot X)
            double errorRobotY = -diffField.getX() * Math.sin(robotTheta) + diffField.getY() * Math.cos(robotTheta); // forward (robot Y)

            // ----- heading error (shortest path) -----
            // ---------------------- CRITICAL FIX ----------------------
            // Use Rotation2d difference (exactly like your original working logic)
            // so we get the correct sign and shortest-path radians
            currentT = new Translation2d(fx, fy);
            currentR = Rotation2d.fromDegrees(fh);
            current = new Pose2d(currentT, currentR);
            Rotation2d toTravelR = target.getRotation().minus(current.getRotation()); // preserves original sign/shortest path
            double errorH = toTravelR.getRadians(); // radians - feed PID in radians (matches original working code)
            // ----------------------------------------------------------

            // ----- integral (anti-windup) -----
            if (Math.abs(errorRobotX) > 0.05) {
                integralX += errorRobotX * dt;
                integralX = Range.clip(integralX, -INTEGRAL_MAX, INTEGRAL_MAX);
            } else {
                integralX = 0;
            }

            if (Math.abs(errorRobotY) > 0.05) {
                integralY += errorRobotY * dt;
                integralY = Range.clip(integralY, -INTEGRAL_MAX, INTEGRAL_MAX);
            } else {
                integralY = 0;
            }

            if (Math.abs(errorH) > Math.toRadians(0.5)) {
                integralH += errorH * dt;
                integralH = Range.clip(integralH, -INTEGRAL_MAX, INTEGRAL_MAX);
            } else {
                integralH = 0;
            }

            // ----- derivative (with small LPF to reduce spikes) -----
            double rawDerivX = (errorRobotX - prevErrorX) / dt;
            double derivX = DERIVATIVE_LPF_ALPHA * rawDerivX + (1 - DERIVATIVE_LPF_ALPHA) * prevDerivX;

            double rawDerivY = (errorRobotY - prevErrorY) / dt;
            double derivY = DERIVATIVE_LPF_ALPHA * rawDerivY + (1 - DERIVATIVE_LPF_ALPHA) * prevDerivY;

            double rawDerivH = (errorH - prevErrorH) / dt;
            double derivH = DERIVATIVE_LPF_ALPHA * rawDerivH + (1 - DERIVATIVE_LPF_ALPHA) * prevDerivH;

            // ----- PID outputs -----
            double outX = kP_X * errorRobotX + kI_X * integralX + kD_X * derivX;
            double outY = kP_Y * errorRobotY + kI_Y * integralY + kD_Y * derivY;
            double outH = kP_H * errorH + kI_H * integralH + kD_H * derivH;

            // save derivative & error for next loop
            prevDerivX = derivX; prevDerivY = derivY; prevDerivH = derivH;
            prevErrorX = errorRobotX; prevErrorY = errorRobotY; prevErrorH = errorH;

            // ----- scale outputs as we approach target (smooth slow down) -----
            // map to [0.15 .. 1.0] as distance ranges [0 .. SLOW_DIST]
            double slowFactor = 1.0;
            if (toTravelD < SLOW_DIST) {
                slowFactor = Range.clip(0.15 + (toTravelD / SLOW_DIST) * (1.0 - 0.15), 0.15, 1.0);
            }
            outX *= slowFactor;
            outY *= slowFactor;
            outH *= slowFactor;


            // ----- clip final drive commands to safe range -----
            outX = Range.clip(outX, -1.0, 1.0);
            outY = Range.clip(outY, -1.0, 1.0);
            outH = Range.clip(outH, -1.0, 1.0);

            // ----- stopping condition -----
            if (toTravelD <= DIST_TOLERANCE && Math.abs(errorH) <= HEADING_TOLERANCE) {
                driveRobotFC(0, 0, 0);   // STOP IMMEDIATELY
                break;
            }

            // ----- send to drivetrain -----
            // KEEPING original sign convention for X/Y. For heading we use the same sign as original working code:
            Log.i("FTC18 driveByOtos[loop]", String.format("Drive commands (robot frame) X=%.3f Y=%.3f H=%.3f  dist=%.2f", outX, outY, outH, toTravelD));
            driveRobotFC(-outY, -outX, -outH); // NOTE: heading sign preserved as in original working version

            // ----- telemetry -----
            myOpMode.telemetry.addData("Pos", "X=%.2f Y=%.2f H=%.2f", fx, fy, fh);
            myOpMode.telemetry.addData("Target", "X=%.2f Y=%.2f H=%.1f", target.getTranslation().getX(), target.getTranslation().getY(), targetHeadingDegrees);
            myOpMode.telemetry.addData("Err (robot)", "X=%.2f Y=%.2f H=%.3f", errorRobotX, errorRobotY, errorH);
            myOpMode.telemetry.addData("Outputs", "X=%.2f Y=%.2f H=%.2f", outX, outY, outH);
            myOpMode.telemetry.addData("Distance", "%.2f", toTravelD);
            myOpMode.telemetry.update();

            // small delay to limit loop rate and let hardware breathe
            myOpMode.sleep(15);
        }
        integralX = 0;
        integralY = 0;
        integralH = 0;
        prevErrorX = prevErrorY = prevErrorH = 0;
        prevDerivX = prevDerivY = prevDerivH = 0;

        // safe stop and short settle
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

    public void driveDiagonalByEncoder(double speed, double distance, double timeout) {
        // Break down the diagonal movement into straight and strafe components
        double straightDistance = distance * Math.cos(Math.PI / 4);  // Diagonal is 45 degrees, so cos(45) = 1/sqrt(2)
        double strafeDistance = distance * Math.sin(Math.PI / 4);    // Similarly, sin(45) = 1/sqrt(2)

        // First, move forward/straight by the appropriate amount
        straightByEncoder(speed, straightDistance, timeout);

        // Then, strafe by the appropriate amount
        strafeByEncoder(speed, strafeDistance, timeout);
    }
    public void driveByEncoderXY(double speed, double xInches, double yInches, double timeout) {
        if (myOpMode.opModeIsActive()) {
            DcMotor.RunMode oldMotorMode = leftFrontDrive.getMode();

            // Reset encoders
            setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Compute movement in encoder counts
            int xCounts = (int)(xInches * COUNTS_PER_INCH);
            int yCounts = (int)(yInches * COUNTS_PER_INCH);

            // Calculate each motor’s movement
            int leftFrontTarget  = leftFrontDrive.getCurrentPosition()  + (yCounts + xCounts);
            int rightFrontTarget = rightFrontDrive.getCurrentPosition() + (yCounts - xCounts);
            int leftBackTarget   = leftBackDrive.getCurrentPosition()   + (yCounts - xCounts);
            int rightBackTarget  = rightBackDrive.getCurrentPosition()  + (yCounts + xCounts);

            // Normalize the values so that no wheel goes beyond the longest distance
            int max = Math.max(
                    Math.max(Math.abs(leftFrontTarget), Math.abs(rightFrontTarget)),
                    Math.max(Math.abs(leftBackTarget), Math.abs(rightBackTarget))
            );
            if (max > Math.abs(yCounts) + Math.abs(xCounts)) {
                double scale = ((double)(Math.abs(yCounts) + Math.abs(xCounts))) / max;
                leftFrontTarget  *= scale;
                rightFrontTarget *= scale;
                leftBackTarget   *= scale;
                rightBackTarget  *= scale;
            }

            // Set target positions
            leftFrontDrive.setTargetPosition(leftFrontTarget);
            rightFrontDrive.setTargetPosition(rightFrontTarget);
            leftBackDrive.setTargetPosition(leftBackTarget);
            rightBackDrive.setTargetPosition(rightBackTarget);

            // Run to position
            setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Start motion
            runtime.reset();
            leftFrontDrive.setPower(Math.abs(speed));
            leftBackDrive.setPower(Math.abs(speed));
            rightFrontDrive.setPower(Math.abs(speed));
            rightBackDrive.setPower(Math.abs(speed));

            // Wait until done or timeout
            while (myOpMode.opModeIsActive() &&
                    (runtime.seconds() < timeout) &&
                    (leftFrontDrive.isBusy() || rightFrontDrive.isBusy() ||
                            leftBackDrive.isBusy() || rightBackDrive.isBusy())) {

                myOpMode.telemetry.addData("Target", "lf:%d lb:%d rf:%d rb:%d",
                        leftFrontTarget, leftBackTarget, rightFrontTarget, rightBackTarget);
                myOpMode.telemetry.addData("Current", "lf:%d lb:%d rf:%d rb:%d",
                        leftFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition(),
                        rightFrontDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
                // myOpMode.telemetry.update();
            }

            // Stop motors
            stop();
            setMotorMode(oldMotorMode);
            myOpMode.sleep(250);
        }
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
    //added -180 to headingerror thing (Math.abs(headingError) > HEADING_THRESHOLD)
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

    // Decode 2026
    public void spinTake(double intake_power) {
        spinTake.setPower(intake_power);
    }

    public void turntable(double intake_power) {
        cameraMotor.setPower(intake_power);
    }

    public int autoDecision() {
        int pattern = 0;
        List<AprilTagDetection> detections = aprilTag.getDetections();

        for (AprilTagDetection detection : detections) {

            // Look for a specific tag ID
            if (detection.id == 21) {
                pattern = 1; //GPP
                return pattern;
            } else if (detection.id == 22) {
                pattern = 2; //PGP
                return pattern;
            } else if (detection.id == 23) {
                pattern = 3; //PPG
                return pattern;
            } else {
                return pattern;
            }
        }   // end method telemetryAprilTag()
        return pattern;
    }

    //autonomous stuff!
    public void autoGPP() {
        driveByOtosTesting(5,5,0, 30);
    }

    public void autoGPG() {
        driveByOtos(2,2,0, 30);
    }

    public void autoPPG() {
        driveByOtos(2,2,0, 30);
    }

    /*
    -------HARDWARE---------
    private DcMotor intakeMotor, turntableMotor;
    private Servo doorToTurntable, launcherDoor;
    private ColorSensor colorSensor;

    private String[] slots = {null, null, null};
    private int currentSlot = 0;
    private static final double DOOR_TURN_OPEN = 0.8;
    private static final double DOOR_TURN_CLOSED = 0.2;
    private static final double LAUNCHER_OPEN = 0.9;
    private static final double LAUNCHER_CLOSED = 0.1;
    private static final int COUNTS_PER_SLOT = COUNTS_PER_MOTOR_REV / 3;

    public void init(HardwareMap hwMap) {
        //just add motors to init class
        intakeMotor = hwMap.dcMotor.get("intake");
        turntableMotor = hwMap.dcMotor.get("turntable");
        doorToTurntable = hwMap.servo.get("doorTurn");
        launcherDoor = hwMap.servo.get("doorLaunch");
        colorSensor = hwMap.colorSensor.get("color");

        turntableMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turntableMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    //idk what interrupted exception is, make sure that works
    private void intakeCycle() throws InterruptedException {
        // Spin intake and detect color intakeMotor.setPower(0.8);
        String detectedColor = detectColor();
        if (detectedColor != null) {
            intakeMotor.setPower(0);
            openDoorToTurntable();
            sleep(300);
            insertIntoTurntable(detectedColor);
            closeDoorToTurntable();
            }
        }
    private String detectColor() {
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();
        if (green > red && green > blue) {
            return "green"; }
        else if (red > green && blue > green) {
            return "purple";
            // adjust thresholding for your sensor
        } else {
            return null; }
        }
    private void insertIntoTurntable(String color) throws InterruptedException {
        slots[currentSlot] = color; telemetry.addData("Inserted", color + " in slot " + currentSlot);
        telemetry.update();
        // Rotate to next intake position (next slot)
        currentSlot = (currentSlot + 1) % 3;
        rotateTurntableToSlot(currentSlot);
    }
    private void rotateTurntableToSlot(int slotIndex) {
        int targetTicks = slotIndex * TICKS_PER_SLOT;
        turntableMotor.setTargetPosition(targetTicks);
        turntableMotor.setPower(0.4);
        while (opModeIsActive() && turntableMotor.isBusy()) {
            telemetry.addData("Turning to slot", slotIndex);
            telemetry.update(); }
        turntableMotor.setPower(0); }
    private void shootColor(String color) throws InterruptedException {
        for (int i = 0; i < 3; i++) {
            if (color.equals(slots[i])) {
                rotateTurntableToSlot(i);
                openLauncherDoor();
                sleep(500);
                launch();
                closeLauncherDoor();
                slots[i] = null;
                break; }
                }
                }
    private void launch() {
        // This could trigger a flywheel, pneumatic, or similar
        telemetry.addLine("Launched!");
        telemetry.update(); }

    // Servo control helpers
    public void openDoorToTurntable() { doorToTurntable.setPosition(DOOR_TURN_OPEN); }
    public void closeDoorToTurntable() { doorToTurntable.setPosition(DOOR_TURN_CLOSED); }
    public void openLauncherDoor() { launcherDoor.setPosition(LAUNCHER_OPEN); }
    public void closeLauncherDoor() { launcherDoor.setPosition(LAUNCHER_CLOSED); }

    ------TELEOP------
    @TeleOp(name="Color Sorter Launcher", group="Linear Opmode")
    public class ColorSorterLauncherTeleOp extends LinearOpMode {

        private HardwareColorSorter robot = new HardwareColorSorter();

        public void runOpMode() throws InterruptedException {
            robot.init(hardwareMap);

            telemetry.addLine("Ready");
            telemetry.update();

            waitForStart();

            while (opModeIsActive()) {
                // Press A to run intake cycle
                if (gamepad1.a) {
                    robot.intakeCycle();
                }

                // Press B to shoot green
                if (gamepad1.b) {
                    robot.shootColor("green");
                }

                // Press X to shoot purple
                if (gamepad1.x) {
                    robot.shootColor("purple");
                }

                telemetry.addData("Slots", Arrays.toString(robot.slots));
                telemetry.update();
            }
        }
    }
     */

    /*
        intake spins (use encoder and ticks --> how many ticks in one revolution?)
        artifact goes on ramp
        door to turntable opens (servo-->open while intake spins?)
        detect color
        go into turntable
            spots 1,2,3
            spot 1 = color detected (green)
            spot 2 = color detected (purple)
            spot 3 = color detected (green)
            spot 1 intake, spot 3 shoot
            spot 2 intake, spot 1 shoot
            spot 3 intake, spot 2 shoot
        turntable turns 120 degrees + shoots
 */



}

