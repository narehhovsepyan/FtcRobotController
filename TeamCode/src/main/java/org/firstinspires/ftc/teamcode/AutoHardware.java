//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//public class AutoHardware extends Hardware2025 {
//    private final LinearOpMode myOpMode;
//    public AutoHardware(LinearOpMode opmode) {
//        super(opmode.hardwareMap, true);
//        this.myOpMode = opmode;
//
//        // Initialize hardware from Hardware2025 using the opmode's hardwareMap
//        init(opmode.hardwareMap);
//    }
//
//    // Example method to test functionality
//    public void driveForward(double speed, int distance) {
//        straightByEncoder(speed, distance, 10); // Example: Move forward using encoders
//        myOpMode.telemetry.addData("Action", "Driving Forward");
//        myOpMode.telemetry.update();
//    }
//
//    public void scoreSpecimen() {
//        //moving the slide
//        startSlideByEncoder(.5, HIGH_POSITION, 15);
//        while (!isSlideDone()){
//        }
//        strafeByEncoder(.5, 3.1, 15);
//        relativeSlideByEncoder(.5, -4, 5);
//        while(!isSlideDone()) {
//        }
//        clawServo.setPosition(0.7);
//        startSlideByEncoder(.5, WALL_POSITION, 15);
//        while (!isSlideDone()){
//        }
//    }
//
//
//
//    public void driveWhileTurn(double speed,double angle, double timeout) {
//
//    }
//
//
//}
