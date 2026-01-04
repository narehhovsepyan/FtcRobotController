/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains Teleop
 */

@TeleOp(name = "Teleop2026", group = "Linear Opmode")

public class Teleop2026 extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    Hardware2026 robot = new Hardware2026(this);
    private final ElapsedTime runtime = new ElapsedTime();

    // Robot class
    public void runOpMode() {
        robot.init();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            telemetry.update();
            telemetry.addData("Turret Encoder", robot.turntableMotor.getCurrentPosition());
            double ticksFromZero = robot.turntableMotor.getCurrentPosition() / .43;
            telemetry.addData("Angle", ticksFromZero);

            if (gamepad1.options) {
                robot.resetYaw();
            }

            double gp1LY = gamepad1.left_stick_y;
            double gp1LX = gamepad1.left_stick_x;
            double gp1RX = gamepad1.right_stick_x;

            if (gamepad1.right_bumper) {
                double slowScale = .25;
                gp1LY *= slowScale;
                gp1LX *= slowScale;
                gp1RX *= slowScale;
            }

            if (gamepad1.left_bumper) {
                double slowScale = .33;
                gp1LY *= slowScale;
                gp1LX *= slowScale;
                gp1RX *= slowScale;
            }

            robot.driveRobotFC(-gp1LY, gp1LX, gp1RX);

            telemetry.update();

//            if (gamepad2.y){
//                robot.startDaisySpin(1, 1);
//            }
//            robot.updateDaisySpin();

//            double shooterSpeed = gamepad2.right_trigger;
//            robot.shootArtifact(shooterSpeed);

//            if (robot.checkIfDetected(24)){
//                boolean centered = robot.updateTurretToAprilTag(24);
//
//                telemetry.addData("Turret AutoAim", "ACTIVE");
//                telemetry.addData("Turret Centered", centered);
//            } else {
//                robot.updateTurntableToFaceTarget();
//            }

            if (gamepad1.b){
                robot.setTurntableAngle(robot.updateTurntableToFaceTarget(),.8);
            } else {
                robot.turntableMotor.setPower(0);
            }

            if (gamepad2.left_bumper){
                robot.spinTake(1);
            } else {
                robot.stopSpinTake();
            }
//            robot.driveWithOtos();

        }
//        robot.stopCamera();
    }
}