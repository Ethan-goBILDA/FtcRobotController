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

package org.firstinspires.ftc.teamcode.Wayfinder;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


@TeleOp(name="Wayfinder Tester", group="Pinpoint")
//@Disabled
public class WayfinderTester extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    private GoBildaPinpointDriver pinpoint = null; // Declare OpMode member for the Odometry Computer

    private double frontLeftMotorOutput = 0;
    private double frontRightMotorOutput = 0;
    private double backLeftMotorOutput = 0;
    private double backRightMotorOutput = 0;

    @Override
    public void runOpMode() {

        initializeMotors();

        initializePinpoint();

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            calculateMecanumOutput(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);

            // Send calculated power to wheels
            frontLeftDrive.setPower(frontLeftMotorOutput);
            frontRightDrive.setPower(frontRightMotorOutput);
            backLeftDrive.setPower(backLeftMotorOutput);
            backRightDrive.setPower(backRightMotorOutput);

            if(gamepad1.aWasPressed()){
                pinpoint.setPosition(new Pose2D(DistanceUnit.MM, 0,0, AngleUnit.DEGREES, 0));
            }

            pinpoint.update();

            telemetry.addData("Y Stick", gamepad1.left_stick_y);
            telemetry.addData("left X Stick", gamepad1.left_stick_x);
            telemetry.addData("right X Stick", gamepad1.right_stick_x);

            telemetry.addLine("Use the gamepad to drive your robot, pushing the left stick " +
                    "forward should move the robot forward. The left stick left should move the robot " +
                    "left. Moving the right stick left should spin the robot counterclockwise.");
            telemetry.addLine("");
            telemetry.addLine("Once you have verified that the mecanum drive is correct, " +
                    "click A to reset the Pinpoint's estimated position and move the robot forward. " +
                    "The estimated position shown below should see X increase. " +
                    "If X does not increase, reverse the direction of the X pod. " +
                    "When you move the robot to the left, the Y position should increase, if it " +
                    "does not increase, reverse the direction of the Y pod.");
            telemetry.addLine("");
            telemetry.addData("X in MM", pinpoint.getPosX(DistanceUnit.MM));
            telemetry.addData("Y in MM", pinpoint.getPosY(DistanceUnit.MM));
            telemetry.addData("Heading in Degrees", pinpoint.getHeading(AngleUnit.DEGREES));
            telemetry.update();
        }
    }

    /**
     * This is a standard mecanum mix function.
     * @param forward -1 to 1, requested forward drive power.
     * @param strafe -1 to 1, requested strafe drive power.
     * @param yaw -1 to 1, requested yaw (heading) drive power.
     */
    private void calculateMecanumOutput(double forward, double strafe, double yaw) {
        double leftFront = forward - strafe - yaw;
        double rightFront = forward + strafe + yaw;
        double leftBack = forward + strafe - yaw;
        double rightBack = forward - strafe + yaw;

        double max = Math.max(Math.abs(leftFront), Math.abs(rightFront));
        max = Math.max(max, Math.abs(leftBack));
        max = Math.max(max, Math.abs(rightBack));

        if (max > 1.0) {
            leftFront /= max;
            rightFront /= max;
            leftBack /= max;
            rightBack /= max;
        }

        frontLeftMotorOutput  = leftFront;
        frontRightMotorOutput = rightFront;
        backLeftMotorOutput   = leftBack;
        backRightMotorOutput  = rightBack;
    }

    public void initializeMotors(){
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void initializePinpoint(){
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        pinpoint.setOffsets(-142.0, 120.0, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        pinpoint.resetPosAndIMU();
    }
}
