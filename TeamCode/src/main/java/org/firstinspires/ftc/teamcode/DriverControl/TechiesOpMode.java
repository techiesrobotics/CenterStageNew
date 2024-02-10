/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.DriverControl;


import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.TechiesHardwareWithoutDriveTrain;
import org.firstinspires.ftc.teamcode.TechiesRobotHardware;
import org.firstinspires.ftc.teamcode.util.Gyro;

import java.sql.Date;
import java.sql.Time;
import java.time.Instant;

@TeleOp(name = "Basic: Linear OpMode", group = "Linear Opmode")
//@Disabled
public abstract class TechiesOpMode extends LinearOpMode {

    private static final PIDFController pidController = new PIDFController(new PIDCoefficients(0, 0, 0));
    private double refAngle = 0;
    // Declare OpMode members.
    TechiesRobotHardware robot = new TechiesRobotHardware();
    TechiesHardwareWithoutDriveTrain robotCore;


    private Gyro gyro;
    private ElapsedTime runtime = new ElapsedTime();
    public static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    public static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    public static final double WHEEL_DIAMETER_INCHES = 2.0;     // For figuring circumference
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    abstract public double getTurn();

    abstract public double getDrivefb();

    abstract public double getDrivelr();
    public final double ClosedClaw=.1;
    public final double OpenClaw=.76;


    boolean isUp;
    int startingPosition;

    @Override
    public void runOpMode() {
        isUp = false;
        robotCore = new TechiesHardwareWithoutDriveTrain(hardwareMap);
        telemetry.addData("Status", "Initialized");

        telemetry.update();
        robot.init(hardwareMap);
        gyro = new Gyro(hardwareMap);
        gyro.init();
        startingPosition = robotCore.arm.getCurrentPosition();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;
            double backleftPower;
            double backrightPower;
            double Multiplier = 1;
            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.\

            //moved on top
            //This code below has been disabled for the autonomous since the claw is bugging out
            //arm and wrist needs tp be ,pved
            if (gamepad1.a) {
                if (robotCore.claw.getPosition() > 0.5) {
                    telemetry.addData("claw position", "1");
                    robotCore.claw.setPosition(ClosedClaw);
                    telemetry.addData("claw position changed to 0", "0");

                    telemetry.update();
                    sleep(200);
                } else if (robotCore.claw.getPosition() <= 0.5) {
                    telemetry.addData("claw position", "0");
                    robotCore.claw.setPosition(OpenClaw);
                    telemetry.addData("claw position changed to 1", "1");
                    telemetry.update();
                    sleep(200);
                }
            }
            if (gamepad1.b) {
                robotCore.claw.setPosition(0.3);
            }
         /*   if (gamepad1.right_bumper){
                robotCore.arm.setPower(0.2);
            } else if (gamepad1.left_bumper){
                robotCore.arm.setPower(-0.2);
            } else {
                robotCore.arm.setPower(0);
            }*/
            if (gamepad1.right_bumper) {
                if (robotCore.arm.getTargetPosition() / COUNTS_PER_INCH > -3.5) {
                    encoderArm(0.4, -7.3, false);//4.65
                    robotCore.wrist.setPosition(.1);
                    sleep(200);

                }else {//(robotCore.arm.getCurrentPosition() >= -5) {
                    encoderArm(0.45, -0.35, false);//-4.65
                    robotCore.wrist.setPosition(0.1);
                    sleep(200);
                }

            }

            if (gamepad1.dpad_right) {
                encoderArm(1, 500, true);//-4.65
                sleep(40000);
            }

            if (gamepad1.y) {
                if (robotCore.wrist.getPosition() > 0.3
                ) {
                    // encoderDrive(0.35, .4);
                    robotCore.wrist.setPosition(0.1);
                    sleep(200);
                } else if (robotCore.wrist.getPosition() <= 0.3) {
                    //encoderDrive(0.35, -.35);
                    robotCore.wrist.setPosition(.57);
                    sleep(200);
                }
            }

            if (gamepad1.x) {
                robotCore.wrist.setPosition(0.1);


                encoderArm(1, -0.3, false);//4.65
                sleep(5000);

            }
            if (gamepad1.right_trigger > 0 && gamepad1.left_trigger > 0) {
                robotCore.droneLauncher.setPosition(.05);

            }
            if (gamepad1.dpad_up) {
                encoderArm(0.8, -.5, true);
            }
            if (gamepad1.dpad_down) {
                encoderArm(0.8, .5, true);
            }


            double turn = getTurn();
            double drivefb = getDrivefb();  //-gamepad1.left_stick_y;
            double drivelr = getDrivelr(); //gamepad1.left_stick_x;
            final double pidValue = 0;


            if (turn != 0) {
                refAngle = gyro.getAngle();
            } else {
                //  pidValue = pidController.update(gyro.getAngle() - refAngle);
            }

            telemetry.addData("PIDVALUE", pidValue);

            leftPower = Range.clip(-drivefb + turn + drivelr + pidValue, -0.9, 0.9);
            rightPower = Range.clip(-drivefb - turn - drivelr - pidValue, -0.9, 0.9);
            backleftPower = Range.clip(-drivefb + turn - drivelr + pidValue, -0.9, 0.9);
            backrightPower = Range.clip(-drivefb - turn + drivelr - pidValue, -0.9, 0.9);


            // Send calculated power to wheels
            //  Multiplier = driveSpeed();
            robot.leftDrive.setPower(leftPower);
            robot.rightDrive.setPower(rightPower );
            robot.leftBack.setPower(backleftPower);
            robot.rightBack.setPower(backrightPower);
            // Show the elapsed game time and wheel power.
            telemetry.addData("Arm position",  robotCore.arm.getCurrentPosition() / COUNTS_PER_INCH);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("stickX", drivefb + turn + drivelr);
            telemetry.addData("Motor pos: ", robotCore.claw.getPosition());
            telemetry.addData("Position of wrist servo: ", robotCore.wrist.getPosition());
            telemetry.addData("Position of claw servo: ", robotCore.claw.getPosition());
            telemetry.addData("Date", Instant.now().toString());
            telemetry.update();
            // if (gamepad1.y) {
            // robotCore.droneLauncher.setPosition(0);

            // }

        }
    }


    public void encoderArm(double speed,
                           double armInches, boolean increment) {
        int newArmTarget;


        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newArmTarget = robotCore.arm.getCurrentPosition() + (int) (armInches * COUNTS_PER_INCH);
            if (increment==false){
                newArmTarget = (int) (armInches * COUNTS_PER_INCH);
            }
            robotCore.arm.setTargetPosition(newArmTarget);


            // Turn On RUN_TO_POSITION
            robotCore.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robotCore.arm.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.


            sleep(250);   // optional pause after each move.
        }
    }
}

