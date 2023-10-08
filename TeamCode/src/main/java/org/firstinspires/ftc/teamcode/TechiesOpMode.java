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

package org.firstinspires.ftc.teamcode.DriverControlOpMode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.TechiesHardwareWithoutDriveTrain;
import org.firstinspires.ftc.teamcode.TechiesRobotHardware;



@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")
//@Disabled
public abstract class TechiesOpMode extends LinearOpMode {

    // Declare OpMode members.
    TechiesRobotHardware robot   = new TechiesRobotHardware();
    TechiesHardwareWithoutDriveTrain robotCore;
    private ElapsedTime runtime = new ElapsedTime();
    public static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    public static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    public static final double     WHEEL_DIAMETER_INCHES   = 2.0 ;     // For figuring circumference
    public static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    abstract public double getTurn() ;
    abstract public double getDrivefb();
    abstract public double getDrivelr();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot.init(hardwareMap);


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
            // - This uses basic math to combine motions and is easier to drive straight.

            double turn = getTurn();
            double drivefb  = getDrivefb();  //-gamepad1.left_stick_y;
            double drivelr = getDrivelr(); //gamepad1.left_stick_x;

            leftPower    = Range.clip(drivefb + turn + drivelr, -.55, .55) ;
            rightPower   = Range.clip(drivefb - turn - drivelr, -.55, .55) ;
            backleftPower   = Range.clip(drivefb + turn - drivelr, -.55, .55) ;
            backrightPower   = Range.clip(drivefb - turn + drivelr, -.55, .55) ;


            // Send calculated power to wheels
            //  Multiplier = driveSpeed();
            robot.leftDrive.setPower(leftPower*Multiplier);
            robot.rightDrive.setPower(rightPower*Multiplier);
            robot.leftBack.setPower(backleftPower*Multiplier);
            robot.rightBack.setPower(backrightPower*Multiplier);
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
            if (gamepad1.y) {
                robotCore.leftShooter.setPower(0);
                robotCore.rightShooter.setPosition(1);

            }

        }
    }


}

