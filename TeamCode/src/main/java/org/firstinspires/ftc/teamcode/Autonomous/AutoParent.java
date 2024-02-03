/* Copyright (c) 2019 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.DriverControl.TechiesOpMode.COUNTS_PER_INCH;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.TechiesHardwareWithoutDriveTrain;
import org.firstinspires.ftc.teamcode.TechiesRobotHardware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

//@Disabled
@Autonomous(name = "AutoParent", group = "ConceptBlue")
public abstract class AutoParent extends LinearOpMode  {

    static final int TARGET_LEVEL_DEFAULT = 1;
    public static final int LEFT_POSITION = 1;
    public static final int MIDDLE_POSITION = 2;
    public static final int RIGHT_POSITION = 3;

    private ElapsedTime runtime = new ElapsedTime();

    TechiesHardwareWithoutDriveTrain robotCore ;
    SampleMecanumDrive odoDriveTrain;

    static final double FEET_PER_METER = 3.28084;

    //VoltageSensor voltageSensor = hardwareMap.get();
    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!


    double batteryVoltageMultiplier = 1;
    public abstract double adjustTurn(double angle);
    public abstract int adjustZone(int zone);
    public abstract int adjustTrajectorydistance(int distance);


    protected int position = TARGET_LEVEL_DEFAULT;

    @Override
    public void runOpMode() {
        robotCore = new TechiesHardwareWithoutDriveTrain(hardwareMap);
        odoDriveTrain = new SampleMecanumDrive(hardwareMap);

        waitForStart();
        doMissions();
    }

    protected int determineTargetZone(Telemetry telemetry) {

        if (robotCore.rightsensorRange.getDistance(DistanceUnit.INCH) > 1 && robotCore.rightsensorRange.getDistance(DistanceUnit.INCH) < 15) {

            position = MIDDLE_POSITION;
        }else if (robotCore.leftsensorRange.getDistance(DistanceUnit.INCH) > 1 && robotCore.leftsensorRange.getDistance(DistanceUnit.INCH) < 5){
            position = LEFT_POSITION;
        } else {
            position = RIGHT_POSITION;

        }
        telemetry.addData("position", position);
        telemetry.addData("deviceName", robotCore.leftsensorRange.getDeviceName() );
        telemetry.addData("range", String.format("%.01f in", robotCore.leftsensorRange.getDistance(DistanceUnit.INCH)));
        telemetry.addData("deviceName", robotCore.rightsensorRange.getDeviceName() );
        telemetry.addData("range", String.format("%.01f in", robotCore.rightsensorRange.getDistance(DistanceUnit.INCH)));
        telemetry.update();
        return position;
    }

    protected void doMissions() {
        forward(25);
        position = determineTargetZone(telemetry);
        sleep(2000);
       goToTapeFromStart(adjustZone(position));
        dropPixel();
        goToBackdrop(adjustZone(position));
        dropBackdrop();
        park();

    }
    abstract protected void goToTapeFromStart(int targetZone);



    protected void dropPixel(){

        robotCore.wrist.setPosition(.57);
        sleep(1500);
        //robotCore.claw.setPosition(1);
        //sleep(500);
        robotCore.wrist.setPosition(.1);
        sleep(1500);
        // robotCore.claw.setPosition(0);
        //sleep(100);

        // come back to wrist and claw
    }
    abstract protected void goToBackdrop(int targetZone);
    protected void dropBackdrop(){
        encoderArm(0.5, -5.5);
        robotCore.wrist.setPosition(0.19);
        sleep(2000);
        robotCore.claw.setPosition(.82);
        sleep(1500);
        encoderArm(0.3, 5.5);//18
        sleep(1500);
    }
    abstract protected void park();




    protected void back(int inches){
        Pose2d startPose = new Pose2d(0,0, Math.toRadians(0));
        odoDriveTrain.setPoseEstimate(startPose);
        Trajectory back = odoDriveTrain.trajectoryBuilder(new Pose2d(0,0,0))
                .back(inches)
                .build();
        odoDriveTrain.followTrajectory(back);
        Pose2d startPose2 = new Pose2d(0,0, Math.toRadians(0));
        odoDriveTrain.setPoseEstimate(startPose2);

    }
    protected void forward (double inches){
        Pose2d startPose = new Pose2d(0,0, Math.toRadians(0));
        odoDriveTrain.setPoseEstimate(startPose);
        Trajectory forward = odoDriveTrain.trajectoryBuilder(new Pose2d(0,0,0))
                .forward(inches)
                .build();
        odoDriveTrain.followTrajectory(forward);
        // Pose2d startPose2 = new Pose2d(0,0, Math.toRadians(0));
        Pose2d startPose2 = forward.end();
        odoDriveTrain.setPoseEstimate(startPose2);
    }
    protected void lineToSpline(int x, int y, int degrees){
        Pose2d startPose = new Pose2d(0,0, Math.toRadians(0));
        odoDriveTrain.setPoseEstimate(startPose);
        Trajectory linetospline = odoDriveTrain.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(x, y, Math.toRadians(degrees)))
                .build();
        odoDriveTrain.followTrajectory(linetospline);
    }
    protected void straferight (double inches) {
        // odoDriveTrain.rightDrive.setDirection(DcMotor.Direction.FORWARD); // FORWARD // Set to FORWARD if using AndyMark motors*/
        //   odoDriveTrain.rightBack.setDirection(DcMotor.Direction.FORWARD); // FORWARD // Set to FORWARD if using AndyMark motors*/

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        odoDriveTrain.setPoseEstimate(startPose);
        Trajectory straferight = odoDriveTrain.trajectoryBuilder(new Pose2d(0, 0, 0))
                .strafeRight(inches)
                .build();
        odoDriveTrain.followTrajectory(straferight);
        Pose2d startPose2 = straferight.end();
        odoDriveTrain.setPoseEstimate(startPose2);
        // odoDriveTrain.rightBack.setDirection(DcMotor.Direction.REVERSE); // FORWARD // Set to FORWARD if using AndyMark motors*/
        //    odoDriveTrain.rightDrive.setDirection(DcMotor.Direction.REVERSE); // FORWARD // Set to FORWARD if using AndyMark motors*/

    }
    protected void strafeleft (double inches){
        // odoDriveTrain.rightBack.setDirection(DcMotor.Direction.FORWARD); // FORWARD // Set to FORWARD if using AndyMark motors*/
        //  odoDriveTrain.rightDrive.setDirection(DcMotor.Direction.FORWARD); // FORWARD // Set to FORWARD if using AndyMark motors*/
        // sleep(1000);
        Pose2d startPose = new Pose2d(0,0, Math.toRadians(0));
        odoDriveTrain.setPoseEstimate(startPose);
        Trajectory strafeleft = odoDriveTrain.trajectoryBuilder(new Pose2d(0,0,0))
                .strafeLeft(inches)
                .build();
        odoDriveTrain.followTrajectory(strafeleft);
        Pose2d startPose2 = strafeleft.end();
        odoDriveTrain.setPoseEstimate(startPose2);
        //sleep(1000);
        //odoDriveTrain.rightBack.setDirection(DcMotor.Direction.REVERSE); // FORWARD // Set to FORWARD if using AndyMark motors*/
        //  odoDriveTrain.rightDrive.setDirection(DcMotor.Direction.REVERSE); // FORWARD // Set to FORWARD if using AndyMark motors*/

    }

    public void encoderArm(double speed,
                           double armInches) {
        int newArmTarget;


        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newArmTarget = robotCore.arm.getCurrentPosition() + (int)(armInches * COUNTS_PER_INCH);

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

