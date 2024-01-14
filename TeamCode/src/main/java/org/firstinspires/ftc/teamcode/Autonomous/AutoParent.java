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

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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
    static final int TARGET_LEVEL_LEFT = 1;
    static final int TARGET_LEVEL_MIDDLE = 2;
    static final int TARGET_LEVEL_RIGHT = 3;

    TechiesHardwareWithoutDriveTrain robotCore ;
    SampleMecanumDrive odoDriveTrain;

    static final double FEET_PER_METER = 3.28084;

    //VoltageSensor voltageSensor = hardwareMap.get();
    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!



    public static final int LEFT_POSITION = 1;

    public static final int MIDDLE_POSITION = 2;

    public static final int RIGHT_POSITION = 3;

    double batteryVoltageMultiplier = 1;
    public abstract double adjustTurn(double angle);
    public abstract int adjustZone(int zone);
    public abstract double adjustTrajectorydistance(double distance);


    protected int position = TARGET_LEVEL_DEFAULT;

    @Override
    public void runOpMode() {
        robotCore = new TechiesHardwareWithoutDriveTrain(hardwareMap);
        odoDriveTrain = new SampleMecanumDrive(hardwareMap);
        while (!isStarted() && !isStopRequested()) {
            position = determineTargetZone(telemetry);


        }

        waitForStart();
        doMissions();
    }
    protected int determineTargetZone(Telemetry telemetry){
        if (robotCore.leftsensorRange.getDistance(DistanceUnit.INCH) > 12 && robotCore.leftsensorRange.getDistance(DistanceUnit.INCH) < 18){
            position = LEFT_POSITION;
        } else if (robotCore.rightsensorRange.getDistance(DistanceUnit.INCH) > 10 && robotCore.rightsensorRange.getDistance(DistanceUnit.INCH) < 14){
            position = RIGHT_POSITION;
        } else {
            position = MIDDLE_POSITION;
        }
        telemetry.addData("position", position);
        telemetry.addData("deviceName", robotCore.leftsensorRange.getDeviceName() );
        telemetry.addData("range", String.format("%.01f in", robotCore.leftsensorRange.getDistance(DistanceUnit.INCH)));
        telemetry.addData("deviceName", robotCore.rightsensorRange.getDeviceName() );
        telemetry.addData("range", String.format("%.01f in", robotCore.rightsensorRange.getDistance(DistanceUnit.INCH)));
        telemetry.update();
        return 1;
    }

    protected void doMissions() {
        goToTapeFromStart(adjustZone(position));
       dropPixel();
       goToBackdrop(position);
        dropBackdrop(position);
        park();

    }
    protected void goToTapeFromStart(int targetZone) {


        forward(26);
        if (targetZone == LEFT_POSITION) {
            odoDriveTrain.turn(Math.toRadians(-98));
            sleep(800);
            back(22);
            /*Pose2d startPose = new Pose2d(0,0, Math.toRadians(0));
            odoDriveTrain.setPoseEstimate(startPose);
            Trajectory leftTape = odoDriveTrain.trajectoryBuilder(new Pose2d(0,0,0))
                    .lineToLinearHeading(new Pose2d(25,0,Math.toRadians(45)))
                    .build();
            odoDriveTrain.followTrajectory(leftTape);
            Pose2d startPose2 = leftTape.end();
            odoDriveTrain.setPoseEstimate(startPose2);*/
        } else if (targetZone == RIGHT_POSITION) {
            odoDriveTrain.turn((Math.toRadians(-98)));
        } else if (targetZone == MIDDLE_POSITION) {

        }
    }



    protected void dropPixel(){

        robotCore.wrist.setPosition(1);
        sleep(500);
        robotCore.claw.setPosition(1);
        sleep(500);
        robotCore.wrist.setPosition(0);
        sleep(1500);
        robotCore.claw.setPosition(0);
        sleep(100);

         // come back to wrist and claw
    }
    abstract protected void goToBackdrop(int targetZone);
    protected void dropBackdrop(int targetZone){
        if (targetZone == LEFT_POSITION) {

        } else if (targetZone == RIGHT_POSITION) {

        } else if (targetZone == MIDDLE_POSITION) {
        }
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

}
