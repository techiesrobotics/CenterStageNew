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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.TechiesHardwareWithoutDriveTrain;
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
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    static final double OPEN_CLAW = 0;
    static final double CLOSED_CLAW = 1;
    // UNITS ARE METERS
    double tagsize = 0.166;

    //Tag Id for sleeve
    static final int LEFT = 5; // Tag ID 18 from the 36h11 family
    static final int MIDDLE = 6;
    static final int RIGHT = 9;

    public static final int LEFT_POSITION = 1;

    public static final int MIDDLE_POSITION = 2;

    public static final int RIGHT_POSITION = 3;

    double batteryVoltageMultiplier = 1;
    public abstract double adjustTurn(double angle);
    public abstract double adjustTrajectorydistance(double distance);

    protected abstract void park();

    protected int position = 2;

    @Override
    public void runOpMode() {
        robotCore = new TechiesHardwareWithoutDriveTrain(hardwareMap);
        while (!isStarted() && !isStopRequested()) {
            position = determineTargetZone(telemetry);
            // telemetry.addData(">", "Press Play to start op mode");
            //  telemetry.addData("Voltage Multiplier", batteryVoltageMultiplier);
            // telemetry.update();

        }

        waitForStart();
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
        return position;
    }



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
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        odoDriveTrain.setPoseEstimate(startPose);
        Trajectory straferight = odoDriveTrain.trajectoryBuilder(new Pose2d(0, 0, 0))
                .strafeRight(inches)
                .build();
        odoDriveTrain.followTrajectory(straferight);
        Pose2d startPose2 = straferight.end();
        odoDriveTrain.setPoseEstimate(startPose2);
    }
    protected void strafeleft (double inches){
        Pose2d startPose = new Pose2d(0,0, Math.toRadians(0));
        odoDriveTrain.setPoseEstimate(startPose);
        Trajectory strafeleft = odoDriveTrain.trajectoryBuilder(new Pose2d(0,0,0))
                .strafeLeft(inches)
                .build();
        odoDriveTrain.followTrajectory(strafeleft);
        Pose2d startPose2 = strafeleft.end();
        odoDriveTrain.setPoseEstimate(startPose2);
    }
}
