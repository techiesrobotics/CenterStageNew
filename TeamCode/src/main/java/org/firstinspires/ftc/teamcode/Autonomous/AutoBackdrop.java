package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

abstract public class AutoBackdrop extends AutoParent{

    protected void goToBackdrop(int targetZone){
        if (targetZone == LEFT_POSITION) {
            Pose2d startPose = new Pose2d(0,0, Math.toRadians(0));
            odoDriveTrain.setPoseEstimate(startPose);
            Trajectory leftTape = odoDriveTrain.trajectoryBuilder(new Pose2d(0,0,0))
                    .lineToLinearHeading(new Pose2d(-16,0,Math.toRadians(0)))
                    .build();
            //strafeleft(16);
            odoDriveTrain.followTrajectory(leftTape);
            Pose2d startPose2 = leftTape.end();
            odoDriveTrain.setPoseEstimate(startPose2);
        } else if (targetZone == RIGHT_POSITION) {

        } else if (targetZone == MIDDLE_POSITION) {

        }

    }

    protected void park() {
        //straferight(24); strafe don't work
        odoDriveTrain.turn(Math.toRadians(adjustTurn(90)));
        back(20);
        odoDriveTrain.turn(Math.toRadians(adjustTurn(-90)));

    }
}
