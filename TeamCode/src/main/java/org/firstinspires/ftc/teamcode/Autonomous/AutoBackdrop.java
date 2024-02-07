package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.tensorflow.lite.InterpreterFactory;

import kotlin.math.UMathKt;

abstract public class AutoBackdrop extends AutoParent{

    protected void goToTapeFromStart(int targetZone) {
        //make it move more if it is on position 2
        forward(4);// -(targetZone%2 -1)*2);
        if (targetZone == LEFT_POSITION) {
            odoDriveTrain.turn(adjustTurn(Math.toRadians(-94)));
            sleep(800);
            strafeleft(2);
            back(22);
        } else if (targetZone == RIGHT_POSITION) {
            odoDriveTrain.turn(adjustTurn(Math.toRadians(-96)));
        } else if (targetZone == MIDDLE_POSITION) {
            strafeleft(adjustTurn(4));
            back(1);

        }
    }
    protected void goToBackdrop(int targetZone){
        if (targetZone == LEFT_POSITION) {
            lineToSpline(-15,adjustTrajectorydistance(-12),0);
        } else if (targetZone == RIGHT_POSITION) {
            lineToSpline(-36, adjustTrajectorydistance(8), 0);
        } else if (targetZone == MIDDLE_POSITION) {
            back(2);
            odoDriveTrain.turn(adjustTurn(Math.toRadians(-96)));
            lineToSpline(-31, adjustTrajectorydistance(-2), 0);
        }

    }

    protected void park() {
        odoDriveTrain.turn(adjustTurn(Math.toRadians(7)));
        straferight(adjustTrajectorydistance(40));
        back(15);


    }
}
