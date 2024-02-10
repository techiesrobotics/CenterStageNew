package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.tensorflow.lite.InterpreterFactory;

import kotlin.math.UMathKt;

abstract public class AutoBackdrop extends AutoParent{
    int distanceAdd = 0;
    protected void goToTapeFromStart(int targetZone) {
        //make it move more if it is on position 2
        forward(4);// -(targetZone%2 -1)*2);
        if (targetZone == LEFT_POSITION) {
            odoDriveTrain.turn(adjustTurn(Math.toRadians(-94)));
            sleep(800);
            strafeleft(2);
            back(26);
        } else if (targetZone == RIGHT_POSITION) {
            distanceAdd+=10;
            odoDriveTrain.turn(adjustTurn(Math.toRadians(-96)));
            sleep(800);
            forward(5);
            sleep(300);
            back(10);
        } else if (targetZone == MIDDLE_POSITION) {
            strafeleft(adjustTurn(4));
            forward(5);
            back(11);

        }
    }
    protected void goToBackdrop(int targetZone){
        if (targetZone == LEFT_POSITION) {
            lineToSpline(-20,adjustTrajectorydistance(-18),0);
        } else if (targetZone == RIGHT_POSITION) {
            lineToSpline(-36, adjustTrajectorydistance(8), 0);
        } else if (targetZone == MIDDLE_POSITION) {
            back(2);
            odoDriveTrain.turn(adjustTurn(Math.toRadians(-96)));
            lineToSpline(-36, adjustTrajectorydistance(-2), 0);
        }

    }

    protected void park() {
        odoDriveTrain.turn(adjustTurn(Math.toRadians(7)));
        straferight(adjustTrajectorydistance(30+distanceAdd));
        back(10+distanceAdd);


    }
}
