package org.firstinspires.ftc.teamcode.DriverControl;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.DriverControl.TechiesOpMode;

@TeleOp(name="2 Player: DriverControlOpMode", group="Linear Opmode")
public class TwoPlayerOpMode extends TechiesOpMode {
    public double getTurn() {
        double turn = gamepad2.right_stick_x;
        return turn;
    }

    public double getDrivefb() {
        double drivefb = -gamepad2.left_stick_y;
        return drivefb;
    }


    public double getDrivelr() {
        double drivelr = gamepad2.left_stick_x;
        return drivelr;
    }



}

