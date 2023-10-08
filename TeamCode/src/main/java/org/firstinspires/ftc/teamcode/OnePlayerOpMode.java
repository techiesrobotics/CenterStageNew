
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="1 Player: DriverControlOpMode", group="Linear Opmode")
public class OnePlayerOpMode extends org.firstinspires.ftc.teamcode.DriverControlOpMode.TechiesOpMode {
    public double getTurn() {
        double turn = gamepad1.right_stick_x;
        return turn;
    }

    public double getDrivefb() {
        double drivefb = -gamepad1.left_stick_y;
        return drivefb;
    }


    public double getDrivelr() {
        double drivelr = gamepad1.left_stick_x;
        return drivelr;
    }


}

