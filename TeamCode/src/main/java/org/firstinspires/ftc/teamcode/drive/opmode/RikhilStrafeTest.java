package org.firstinspires.ftc.teamcode.drive.opmode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.Gyro;

@TeleOp(name="Rikhil's tests", group = "drive")
public class RikhilStrafeTest extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        Gyro gyro = new Gyro(hardwareMap);
        gyro.init();
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Angle", gyro.getAngle());
            telemetry.update();
        }

    }
}
