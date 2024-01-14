package org.firstinspires.ftc.teamcode.drive.opmode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(group = "drive")
public class RikhilStrafeTest extends LinearOpMode {

    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        leftFront = hardwareMap.get(DcMotor.class, "frontleft");
        leftBack = hardwareMap.get(DcMotor.class, "backleft");
        rightFront = hardwareMap.get(DcMotor.class, "frontright");
        rightBack = hardwareMap.get(DcMotor.class, "backright");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);


        while (opModeIsActive()) {
           if (gamepad1.a) {
               leftFront.setPower(0.5);
               leftBack.setPower(0.5);
               rightFront.setPower(0.5);
               rightBack.setPower(0.5);
           } else {
               leftFront.setPower(0);
               leftBack.setPower(0);
               rightFront.setPower(0);
               rightBack.setPower(0);
           }
        }
    }
}
