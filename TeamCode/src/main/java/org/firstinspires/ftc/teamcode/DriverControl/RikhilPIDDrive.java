
package org.firstinspires.ftc.teamcode.DriverControl;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.DriverControl.TechiesOpMode;
import org.firstinspires.ftc.teamcode.TechiesHardwareWithoutDriveTrain;
import org.firstinspires.ftc.teamcode.TechiesRobotHardware;
import org.firstinspires.ftc.teamcode.util.Gyro;

@TeleOp(name = "Rikhil's 1 Player: DriverControlOpMode", group = "Linear Opmode")
public class RikhilPIDDrive extends LinearOpMode {

    enum PID {
        P, D, I
    }

    ;

    private static PIDCoefficients coefs = new PIDCoefficients(0, 0, 0);
    private static PIDFController pidController = new PIDFController(coefs);
    private double refAngle = 0;
    private PID pidVar = PID.P;
    boolean isUp;
    int startingPosition;

    private ElapsedTime runtime = new ElapsedTime();


    TechiesRobotHardware robot = new TechiesRobotHardware();
    TechiesHardwareWithoutDriveTrain robotCore;
    private Gyro gyro;
    public static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    public static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    public static final double WHEEL_DIAMETER_INCHES = 2.0;     // For figuring circumference
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

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


    @Override
    public void runOpMode() throws InterruptedException {
        isUp = false;
        robotCore = new TechiesHardwareWithoutDriveTrain(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot.init(hardwareMap);
        gyro = new Gyro(hardwareMap);
        gyro.init();
        startingPosition = robotCore.arm.getCurrentPosition();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;
            double backleftPower;
            double backrightPower;
            double Multiplier = 1;
            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.\

            //moved on top
            //This code below has been disabled for the autonomous since the claw is bugging out
            //arm and wrist needs tp be

            if (gamepad1.a) {
                if (pidVar == PID.P) {
                    pidVar = PID.D;
                } else if (pidVar == PID.D) {
                    pidVar = PID.I;
                } else {
                    pidVar = PID.P;
                }
            } else if (gamepad1.dpad_up || gamepad1.dpad_down) {
                int factor = gamepad1.dpad_up ? 1 : -1;

                if (pidVar == PID.P) {
                    coefs.kP += 0.01 * factor;
                } else if (pidVar == PID.D) {
                    coefs.kD += 0.01 * factor;
                } else if (pidVar == PID.I) {
                    coefs.kI += 0.01 * factor;
                }

                pidController = new PIDFController(coefs);
                sleep(500);
            }



            double turn = getTurn();
            double drivefb = getDrivefb();  //-gamepad1.left_stick_y;
            double drivelr = getDrivelr(); //gamepad1.left_stick_x;
            double pidValue = 0.0;


            if (turn != 0) {
                refAngle = gyro.getAngle();
                pidValue = 0.0;
            } else {
                pidValue = pidController.update(gyro.getAngle() - refAngle);
            }

            telemetry.addData("Turning? ", String.valueOf(turn));
            telemetry.addData("Ref angle", String.valueOf(refAngle));
            telemetry.addData("PIDVALUE", String.valueOf(pidValue));
            telemetry.addData("TUNING VAR", String.valueOf(pidVar));
            telemetry.addData("P-I-D values", coefs.kP + " " + coefs.kI + " " + coefs.kD);

            leftPower = Range.clip(-drivefb + turn + drivelr + pidValue, -1, 1);
            rightPower = Range.clip(-drivefb - turn - drivelr - pidValue, -1, 1);
            backleftPower = Range.clip(-drivefb + turn - drivelr + pidValue, -1, 1);
            backrightPower = Range.clip(-drivefb - turn + drivelr - pidValue, -1, 1);


            // Send calculated power to wheels
            //  Multiplier = driveSpeed();
            robot.leftDrive.setPower(leftPower * Multiplier);
            robot.rightDrive.setPower(rightPower * Multiplier);
            robot.leftBack.setPower(backleftPower * Multiplier);
            robot.rightBack.setPower(backrightPower * Multiplier);
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("stickX", drivefb + turn + drivelr);
            telemetry.addData("Motor pos: ", robotCore.claw.getPosition());
            telemetry.addData("Position of wrist servo: ", robotCore.wrist.getPosition());
            telemetry.addData("Position of claw servo: ", robotCore.claw.getPosition());
            telemetry.update();
            // if (gamepad1.y) {
            // robotCore.droneLauncher.setPosition(0);

            // }

        }
    }

    public void encoderArm(double speed,
                           double armInches) {
        int newArmTarget;


        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newArmTarget = robotCore.arm.getCurrentPosition() + (int) (armInches * COUNTS_PER_INCH);

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

