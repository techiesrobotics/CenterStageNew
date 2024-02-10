/*
Copyright (c) 2018 FIRST

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * {@link ObjectDetectionWDistance} illustrates how to use the REV Robotics
 * Time-of-Flight Range Sensor.
 *
 * The op mode assumes that the range sensor is configured with a name of "sensor_range".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * @see <a href="http://revrobotics.com">REV Robotics Web Page</a>
 */
@TeleOp(name = "Sensor: ObjectDetectionWDistance", group = "Sensor")
//@Disabled
public class ObjectDetectionWDistance extends LinearOpMode {

    private Rev2mDistanceSensor leftsensorRange;
    private Rev2mDistanceSensor rightsensorRange;
    private byte position = 2;

    @Override
    public void runOpMode() {
        // you can use this as a regular DistanceSensor.
        leftsensorRange = hardwareMap.get(Rev2mDistanceSensor.class, "leftDistance");
        rightsensorRange = hardwareMap.get(Rev2mDistanceSensor.class, "rightDistance");

        leftsensorRange.initialize();
        rightsensorRange.initialize();
        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.

        telemetry.addData(">>", "Press start to continue");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()) {
            // generic DistanceSensor methods.
            if (leftsensorRange.getDistance(DistanceUnit.INCH) > 6 && leftsensorRange.getDistance(DistanceUnit.INCH) < 15){
                position = 1;
            } else if (rightsensorRange.getDistance(DistanceUnit.INCH) > 6 && rightsensorRange.getDistance(DistanceUnit.INCH) < 15){
                position = 3;
            } else {
                position = 2;
            }
            telemetry.addData("position", position);
            telemetry.addData("deviceName", leftsensorRange.getDeviceName() );
            telemetry.addData("range", String.format("%.01f mm", leftsensorRange.getDistance(DistanceUnit.MM)));
            telemetry.addData("range", String.format("%.01f cm", leftsensorRange.getDistance(DistanceUnit.CM)));
            telemetry.addData("range", String.format("%.01f m", leftsensorRange.getDistance(DistanceUnit.METER)));
            telemetry.addData("range", String.format("%.01f in", leftsensorRange.getDistance(DistanceUnit.INCH)));
            telemetry.addData("deviceName", rightsensorRange.getDeviceName() );

            telemetry.addData("range", String.format("%.01f mm", rightsensorRange.getDistance(DistanceUnit.MM)));
            telemetry.addData("range", String.format("%.01f cm", rightsensorRange.getDistance(DistanceUnit.CM)));
            telemetry.addData("range", String.format("%.01f m", rightsensorRange.getDistance(DistanceUnit.METER)));
            telemetry.addData("range", String.format("%.01f in", rightsensorRange.getDistance(DistanceUnit.INCH)));
            // Rev2mDistanceSensor specific methods.

            telemetry.update();
        }
    }
//code here so I can commite
}