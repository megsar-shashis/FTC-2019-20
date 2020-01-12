/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * This file illustrates the concept of driving up to a line and then stopping.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code shows using two different light sensors:
 *   The Primary sensor shown in this code is a legacy NXT Light sensor (called "sensor_light")
 *   Alternative "commented out" code uses a MR Optical Distance Sensor (called "sensor_ods")
 *   instead of the LEGO sensor.  Chose to use one sensor or the other.
 *
 *   Setting the correct WHITE_THRESHOLD value is key to stopping correctly.
 *   This should be set half way between the light and dark values.
 *   These values can be read on the screen once the OpMode has been INIT, but before it is STARTED.
 *   Move the senso on asnd off the white line and not the min and max readings.
 *   Edit this code to make WHITE_THRESHOLD half way between the min and max.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="AutonomousBBlue", group="Auto")
//@Disabled
public class AutonomousBuildingBlue extends LinearOpMode {

    /* Declare OpMode members. */
    Config c = new Config();   // Use a Pushbot's hardware
    MecanumFunctions mecanumFunctions = new MecanumFunctions(this);
    AutonomousFunctions f = new AutonomousFunctions(this);
    ClawFunctions cf = new ClawFunctions();

    @Override
    public void runOpMode() {

        /* Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        c.init(hardwareMap);
        waitForStart();

        cf.openf(c);

        mecanumFunctions.MoveRobot(c, MecanumFunctions.MoveAction.SlideLeft, 80, 0.5 );
        telemetry.addLine("done 1");
        telemetry.update();
        sleep(500);
        telemetry.clearAll();

        mecanumFunctions.MoveRobot(c, MecanumFunctions.MoveAction.MoveForward, 18, 0.5 );
        telemetry.addLine("done 2");
        telemetry.update();
        sleep(500);
        telemetry.clearAll();

        mecanumFunctions.MoveRobot(c, MecanumFunctions.MoveAction.SlideLeft, 15, 0.5 );
        telemetry.addLine("done 3");
        telemetry.update();
        sleep(500);
        telemetry.clearAll();

        mecanumFunctions.MoveRobot(c, MecanumFunctions.MoveAction.MoveForward, 15, 0.5 );
        telemetry.addLine("done 4");
        telemetry.update();
        sleep(500);
        telemetry.clearAll();

        c.leftPull.setPosition(0);
        c.rightPull.setPosition(1);
        telemetry.addLine("done 5");
        telemetry.update();
        sleep(500);
        telemetry.clearAll();

        mecanumFunctions.MoveRobot(c, MecanumFunctions.MoveAction.MoveBackward, 50, 1.0);
        telemetry.addLine("done 6");
        telemetry.update();
        sleep(100);
        telemetry.clearAll();

        c.leftPull.setPosition(1);
        c.rightPull.setPosition(0);
        telemetry.addLine("done 7");
        telemetry.update();
        sleep(100);
        telemetry.clearAll();

        mecanumFunctions.MoveRobot(c, MecanumFunctions.MoveAction.SlideRight, 50, 1.0);
        telemetry.addLine("done 8");
        telemetry.update();
        sleep(500);
        telemetry.clearAll();

    }
}