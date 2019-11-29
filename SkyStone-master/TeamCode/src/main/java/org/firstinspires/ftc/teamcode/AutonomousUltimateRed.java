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

@Autonomous(name="AutonomousURed", group="Auto")
//@Disabled
public class AutonomousUltimateRed extends LinearOpMode {

    /* Declare OpMode members. */
    Config c = new Config();   // Use a Pushbot's hardware
    AutonomousFunctions f = new AutonomousFunctions();
    ClawFunctions cf = new ClawFunctions();

    @Override
    public void runOpMode() {

        /* Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        c.init(hardwareMap);

        //move forward ~ 12"
        f.MecanumMoveForwardInInches(c.leftFront, c.rightFront, c.leftBack, c.rightBack, 0.3, 12);

        //check for case 1,2,3
        SkystoneVuforia.SkystoneLocation locationPrint = f.FindSkystone();
        telemetry.addData("location", locationPrint.toString());
        telemetry.update();

        if(locationPrint.toString() == "LEFT")
        {
            f.MecanumSlideLeftInInches(c.leftFront, c.rightFront, c.leftBack, c.rightBack, 0.3, 8);
            cf.open(c);
            f.MecanumMoveForwardInInches(c.leftFront, c.rightFront, c.leftBack, c.rightBack, 0.3, 16);
            cf.close(c);
            double p = c.VSWinch.getPosition();
            p+=0.01;
            c.VSWinch.setPosition(p);
            f.MecanumMoveBackwardInInches(c.leftFront, c.rightFront, c.leftBack, c.rightBack, 0.3, 16);
            f.MecanumSlideRightInInches(c.leftFront, c.rightFront, c.leftBack, c.rightBack, 0.3, 16);

        }
        if(locationPrint.toString() == "RIGHT")
        {
            f.MecanumSlideRightInInches(c.leftFront, c.rightFront, c.leftBack, c.rightBack, 0.3, 8);
            cf.open(c);
            f.MecanumMoveForwardInInches(c.leftFront, c.rightFront, c.leftBack, c.rightBack, 0.3, 16);
            cf.close(c);
            double p = c.VSWinch.getPosition();
            p+=0.01;
            c.VSWinch.setPosition(p);
            f.MecanumMoveBackwardInInches(c.leftFront, c.rightFront, c.leftBack, c.rightBack, 0.3, 16);
        }
        if(locationPrint.toString() == "CENTER")
        {
            cf.open(c);
            f.MecanumMoveForwardInInches(c.leftFront, c.rightFront, c.leftBack, c.rightBack, 0.3, 16);
            cf.close(c);
            double p = c.VSWinch.getPosition();
            p+=0.01;
            c.VSWinch.setPosition(p);
            f.MecanumMoveBackwardInInches(c.leftFront, c.rightFront, c.leftBack, c.rightBack, 0.3, 16);
            f.MecanumSlideRightInInches(c.leftFront, c.rightFront, c.leftBack, c.rightBack, 0.3, 8);
        }
        if(locationPrint.toString() == "NONE")
        {
            cf.open(c);
            f.MecanumMoveForwardInInches(c.leftFront, c.rightFront, c.leftBack, c.rightBack, 0.3, 16);
            cf.close(c);
            double p = c.VSWinch.getPosition();
            p+=0.01;
            c.VSWinch.setPosition(p);
            f.MecanumMoveBackwardInInches(c.leftFront, c.rightFront, c.leftBack, c.rightBack, 0.3, 16);
            f.MecanumSlideRightInInches(c.leftFront, c.rightFront, c.leftBack, c.rightBack, 0.3, 8);
        }

        //slide right ~72"
        f.MecanumSlideRightInInches(c.leftFront, c.rightFront, c.leftBack, c.rightBack, 0.3, 72);

        //move forward to foundation ~ 12"
        f.MecanumMoveForwardInInches(c.leftFront, c.rightFront, c.leftBack, c.rightBack, 0.3, 12);

        cf.open(c);

        //move backward to foundation ~ 12"
        f.MecanumMoveBackwardInInches(c.leftFront, c.rightFront, c.leftBack, c.rightBack, 0.3, 12);

        //turn 180 deg
        f.MecanumTurningInDegrees(c.leftFront, c.rightFront, c.leftBack, c.rightBack, 0.3, 180);

        //move backward to foundation ~ 12"
        f.MecanumMoveBackwardInInches(c.leftFront, c.rightFront, c.leftBack, c.rightBack, 0.3, 12);

        //clamp pull servos

        //move forward ~ 30"
        f.MecanumMoveForwardInInches(c.leftFront, c.rightFront, c.leftBack, c.rightBack, 0.3, 30);

        //release pull servos

        //slide left ~ 52"
        f.MecanumSlideRightInInches(c.leftFront, c.rightFront, c.leftBack, c.rightBack, 0.3, 52);

    }
}
