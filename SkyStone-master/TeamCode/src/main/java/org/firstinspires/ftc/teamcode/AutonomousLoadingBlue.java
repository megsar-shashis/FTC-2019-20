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
import com.qualcomm.robotcore.hardware.Servo;

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

@Autonomous(name="AutonomousLBlue", group="Auto")
//@Disabled
public class AutonomousLoadingBlue extends LinearOpMode {

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
        FindSkystoneOpenCV fs = new FindSkystoneOpenCV(this);
        int stonePosition = fs.FindSkystone();

        Servo winch = c.winch1;

        waitForStart();

        c.orient.setPosition(0.6);
        //cf.openf(c);
        // open claw to the top position
        c.claw.setPosition(1.0);
        //sleep(1000);

        double position = winch.getPosition();
        telemetry.addData("initial winch position:", position);
        telemetry.update();

        //move forward
        mecanumFunctions.MoveRobot(c, MecanumFunctions.MoveAction.MoveForward, 32.5, 0.5 );

        //move horizontally 8 * stoneposition
        mecanumFunctions.MoveRobot(c, MecanumFunctions.MoveAction.SlideRight, 9*stonePosition, 0.5 );
        sleep(100);

        telemetry.addLine("before claw");
        cf.close(c);
        telemetry.addLine("after claw");
        telemetry.update();
        // sleep one sec to grab the stone before move backward
        sleep(1000);

        // after grab the stone, need to move back to avoid bumping into the bridge
        mecanumFunctions.MoveRobot(c, MecanumFunctions.MoveAction.MoveBackward, 8, 0.5 );
        telemetry.addLine("move backward 8");
        telemetry.update();
        sleep(100);
        telemetry.clearAll();

        mecanumFunctions.MoveRobot(c, MecanumFunctions.MoveAction.SlideLeft, 11+9*stonePosition, 0.5 );
        telemetry.addLine("move left");
        telemetry.update();
        sleep(100);
        telemetry.clearAll();

        // after grab the stone, need to move back to avoid bumping into the bridge
        mecanumFunctions.MoveRobot(c, MecanumFunctions.MoveAction.MoveBackward, 4, 0.5 );
        telemetry.addLine("move back 4");
        telemetry.update();
        sleep(100);
        telemetry.clearAll();

        mecanumFunctions.MoveRobot(c, MecanumFunctions.MoveAction.SlideLeft, 40, 0.5 );
        telemetry.addLine("move right 25");
        telemetry.update();
        sleep(100);
        telemetry.clearAll();

        // lift the stone up
        winch.setPosition(0.4);
        sleep(4000);

        mecanumFunctions.MoveRobot(c, MecanumFunctions.MoveAction.SlideLeft, 37, 0.5 );
        telemetry.addLine("move right 37");
        telemetry.update();
        sleep(100)
        ;
        telemetry.clearAll();
        position = winch.getPosition();
        telemetry.addData("winch position:", position);

        mecanumFunctions.MoveRobot(c, MecanumFunctions.MoveAction.MoveForward, 16, 0.5 );
        telemetry.addLine("done 2");
        telemetry.update();
        sleep(100);
        telemetry.clearAll();
        // mecanumFunctions.MoveRobot(c, MecanumFunctions.MoveAction.SlideLeft, 5, 0.5 );
        telemetry.addLine("done 3");
        telemetry.update();
        sleep(100);
        telemetry.clearAll();

        //mecanumFunctions.MoveRobot(c, MecanumFunctions.MoveAction.MoveForward, 15, 0.5 );
        telemetry.addLine("done 4");
        telemetry.update();
        //sleep(1000);
        telemetry.clearAll();

        telemetry.addLine("before claw");
        //cf.openf(c);
        //keep the claw open up
        c.claw.setPosition(1.0);
        telemetry.addLine("after claw");
        sleep(1000);

        // down the winch
        winch.setPosition(0.0);
        sleep(1000);

        //mecanumFunctions.MoveRobot(c, MecanumFunctions.MoveAction.SlideLeft, 7, 0.5 );
        c.leftPull.setPosition(0);
        c.rightPull.setPosition(1);
        telemetry.addLine("done 5");
        telemetry.update();
        sleep(1000);
        telemetry.clearAll();

        mecanumFunctions.MoveRobot(c, MecanumFunctions.MoveAction.MoveBackward, 48, 1);
        telemetry.addLine("done 6");
        telemetry.update();
        sleep(100);
        telemetry.clearAll();

        c.leftPull.setPosition(1);
        c.rightPull.setPosition(0);
        telemetry.addLine("done 7");
        telemetry.update();
        sleep(1000);
        telemetry.clearAll();

        mecanumFunctions.MoveRobot(c, MecanumFunctions.MoveAction.SlideRight, 60, 0.5 );
        telemetry.addLine("done 8");
        telemetry.update();
        sleep(100);
        telemetry.clearAll();

//        c.orient.setPosition(0.4);
//        cf.openf(c);
//        sleep(1000);
//
//        fs.FindSkystoneAndMoveRobot();
//        telemetry.addLine("before claw");
//        cf.close(c);
//        telemetry.addLine("after claw");
//        telemetry.update();
//        sleep(1000);
//
//        f.MecanumMoveBackwardInInches(c.leftFront, c.rightFront, c.leftBack, c.rightBack, 0.8, 0.6, 0.5, 0.5,8);
//
//        //slide right ~80"
//        f.MecanumSlideLeftInInches(c.leftFront, c.rightFront, c.leftBack, c.rightBack, 0.716, 0.42, 0.35, 0.716,120);
//        double position = c.winch1.getPosition();
//        position += .1;
//        c.winch1.setPosition(position);
//
//        //move forward to foundation ~ 12"
//        f.MecanumMoveForwardInInches(c.leftFront, c.rightFront, c.leftBack, c.rightBack, 0.7, 0.6, 0.5, 0.5,12);
//
//        cf.openl(c);
//
//        //move backward away ~ 8"
//        f.MecanumMoveBackwardInInches(c.leftFront, c.rightFront, c.leftBack, c.rightBack, 0.8, 0.6, 0.5, 0.5, 8);
//        position = c.winch1.getPosition();
//        position -= .1;
//        c.winch1.setPosition(position);
//
//        //move left to park away ~ 48"
//        f.MecanumSlideRightInInches(c.leftFront, c.rightFront, c.leftBack, c.rightBack, 0.716, 0.42, 0.35, 0.716, 48);
    }
}

