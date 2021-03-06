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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="LSlideTest", group="Test")
//@Disabled
public class LSlideTest extends OpMode{

    /* Declare OpMode members. */
    ClawFunctions cf       = new ClawFunctions(); // use the class created to define a Pushbot's hardware
    Config robot = new Config();

    Servo winch1;
    Servo winch2;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        winch1 = robot.winch1;
        winch2 = robot.winch2;

        winch1.setPosition(1);
        winch2.setPosition(0);
        // Send telemetry message to signify robot waiting;

        telemetry.addData("winch1 initpos: ", winch1.getPosition());
        telemetry.addData("winch2 initpos: ", winch2.getPosition());
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        if(gamepad1.left_bumper ==  true){
            cf.openl(robot);
            //open little
        }
        if(gamepad1.right_bumper == true){
            cf.close(robot);
            //close
        }
        if(gamepad1.y == true)
        {
            cf.openf(robot);
        }


//        winch1.setPosition(position2-(gamepad1.right_trigger/10)-(gamepad1.left_trigger/10));

        double position1 = winch1.getPosition();
        double position2 = winch2.getPosition();
        //double delta = (gamepad1.right_trigger/10)-(gamepad1.left_trigger/10);
        double delta = 0.05;

        telemetry.addData("position1: ", position1);
        telemetry.addData("position2", position2);

        telemetry.addData("winch1 pos before set: ", winch1.getPosition());
        telemetry.addData("winch2 pos before set: ", winch2.getPosition());
        if(gamepad1.left_trigger == 1)
        {

            position1 = position1 + delta;
            position2 = position2 + delta;
        }
        if(gamepad1.right_trigger == 1)
        {
            position1 = position1 - delta;
            position2 = position2 - delta;
        }
        if(gamepad1.left_trigger == 0 && gamepad1.right_trigger == 0)
        {
            position1 +=0;
            position2 +=0;
        }

        telemetry.addData("position1: ", position1);
        telemetry.addData("position2", position2);
        winch1.setPosition(position1);
        winch2.setPosition(position2);

        telemetry.addData("winch1 pos after set: ", winch1.getPosition());
        telemetry.addData("winch2 pos after set: ", winch2.getPosition());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
