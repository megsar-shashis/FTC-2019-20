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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

@TeleOp(name="TeleopTC", group="TeleopTC")
//@Disabled
public class TeleopTwoController1 extends OpMode{

    /* Declare OpMode members. */
    ClawFunctions cf       = new ClawFunctions(); // use the class created to define a Pushbot's hardware
    Config config = new Config();


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        config.init(hardwareMap);
        config.leftFront.setDirection(DcMotor.Direction.REVERSE);
        config.leftBack.setDirection(DcMotor.Direction.REVERSE);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
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
        config.orient.setPosition(0.4);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

/**************************PULL********************************************/

        if(gamepad1.x == true)
        {
            cf.pull(config);
        }
        if(gamepad1.b == true)
        {
            cf.nopull(config);
        }

/************************CLAW***************************/

        if(gamepad1.left_bumper ==  true){
            cf.openl(config);
            //open little
        }
        if(gamepad1.right_bumper == true){
            cf.close(config);
            //close
        }
        if(gamepad1.y == true)
        {
            cf.openf(config);
        }
/**************************LINEAR SLIDE************************************/
        Servo winch1 = config.winch1;
        double position = winch1.getPosition();
        telemetry.addData("winch position:", position);

//        winch1.setPosition((0.05 + (gamepad2.right_trigger / 2) - (gamepad2.left_trigger / 2)));

        if(gamepad1.left_trigger == 1 && gamepad1.right_trigger == 0)
        {
            position -= .1;
            winch1.setPosition(position);
        }

        if(gamepad1.left_trigger == 0 && gamepad1.right_trigger == 1)
        {
            position += .1;
            winch1.setPosition(position);
        }

        if(gamepad1.left_trigger == 0 && gamepad1.right_trigger == 0)
        {
            position = 0.05;
            winch1.setPosition(position);
        }
/************************************CHASSIS******************************/
        double fwdBackPower = -gamepad1.left_stick_y;
        double strafePower = gamepad1.left_stick_x;
        double turnPower = gamepad1.right_stick_x;

        double leftFrontPower = fwdBackPower + turnPower + strafePower;
        double rightFrontPower = fwdBackPower - turnPower - strafePower;
        double leftBackPower = fwdBackPower + turnPower - strafePower;
        double rightBackPower = fwdBackPower - turnPower + strafePower;

        double maxPower = Math.abs(leftFrontPower);
        if(Math.abs(rightFrontPower) > maxPower) {maxPower = Math.abs(rightFrontPower);}
        if(Math.abs(leftBackPower) > maxPower) {maxPower = Math.abs(leftBackPower);}
        if(Math.abs(rightBackPower) > maxPower) {maxPower = Math.abs(rightBackPower);}

        if (maxPower > 1) {
            leftFrontPower = leftFrontPower/maxPower;
            rightFrontPower = rightFrontPower/maxPower;
            leftBackPower = leftBackPower/maxPower;
            rightBackPower = rightBackPower/maxPower;
        }

        telemetry.addData("powers", "|%.3f|%.3f|%.3f|%.3f|",
                leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
        telemetry.update();
        config.leftFront.setPower(leftFrontPower);
        config.rightFront.setPower(rightFrontPower);
        config.leftBack.setPower(leftBackPower);
        config.rightBack.setPower(rightBackPower);



    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}