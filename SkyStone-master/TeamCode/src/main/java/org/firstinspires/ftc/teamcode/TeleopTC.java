package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "RS Teleop")
public class TeleopTC extends LinearOpMode{

    /* Declare OpMode members. */
    ClawFunctions cf       = new ClawFunctions(); // use the class created to define a Pushbot's hardware
    Config config = new Config();

    public void runOpMode() {
        double fwdBackPower, strafePower, turnPower, maxPower;
        double leftFrontPower, rightFrontPower;
        double leftBackPower, rightBackPower;


        config.init(hardwareMap);
        config.leftFront.setDirection(DcMotor.Direction.REVERSE);
        config.leftBack.setDirection(DcMotor.Direction.REVERSE);

        config.orient.setPosition(0.4);

        waitForStart();
        while(opModeIsActive()) {
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
            Servo winch1 = config.winch2;
            double position = winch1.getPosition();
            telemetry.addData("winch position:", position);

//        winch1.setPosition((0.05 + (gamepad2.right_trigger / 2) - (gamepad2.left_trigger / 2)));

            if (gamepad1.right_trigger == 1) {
//            winch1.setPosition(.5);
                winch1.setPosition(position - .2);
            }
            if (gamepad1.left_trigger == 1) {
//            winch1.setPosition(.3);
                position = position + .2;
                winch1.setPosition(position);
            }


            //control the mecanum wheels - left joystick up = forward and vise versa, left joystick left = slide left and vise versa
            fwdBackPower = -gamepad1.left_stick_y;
            strafePower = gamepad1.left_stick_x;
            turnPower = gamepad1.right_stick_x;

            leftFrontPower = fwdBackPower + turnPower + strafePower;
            rightFrontPower = fwdBackPower - turnPower - strafePower;
            leftBackPower = -fwdBackPower + turnPower - strafePower;
            rightBackPower = -fwdBackPower - turnPower + strafePower;

            maxPower = Math.abs(leftFrontPower);
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

    }

}
