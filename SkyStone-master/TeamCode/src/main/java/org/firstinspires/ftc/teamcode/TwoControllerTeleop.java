package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Two Controller Teleop", group="Teleop")
public class TwoControllerTeleop extends LinearOpMode{

    public void runOpMode() {
        Config config = new Config();
        ClawFunctions cf = new ClawFunctions();
        waitForStart();
        while(opModeIsActive()) {

            if(gamepad1.right_trigger == 1 && gamepad1.left_trigger == 0)
            {
            }
            if(gamepad1.left_trigger == 1 && gamepad1.right_trigger == 0)
            {
                cf.close(config);
            }

            Servo winch1 = config.VSWinch;
            double position = winch1.getPosition();

            if(gamepad2.left_trigger == 1 && gamepad2.right_trigger == 0)
            {
                position -= .1;
                winch1.setPosition(position);
            }

            if(gamepad2.left_trigger == 0 && gamepad2.right_trigger == 1)
            {
                position += .1;
                winch1.setPosition(position);
            }

            if(gamepad2.left_trigger == 0 && gamepad2.right_trigger == 0)
            {
                winch1.setPosition(position);
            }

//            Servo winch2 = config.HSWinch;
//            double position2 = winch2.getPosition();
//
//            if(gamepad2.left_bumper == true && gamepad2.right_bumper == false)
//            {
//                position2 -= .1;
//                winch2.setPosition(position2);
//            }
//
//            if(gamepad2.left_bumper == false && gamepad2.right_bumper == true)
//            {
//                position2 += .1;
//                winch2.setPosition(position2);
//            }
//
//            if(gamepad2.left_bumper == false && gamepad2.right_bumper == false)
//            {
//                winch1.setPosition(position2);
//            }


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

    }

}
