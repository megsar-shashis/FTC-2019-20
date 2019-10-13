package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Robowasps Teleop")
public class RoboWaspsTeleop_two_controller extends LinearOpMode{

    public void runOpMode() {
        Config config = new Config();
        waitForStart();
        while(opModeIsActive()) {


            if(gamepad2.y == true) {
                config.HSWinch.setPosition(0);
            }

            if(gamepad2.a == true) {
                config.HSWinch.setPosition(0.35);
            }

            // control the lead screw motor - extend B, retract X. Pressing the Dpad right will make the lead screw come up and down faster
            if(gamepad1.x == true || gamepad1.b == true) {
                if (gamepad1.x == true) {
                    if(gamepad1.dpad_right == true) {
                        config.leftBack.setPower(-1);
                    } else {
                        config.leftBack.setPower(-0.2);
                    }
                }

                if (gamepad1.b == true) {
                    if(gamepad1.dpad_right == true) {
                        config.leftBack.setPower(1);
                    } else {
                        config.leftBack.setPower(0.2);
                    }
                }
            }
            else {
                config.leftBack.setPower(0);
            }


//            // run the intake motor by pressing the right bumper
//            if(gamepad2.right_bumper == true) {
//                intakeMotor.setPower(0.5);
//            } else {
//                intakeMotor.setPower(0);
//            }
//
//            // control the vertical linear slide - extend is Dpad up, Retract is Dpad down
//            if(gamepad2.dpad_up == true) {
//                linearSlideUp.setPower(0.5);
//            } else if(gamepad2.dpad_down == true){
//                linearSlideUp.setPower(-0.5);
//            } else {
//                linearSlideUp.setPower(0);
//            }
//
//            // control the drop servo - press left bumper
//            if(gamepad2.left_bumper == true) {
//                dropServo.setPosition(1);
//                sleep(2000); // wait 2 seconds for operation to finish
//                dropServo.setPosition(0.1);
//                sleep(2000);
//            }
//
//            // control the chain arm - to bring it down press left trigger, to bring it up press right trigger
//            if(gamepad2.right_trigger > 0) {
//                chainArm.setPower(gamepad2.right_trigger/2);
//            } else if (gamepad2.left_trigger > 0) {
//                chainArm.setPower(-gamepad2.left_trigger/3);
//            } else {
//                chainArm.setPower(0);
//            }
//
//
//            //control the mecanum wheels - left joystick up = forward and vise versa, left joystick left = slide left and vise versa
//            fwdBackPower = -gamepad1.left_stick_y;
//            strafePower = gamepad1.left_stick_x;
//            turnPower = gamepad1.right_stick_x;
//
//            leftFrontPower = fwdBackPower + turnPower + strafePower;
//            rightFrontPower = fwdBackPower - turnPower - strafePower;
//            leftBackPower = fwdBackPower + turnPower - strafePower;
//            rightBackPower = fwdBackPower - turnPower + strafePower;
//
//            maxPower = Math.abs(leftFrontPower);
//            if(Math.abs(rightFrontPower) > maxPower) {maxPower = Math.abs(rightFrontPower);}
//            if(Math.abs(leftBackPower) > maxPower) {maxPower = Math.abs(leftBackPower);}
//            if(Math.abs(rightBackPower) > maxPower) {maxPower = Math.abs(rightBackPower);}
//
//            if (maxPower > 1) {
//                leftFrontPower = leftFrontPower/maxPower;
//                rightFrontPower = rightFrontPower/maxPower;
//                leftBackPower = leftBackPower/maxPower;
//                rightBackPower = rightBackPower/maxPower;
//            }
//
//            telemetry.addData("powers", "|%.3f|%.3f|%.3f|%.3f|",
//                    leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
//            telemetry.update();
//            leftFrontMotor.setPower(leftFrontPower);
//            rightFrontMotor.setPower(rightFrontPower);
//            leftBackMotor.setPower(leftBackPower);
//            rightBackMotor.setPower(rightBackPower);
        }

    }

}
