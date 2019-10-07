package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Robowasps Teleop")
public class RoboWaspsTeleop_two_controller extends LinearOpMode{

    public void runOpMode() {
        //"winch_servo" "drop_servo"
        //intake_motor, linear_slide_up, chain_arm
        Servo markerServo = hardwareMap.get(Servo.class, "marker_servo");//hub 2 - port 0

        Servo winchServo = hardwareMap.get(Servo.class, "winch_servo");
        Servo dropServo = hardwareMap.get(Servo.class, "drop_servo");

        DcMotor chainArm = hardwareMap.get(DcMotor.class, "chain_arm");
        DcMotor linearSlideUp = hardwareMap.get(DcMotor.class, "linear_slide_up");
        DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        DcMotor leadScrewMotor = hardwareMap.get(DcMotor.class, "lead_screw");
        double fwdBackPower, strafePower, turnPower, maxPower;
        double leftFrontPower, rightFrontPower;
        double leftBackPower, rightBackPower;

        DcMotor leftFrontMotor = hardwareMap.dcMotor.get("left_front");
        DcMotor rightFrontMotor = hardwareMap.dcMotor.get("right_front");
        DcMotor leftBackMotor = hardwareMap.dcMotor.get("left_back");
        DcMotor rightBackMotor = hardwareMap.dcMotor.get("right_back");
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();
        while(opModeIsActive()) {
            markerServo.setPosition(0.25);
            // control the winch to move the horizontal slide - extend Y, retract A

            if(gamepad2.y == true) {
                winchServo.setPosition(0);
            }

            if(gamepad2.a == true) {
                winchServo.setPosition(0.35);
            }

            // control the lead screw motor - extend B, retract X. Pressing the Dpad right will make the lead screw come up and down faster
            if(gamepad1.x == true || gamepad1.b == true) {
                if (gamepad1.x == true) {
                    if(gamepad1.dpad_right == true) {
                        leadScrewMotor.setPower(-1);
                    } else {
                        leadScrewMotor.setPower(-0.2);
                    }
                }

                if (gamepad1.b == true) {
                    if(gamepad1.dpad_right == true) {
                        leadScrewMotor.setPower(1);
                    } else {
                        leadScrewMotor.setPower(0.2);
                    }
                }
            }
            else {
                leadScrewMotor.setPower(0);
            }


            // run the intake motor by pressing the right bumper
            if(gamepad2.right_bumper == true) {
                intakeMotor.setPower(0.5);
            } else {
                intakeMotor.setPower(0);
            }

            // control the vertical linear slide - extend is Dpad up, Retract is Dpad down
            if(gamepad2.dpad_up == true) {
                linearSlideUp.setPower(0.5);
            } else if(gamepad2.dpad_down == true){
                linearSlideUp.setPower(-0.5);
            } else {
                linearSlideUp.setPower(0);
            }

            // control the drop servo - press left bumper
            if(gamepad2.left_bumper == true) {
                dropServo.setPosition(1);
                sleep(2000); // wait 2 seconds for operation to finish
                dropServo.setPosition(0.1);
                sleep(2000);
            }

            // control the chain arm - to bring it down press left trigger, to bring it up press right trigger
            if(gamepad2.right_trigger > 0) {
                chainArm.setPower(gamepad2.right_trigger/2);
            } else if (gamepad2.left_trigger > 0) {
                chainArm.setPower(-gamepad2.left_trigger/3);
            } else {
                chainArm.setPower(0);
            }


            //control the mecanum wheels - left joystick up = forward and vise versa, left joystick left = slide left and vise versa
            fwdBackPower = -gamepad1.left_stick_y;
            strafePower = gamepad1.left_stick_x;
            turnPower = gamepad1.right_stick_x;

            leftFrontPower = fwdBackPower + turnPower + strafePower;
            rightFrontPower = fwdBackPower - turnPower - strafePower;
            leftBackPower = fwdBackPower + turnPower - strafePower;
            rightBackPower = fwdBackPower - turnPower + strafePower;

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
            leftFrontMotor.setPower(leftFrontPower);
            rightFrontMotor.setPower(rightFrontPower);
            leftBackMotor.setPower(leftBackPower);
            rightBackMotor.setPower(rightBackPower);
        }

    }

}
