package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by meghn on 10/1/2018.
 */

public class AutonomousFunctions {

    private LinearOpMode opMode;
    BNO055IMU imu;
    // **************************** START OF MOVE, SLIDE & TURN ROBOT ******************************************************

    public void MecanumSlideInInches
            (DcMotor leftFront, DcMotor rightFront,
             DcMotor leftBack, DcMotor rightBack, double power, double inches)
    {
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        //sets proper direction
        this.opMode.telemetry.addLine("Wheel Direction set");
        this.opMode.telemetry.update();

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //resets encoder

        double count = 2.5;

        double positionsPerInch = 1120/12.533;
        //positionsPerInch is the amount of counts the encoder makes to move 1 inch

        //target position to run to
        this.opMode.telemetry.addLine("set target");
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //runs to the position

        leftFront.setTargetPosition((int) (inches * positionsPerInch));
        leftBack.setTargetPosition((int) (inches * positionsPerInch));
        rightFront.setTargetPosition((int) (inches * positionsPerInch));
        rightBack.setTargetPosition((int) (inches * positionsPerInch));

        //ensures robot will go to the destination needed
        this.opMode.telemetry.addLine("run to");
        leftFront.setPower(power);
        leftBack.setPower(power);
        rightFront.setPower(power);
        rightBack.setPower(power);
        this.opMode.telemetry.addLine("set power");
        //sets speed at which robot will run

        while (opMode.opModeIsActive()
                && leftFront.isBusy()
                && leftBack.isBusy()
                && rightFront.isBusy()
                && rightBack.isBusy()) {
            this.opMode.telemetry.addData("Left Front Motor Current Position: ", leftFront.getCurrentPosition());
            this.opMode.telemetry.addData("Left Back Motor Current Position: ", leftBack.getCurrentPosition());
            this.opMode.telemetry.addData("Right Front Current Position: ", rightFront.getCurrentPosition());
            this.opMode.telemetry.addData("Right Back Motor Current Position: ", rightBack.getCurrentPosition());
            this.opMode.telemetry.update();
            //sends data on the positions of where the motor is located
        }
    }

    public void MecanumSlideRightInInches
            (DcMotor leftFront, DcMotor rightFront,
             DcMotor leftBack, DcMotor rightBack, double power, double inches)
    {
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        //sets proper direction

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //resets encoder

        double count = 2.5;

        double positionsPerInch = 1120/12.533;
        //positionsPerInch is the amount of counts the encoder makes to move 1 inch

        //target position to run to
        this.opMode.telemetry.addLine("set target");
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //runs to the position

        leftFront.setTargetPosition((int) (inches * positionsPerInch));
        leftBack.setTargetPosition((int) (inches * positionsPerInch));
        rightFront.setTargetPosition((int) (inches * positionsPerInch));
        rightBack.setTargetPosition((int) (inches * positionsPerInch));

        //ensures robot will go to the destination needed
        this.opMode.telemetry.addLine("run to");
        leftFront.setPower(power);
        leftBack.setPower(power);
        rightFront.setPower(power);
        rightBack.setPower(power);
        this.opMode.telemetry.addLine("set power");
        //sets speed at which robot will run

        while (opMode.opModeIsActive()
                && leftFront.isBusy()
                && leftBack.isBusy()
                && rightFront.isBusy()
                && rightBack.isBusy()) {
            this.opMode.telemetry.addData("Left Front Motor Current Position: ", leftFront.getCurrentPosition());
            this.opMode.telemetry.addData("Left Back Motor Current Position: ", leftBack.getCurrentPosition());
            this.opMode.telemetry.addData("Right Front Current Position: ", rightFront.getCurrentPosition());
            this.opMode.telemetry.addData("Right Back Motor Current Position: ", rightBack.getCurrentPosition());
            this.opMode.telemetry.update();
            //sends data on the positions of where the motor is located
        }
    }

    public void MecanumSlideLeftInInches
            (DcMotor leftFront, DcMotor rightFront,
             DcMotor leftBack, DcMotor rightBack, double power, double inches)
    {
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        //sets proper direction

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //resets encoder

        double count = 2.5;

        double positionsPerInch = 1120/12.533;
        //positionsPerInch is the amount of counts the encoder makes to move 1 inch

        //target position to run to
        this.opMode.telemetry.addLine("set target");
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //runs to the position

        leftFront.setTargetPosition((int) (inches * positionsPerInch));
        leftBack.setTargetPosition((int) (inches * positionsPerInch));
        rightFront.setTargetPosition((int) (inches * positionsPerInch));
        rightBack.setTargetPosition((int) (inches * positionsPerInch));

        //ensures robot will go to the destination needed
        this.opMode.telemetry.addLine("run to");
        leftFront.setPower(power);
        leftBack.setPower(power);
        rightFront.setPower(power);
        rightBack.setPower(power);
        this.opMode.telemetry.addLine("set power");
        //sets speed at which robot will run

        while (opMode.opModeIsActive()
                && leftFront.isBusy()
                && leftBack.isBusy()
                && rightFront.isBusy()
                && rightBack.isBusy()) {
            this.opMode.telemetry.addData("Left Front Motor Current Position: ", leftFront.getCurrentPosition());
            this.opMode.telemetry.addData("Left Back Motor Current Position: ", leftBack.getCurrentPosition());
            this.opMode.telemetry.addData("Right Front Current Position: ", rightFront.getCurrentPosition());
            this.opMode.telemetry.addData("Right Back Motor Current Position: ", rightBack.getCurrentPosition());
            this.opMode.telemetry.update();
            //sends data on the positions of where the motor is located
        }
    }

    public void MecanumMoveInInches
            (DcMotor leftFront, DcMotor rightFront,
             DcMotor leftBack, DcMotor rightBack, double power, double inches)
    {
        //positive slides right, negative slides left

        this.opMode.telemetry.addLine("Remember, positive inches slide right...");
        this.opMode.telemetry.addLine(" and negative inches slide left");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double positionsPerInch = 1120/12.605;
        //positionsPerInch is the amount of counts the encoder makes to move 1 inch

        //positionsToMove is the amount of counts the encoder must make to reach the target location

        //target position to run to
        this.opMode.telemetry.addLine("set target");
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setTargetPosition((int)(inches*positionsPerInch));
        rightFront.setTargetPosition((int)(inches*positionsPerInch));
        leftBack.setTargetPosition((int)(inches*positionsPerInch));
        rightBack.setTargetPosition((int)(inches*positionsPerInch));

        //ensures robot will go to the destination needed
        this.opMode.telemetry.addLine("run to");
        leftFront.setPower(power);
        rightFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);
        this.opMode.telemetry.addLine("set power");
        //sets speed at which robot will run, 'power' is input


        while(this.opMode.opModeIsActive()
                && leftFront.isBusy()
                && rightFront.isBusy()
                && leftBack.isBusy()
                && rightBack.isBusy())
        {
            this.opMode.telemetry.addData("Left Front Motor Current Position: ", leftFront.getCurrentPosition());
            this.opMode.telemetry.addData("Right Front Current Position: ", rightFront.getCurrentPosition());
            this.opMode.telemetry.addData("Left Back Motor Current Position: ", leftBack.getCurrentPosition());
            this.opMode.telemetry.addData("Right Back Motor Current Position: ", rightBack.getCurrentPosition());
            this.opMode.telemetry.update();
            //sends data on the positions of where the motor is located
        }

    }

    public void MecanumMoveForwardInInches
            (DcMotor leftFront, DcMotor rightFront,
             DcMotor leftBack, DcMotor rightBack, double power, double inches)
    {
        this.opMode.telemetry.addLine("Mecanum move forward ...");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double positionsPerInch = 1120/12.605;
        //positionsPerInch is the amount of counts the encoder makes to move 1 inch

        //positionsToMove is the amount of counts the encoder must make to reach the target location

        //target position to run to
        this.opMode.telemetry.addLine("set target");
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setTargetPosition((int)(inches*positionsPerInch));
        rightFront.setTargetPosition((int)(inches*positionsPerInch));
        leftBack.setTargetPosition((int)(inches*positionsPerInch));
        rightBack.setTargetPosition((int)(inches*positionsPerInch));

        //ensures robot will go to the destination needed
        this.opMode.telemetry.addLine("run to");
        leftFront.setPower(power);
        rightFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);
        this.opMode.telemetry.addLine("set power");
        //sets speed at which robot will run, 'power' is input


        while(this.opMode.opModeIsActive()
                && leftFront.isBusy()
                && rightFront.isBusy()
                && leftBack.isBusy()
                && rightBack.isBusy())
        {
            this.opMode.telemetry.addData("Left Front Motor Current Position: ", leftFront.getCurrentPosition());
            this.opMode.telemetry.addData("Right Front Current Position: ", rightFront.getCurrentPosition());
            this.opMode.telemetry.addData("Left Back Motor Current Position: ", leftBack.getCurrentPosition());
            this.opMode.telemetry.addData("Right Back Motor Current Position: ", rightBack.getCurrentPosition());
            this.opMode.telemetry.update();
            //sends data on the positions of where the motor is located
        }

    }

    public void MecanumMoveBackwardInInches
            (DcMotor leftFront, DcMotor rightFront,
             DcMotor leftBack, DcMotor rightBack, double power, double inches)
    {
        //positive slides right, negative slides left

        this.opMode.telemetry.addLine("Remember, positive inches slide right...");
        this.opMode.telemetry.addLine(" and negative inches slide left");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double positionsPerInch = 1120/12.605;
        //positionsPerInch is the amount of counts the encoder makes to move 1 inch

        //positionsToMove is the amount of counts the encoder must make to reach the target location

        //target position to run to
        this.opMode.telemetry.addLine("set target");
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setTargetPosition((int)(inches*positionsPerInch));
        rightFront.setTargetPosition((int)(inches*positionsPerInch));
        leftBack.setTargetPosition((int)(inches*positionsPerInch));
        rightBack.setTargetPosition((int)(inches*positionsPerInch));

        //ensures robot will go to the destination needed
        this.opMode.telemetry.addLine("run to");
        leftFront.setPower(power);
        rightFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);
        this.opMode.telemetry.addLine("set power");
        //sets speed at which robot will run, 'power' is input


        while(this.opMode.opModeIsActive()
                && leftFront.isBusy()
                && rightFront.isBusy()
                && leftBack.isBusy()
                && rightBack.isBusy())
        {
            this.opMode.telemetry.addData("Left Front Motor Current Position: ", leftFront.getCurrentPosition());
            this.opMode.telemetry.addData("Right Front Current Position: ", rightFront.getCurrentPosition());
            this.opMode.telemetry.addData("Left Back Motor Current Position: ", leftBack.getCurrentPosition());
            this.opMode.telemetry.addData("Right Back Motor Current Position: ", rightBack.getCurrentPosition());
            this.opMode.telemetry.update();
            //sends data on the positions of where the motor is located
        }

    }

    public void MecanumMoveInInchesWithRampUpSpeed
            (DcMotor leftFront, DcMotor rightFront,
             DcMotor leftBack, DcMotor rightBack, double power, double inches)
    {
        ElapsedTime motorTimer = new ElapsedTime();

        double motorStartPower = 0.1;

        double motorCurrentPower = motorStartPower;
        motorTimer.reset();
        double currentTime = motorTimer.milliseconds();
        double maxTimeToRampUp = 1000;
        double timeStep = 10;
        double powerUpPerTimeStep = (power - motorStartPower)/(maxTimeToRampUp/timeStep);

        double positionsPerInch = 1120/12.533;

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        //sets proper direction

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //resets encoder


        //target position to run to
        this.opMode.telemetry.addLine("set target");
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //runs to the position

        leftFront.setTargetPosition((int) (inches * positionsPerInch));
        leftBack.setTargetPosition((int) (inches * positionsPerInch));
        rightFront.setTargetPosition((int) (inches * positionsPerInch));
        rightBack.setTargetPosition((int) (inches * positionsPerInch));

        //ensures robot will go to the destination needed
        this.opMode.telemetry.addLine("run to");

        while (opMode.opModeIsActive()
                && leftFront.isBusy()
                && leftBack.isBusy()
                && rightFront.isBusy()
                && rightBack.isBusy()) {

            // motor power has exceeded the max power allowed
            if(motorTimer.milliseconds() > currentTime + timeStep && motorCurrentPower <= power) {

                motorCurrentPower = motorCurrentPower +  powerUpPerTimeStep;
                currentTime = currentTime + timeStep;

                leftFront.setPower(motorCurrentPower);
                leftBack.setPower(motorCurrentPower);
                rightFront.setPower(motorCurrentPower);
                rightBack.setPower(motorCurrentPower);
                this.opMode.telemetry.addLine("set power");
                //sets speed at which robot will run
            }

            this.opMode.telemetry.addData("Left Front Motor Current Position: ", leftFront.getCurrentPosition());
            this.opMode.telemetry.addData("Left Back Motor Current Position: ", leftBack.getCurrentPosition());
            this.opMode.telemetry.addData("Right Front Current Position: ", rightFront.getCurrentPosition());
            this.opMode.telemetry.addData("Right Back Motor Current Position: ", rightBack.getCurrentPosition());
            this.opMode.telemetry.update();
            //sends data on the positions of where the motor is located
        }
    }

    public void MecanumMoveInInchesWithCounter
            (DcMotor leftFront, DcMotor rightFront,
             DcMotor leftBack, DcMotor rightBack, double power, double inches)
    {
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        //sets proper direction

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //resets encoder

        double count = 2.5;

        double positionsPerInch = 1120/12.533;
        //positionsPerInch is the amount of counts the encoder makes to move 1 inch

        if ((double)(count/(positionsPerInch * inches))*power < power) {
            count = count + 10;

            //target position to run to
            this.opMode.telemetry.addLine("set target");
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //runs to the position

            leftFront.setTargetPosition((int) (inches * positionsPerInch));
            leftBack.setTargetPosition((int) (inches * positionsPerInch));
            rightFront.setTargetPosition((int) (inches * positionsPerInch));
            rightBack.setTargetPosition((int) (inches * positionsPerInch));

            //ensures robot will go to the destination needed
            this.opMode.telemetry.addLine("run to");
            leftFront.setPower(power * (count / (positionsPerInch * inches)));
            leftBack.setPower(power * (count / (positionsPerInch * inches)));
            rightFront.setPower(power * (count / (positionsPerInch * inches)));
            rightBack.setPower(power * (count / (positionsPerInch * inches)));
            this.opMode.telemetry.addLine("set power");
            //sets speed at which robot will run

            while (opMode.opModeIsActive()
                    && leftFront.isBusy()
                    && leftBack.isBusy()
                    && rightFront.isBusy()
                    && rightBack.isBusy()) {
                this.opMode.telemetry.addData("Left Front Motor Current Position: ", leftFront.getCurrentPosition());
                this.opMode.telemetry.addData("Left Back Motor Current Position: ", leftBack.getCurrentPosition());
                this.opMode.telemetry.addData("Right Front Current Position: ", rightFront.getCurrentPosition());
                this.opMode.telemetry.addData("Right Back Motor Current Position: ", rightBack.getCurrentPosition());
                this.opMode.telemetry.update();
                //sends data on the positions of where the motor is located
            }
        }
        else {
            //target position to run to
            this.opMode.telemetry.addLine("set target");
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //runs to the position

            leftFront.setTargetPosition((int) (inches * positionsPerInch));
            leftBack.setTargetPosition((int) (inches * positionsPerInch));
            rightFront.setTargetPosition((int) (inches * positionsPerInch));
            rightBack.setTargetPosition((int) (inches * positionsPerInch));

            //ensures robot will go to the destination needed
            this.opMode.telemetry.addLine("run to");
            leftFront.setPower(power);
            leftBack.setPower(power);
            rightFront.setPower(power);
            rightBack.setPower(power);
            this.opMode.telemetry.addLine("set power");
            //sets speed at which robot will run

            while (opMode.opModeIsActive()
                    && leftFront.isBusy()
                    && leftBack.isBusy()
                    && rightFront.isBusy()
                    && rightBack.isBusy()) {
                this.opMode.telemetry.addData("Left Front Motor Current Position: ", leftFront.getCurrentPosition());
                this.opMode.telemetry.addData("Left Back Motor Current Position: ", leftBack.getCurrentPosition());
                this.opMode.telemetry.addData("Right Front Current Position: ", rightFront.getCurrentPosition());
                this.opMode.telemetry.addData("Right Back Motor Current Position: ", rightBack.getCurrentPosition());
                this.opMode.telemetry.update();
                //sends data on the positions of where the motor is located
            }
        }
    }

    public void MecanumTurningInDegrees(DcMotor leftFront, DcMotor rightFront,
                                        DcMotor leftBack, DcMotor rightBack,
                                        double power, double degrees){


        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double accumulatedTurn = 0;
        double startAngle = this.getAbsoluteHeading();

        while(this.opMode.opModeIsActive() && Math.abs(accumulatedTurn) <= Math.abs(degrees)) {

            accumulatedTurn = this.getAbsoluteHeading() - startAngle; // calculate the turn amount

            // normalize the turned amount within -180 to +180
            if (accumulatedTurn < -180)
                accumulatedTurn += 360;
            else if (accumulatedTurn > 180)
                accumulatedTurn -= 360;

            if (degrees < 0) {
                // turn left
                leftFront.setPower(-0.3);
                leftBack.setPower(-0.3);
                rightFront.setPower(0.3);
                rightBack.setPower(0.3);
            } else {
                // turn right
                leftFront.setPower(0.3);
                leftBack.setPower(0.3);
                rightFront.setPower(-0.3);
                rightBack.setPower(-0.3);
            }
        }
    }

    private double getAbsoluteHeading()
    {
        return this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    private void ConfigureToMoveForward(DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack){
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

//        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }




    // **************************** END OF MOVE, SLIDE & TURN ROBOT ***********************************************************


    // **************************** START OF SKYSTONE DETECTION ***********************************************************
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private static final String VUFORIA_KEY =
            "AR/ZUHH/////AAABmUqm26qkS0kJtcAm07xRqe5jkIrpagCp8Mt6fJQLNN3uG4F5Qn6UIwRnhbinYkn+S+rbdFMcS0aEcORq5kSi5hNxMxGq7YB3V2f8pDhtPJFb5DDLwzrhKDIwGI8CST3T+JhN6mQhsHnMI45xtjMASIKs6v2b0ZpYh2YvzNY8ZgDDK4jVSZ9wg7jGlIlOVnUINeMtSoUnrXRCqqQ6OHZSNVkPjcP7pPitJuXgPltX0uz+b90EWWOsxzW4K2R2+3FE5WtUr6/8MOgUHDc+64BqVJe7ird88ctEJ/W0E5rRspZ6BRre1N9/4x19XZDyLJWKIcMiAtepqz8T7F55GbEyPNXTet3KYAbPeCR+Sj7YuOoE";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;


    //find location of skystone
    public enum SkystoneLocation{
        NONE,
        LEFT,
        CENTER,
        RIGHT,
        FORWARD,
        BACKWARD
    }
//    public SkystoneVuforia.SkystoneLocation FindSkystone(){
//        SkystoneVuforia.SkystoneLocation skystoneLocation = SkystoneVuforia.SkystoneLocation.NONE;
//        initVuforia();
//
//        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
//            initTfod();
//        } else {
//            this.opMode.telemetry.addData("Sorry!", "This device is not compatible with TFOD");
//        }
//
//        /**
//         * Activate TensorFlow Object Detection before we wait for the start command.
//         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
//         **/
//        if (tfod != null) {
//            tfod.activate();
//        }
//
//        this.opMode.telemetry.addData(">", "Press Play to start op mode");
//        this.opMode.telemetry.update();
//        this.opMode.waitForStart();
//
//        if (this.opMode.opModeIsActive()) {
//            while (this.opMode.opModeIsActive() && skystoneLocation == SkystoneVuforia.SkystoneLocation.NONE) {
//                if (tfod != null) {
//                    // getUpdatedRecognitions() will return null if no new information is available since
//                    // the last time that call was made.
//                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
//                    if (updatedRecognitions != null) {
//                        this.opMode.telemetry.addData("# Object Detected", updatedRecognitions.size());
//                        // step through the list of recognitions and display boundary info.
//                        int i = 0;
//                        List<Recognition> skystone = new ArrayList<Recognition>();
//                        List<Recognition> stone = new ArrayList<Recognition>();
//                        float objectHeight = 0;
//                        float imageHeight = 0;
//                        float ratioHeight = 0;
//                        double horizontalAngle = 0;
//                        int skystoneUsed = 0;
//                        for (Recognition recognition : updatedRecognitions) {
//                            this.opMode.telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
//                            this.opMode.telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
//                                    recognition.getLeft(), recognition.getTop());
//                            this.opMode.telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
//                                    recognition.getRight(), recognition.getBottom());
//
//                            objectHeight = recognition.getHeight();
//                            imageHeight = recognition.getImageHeight();
//                            ratioHeight = objectHeight/imageHeight;
//                            horizontalAngle = recognition.estimateAngleToObject(AngleUnit.DEGREES);
//
//                            this.opMode.telemetry.addData("Estimated Angle", "%.3f", horizontalAngle);
//                            this.opMode.telemetry.addData("Height Ratio", "%.3f", ratioHeight);
//                            this.opMode.telemetry.addData("Object Height", "%.3f", objectHeight);
//                            this.opMode.telemetry.addData("Image Height", "%.3f", imageHeight);
//                            if (recognition.getLabel().equals("Skystone")){
//                                //Add to Skystone List
//                                skystone.add(recognition);
//                            }else {
//                                //Add to Stone List
//                                stone.add(recognition);
//                            }
//
//                            i++;
//                        }
//
//                        //Check if Skystone list is empty
//                        if(skystone.size() > 0){
//                            //Recognizes Skystone
//                            this.opMode.telemetry.addData("Skystone Detected: ", skystone.size());
//                            if(skystone.size() == 1){
//                                skystoneUsed = 0;
//                            }else{
//                                if(skystone.get(0).estimateAngleToObject(AngleUnit.DEGREES) > skystone.get(1).estimateAngleToObject(AngleUnit.DEGREES)){
//                                    skystoneUsed = 0;
//                                }else{
//                                    skystoneUsed = 1;
//                                }
//                            }
//                            if(skystone.get(skystoneUsed).estimateAngleToObject(AngleUnit.DEGREES) < -5){
//                                //move left
//                                skystoneLocation = SkystoneVuforia.SkystoneLocation.LEFT;
//                            } else if(skystone.get(skystoneUsed).estimateAngleToObject(AngleUnit.DEGREES) > 5){
//                                //move right
//                                skystoneLocation = skystoneLocation.RIGHT;
//                            } else {
//                                //skystoneLocation = skystoneLocation.CENTERX;
//                                if (skystone.get(skystoneUsed).getHeight()/skystone.get(skystoneUsed).getImageHeight() < 0.45){
//                                    //move forwards
//                                    skystoneLocation = SkystoneVuforia.SkystoneLocation.FORWARD;
//                                }else if (skystone.get(skystoneUsed).getHeight()/skystone.get(skystoneUsed).getImageHeight() > 0.55){
//                                    //move backwards
//                                    skystoneLocation = SkystoneVuforia.SkystoneLocation.BACKWARD;
//                                }else{
//                                    skystoneLocation = SkystoneVuforia.SkystoneLocation.CENTER;
//                                }
//                            }
//                        }else if(stone.size() > 0){
//                            //No Skystones, Move to stone
//                            this.opMode.telemetry.addData("No Skystones Detected, Stones Detected: ", stone.size());
//
//                        }else{
//                            //Nothing Detected
//                            this.opMode.telemetry.addData("Nothing Detected", 0);
//                        }
//
//                        this.opMode.telemetry.update();
//                    }
//                }
//            }
//        }
//
//        if (tfod != null) {
//            tfod.shutdown();
//        }
//        return skystoneLocation;
//    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = this.opMode.hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = this.opMode.hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", this.opMode.hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.7;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}