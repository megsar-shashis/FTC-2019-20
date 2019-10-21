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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

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


    // **************************** START OF LATCHING & LANDING ROBOT ***********************************************************
    public boolean GetTouchSensorState (DigitalChannel touchSensor) {
        return touchSensor.getState();
    }

    public void MoveLeadScrew(DcMotor lScrewMotor, DigitalChannel touchSensor, double power) {
        while (this.opMode.opModeIsActive() && GetTouchSensorState(touchSensor)) {
            this.opMode.telemetry.addData("position", lScrewMotor.getCurrentPosition());
            this.opMode.telemetry.update();
            lScrewMotor.setPower(power);
        }
        lScrewMotor.setPower(0);
    }

    public void oneMotorArmEncoder(int inches, double power, DcMotor liftMotor)
    {
        liftMotor.setDirection(DcMotor.Direction.FORWARD);

        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double postionsPerInch = 1120/(3.14*4);
        int positionsToMove = (int)(postionsPerInch * inches);
        liftMotor.setTargetPosition(positionsToMove);

        liftMotor.setPower(power);

        while(this.opMode.opModeIsActive() && liftMotor.isBusy()) {
            this.opMode.telemetry.addData("leftMotor", liftMotor.getCurrentPosition());
            this.opMode.telemetry.update();
        }
        //liftMotor.setPower(0);
    }


    public  enum ScrewDirection{
        Extend, Contract
    }
    //This is a function that lands the robot in the autonomous mode.
    // Input Parameters are top magnetic sensor, bottom magnetic sensor, lead screw motor, and the power at which the lead screw will spin

    public void MoveLeadScrewWithMagnets(DigitalChannel _topSensor, DigitalChannel _bottomSensor,
                                         DcMotor _leadScrewMotor, ScrewDirection direction) {
        double motorPower = 0;
        if (direction == ScrewDirection.Extend) {
            motorPower = 1;
        } else if (direction == ScrewDirection.Contract) {
            motorPower = -1;
        }
        while (this.opMode.opModeIsActive()) {
            if (_topSensor.getState() == false) { // top sensor detects a magnet
                if (motorPower > 0) { // positive power extends the lead screw
                    // stop, do nothing
                    this.opMode.telemetry.addData("Top sensor detected magnet.", "upwards motion stopped");
                    this.opMode.telemetry.update();
                    motorPower = 0;
                    _leadScrewMotor.setPower(motorPower);
                    // you could put a return statement here to exit out of the function when top magnet detects sensor.
                    // Uncomment the following to do so
                    return;
                } else {
                    // top sensor detects magnet but the assembly is moving down, so it is ok
                    _leadScrewMotor.setPower(motorPower);
                }
            } else if (_bottomSensor.getState() == false) { // bottom sensor detects a magnet
                if (motorPower < 0) {
                    // stop do nothing unless power
                    this.opMode.telemetry.addData("Bottom sensor detected magnet", "downwards motion stopped");
                    this.opMode.telemetry.update();
                    motorPower = 0;
                    _leadScrewMotor.setPower(motorPower);
                    // you could put a return statement here to exit out of the function.
                    // Uncomment the following to do so/
                    return;
                } else {
                    // bottom sensor detects magnet but the assembly is moving up so it is ok
                    _leadScrewMotor.setPower(motorPower);
                }
            } else {
                // neither the top, nor the bottom magnetic sensor has detected a magnet
                _leadScrewMotor.setPower(motorPower);
            }
        }
    }

    // position is 14800
    //public boolean MoveLeadScrew(DcMotor _leadScrewMotor, ScrewDirection direction, int position) {
    public void MoveLeadScrew(DcMotor _leadScrewMotor, ScrewDirection direction, int position) {
        double motorPower = 0;
        if (direction == ScrewDirection.Extend) {
            motorPower = 1;
        } else if (direction == ScrewDirection.Contract) {
            motorPower = -1;
        }
        _leadScrewMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _leadScrewMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _leadScrewMotor.setTargetPosition(position);
        _leadScrewMotor.setPower(motorPower);
        while (this.opMode.opModeIsActive() &&_leadScrewMotor.getCurrentPosition() <= position) {

            // you could put a return statement here to exit out of the function when top magnet detects sensor.
            // Uncomment the following to do so

        }
        _leadScrewMotor.setPower(0);
        //return true;
    }

    // **************************** END OF LATCHING & LANDING ROBOT *********************************************************



    // **************************** START OF ALIGNING OF ROBOT WITH COLOR SENSOR ********************************************

    // **************************** END OF ALIGNING OF ROBOT WITH COLOR SENSOR ********************************************

    //*****************************         START OF SERVO FUNCTIONS           *******************************************
    public void dropTeamMarker (Servo markerServo, double position) {
        markerServo.setPosition(position);
    }
}
