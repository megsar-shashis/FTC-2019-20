package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by meghna on 12/08/2019
 */


public class MecanumFunctions {

   private LinearOpMode opMode;
   private static final double PositionsPerInch = 1120/12.533;

    // **************************** START OF MOVE, SLIDE & TURN ROBOT ******************************************************
    public MecanumFunctions(LinearOpMode _opMode) {
        opMode = _opMode;
    }

    public MecanumFunctions()
    {
    }

    public enum MoveAction {
        MoveForward,
        MoveBackward,
        SlideLeft,
        SlideRight,
        TurnLeft,
        TurnRight
    }


    public void MoveRobot(Config robot, MoveAction moveAction, double inches, double power) {
        switch (moveAction) {

            case MoveForward:
                robot.leftFront.setDirection(DcMotor.Direction.REVERSE);
                robot.leftBack.setDirection(DcMotor.Direction.REVERSE);
                robot.rightFront.setDirection(DcMotor.Direction.FORWARD);
                robot.rightBack.setDirection(DcMotor.Direction.FORWARD);
                break;
            case MoveBackward:
                robot.leftFront.setDirection(DcMotor.Direction.FORWARD);
                robot.leftBack.setDirection(DcMotor.Direction.FORWARD);
                robot.rightFront.setDirection(DcMotor.Direction.REVERSE);
                robot.rightBack.setDirection(DcMotor.Direction.REVERSE);
                break;
            case SlideRight:
                robot.leftFront.setDirection(DcMotor.Direction.REVERSE);
                robot.leftBack.setDirection(DcMotor.Direction.FORWARD);
                robot.rightFront.setDirection(DcMotor.Direction.REVERSE);
                robot.rightBack.setDirection(DcMotor.Direction.FORWARD);
                break;
            case SlideLeft:
                robot.leftFront.setDirection(DcMotor.Direction.FORWARD);
                robot.leftBack.setDirection(DcMotor.Direction.REVERSE);
                robot.rightFront.setDirection(DcMotor.Direction.FORWARD);
                robot.rightBack.setDirection(DcMotor.Direction.REVERSE);
                break;
            case TurnLeft:
                robot.leftFront.setDirection(DcMotor.Direction.FORWARD);
                robot.leftBack.setDirection(DcMotor.Direction.FORWARD);
                robot.rightFront.setDirection(DcMotor.Direction.FORWARD);
                robot.rightBack.setDirection(DcMotor.Direction.FORWARD);
                break;
            case TurnRight:
                robot.leftFront.setDirection(DcMotor.Direction.REVERSE);
                robot.leftBack.setDirection(DcMotor.Direction.REVERSE);
                robot.rightFront.setDirection(DcMotor.Direction.REVERSE);
                robot.rightBack.setDirection(DcMotor.Direction.REVERSE);
                break;
        }

        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftFront.setTargetPosition((int) (inches * PositionsPerInch));
        robot.leftBack.setTargetPosition((int) (inches * PositionsPerInch));
        robot.rightFront.setTargetPosition((int) (inches * PositionsPerInch));
        robot.rightBack.setTargetPosition((int) (inches * PositionsPerInch));

        //ensures robot will go to the destination needed
        this.opMode.telemetry.addLine("run to");
        robot.leftFront.setPower(power); //change values
        robot.rightFront.setPower(power);
        robot.leftBack.setPower(power);
        robot.rightBack.setPower(power);
        this.opMode.telemetry.addLine("set power");
        this.opMode.telemetry.update();
        //sets speed at which robot will run

        while (this.opMode.opModeIsActive()
                && robot.leftFront.isBusy()
                && robot.leftBack.isBusy()
                && robot.rightFront.isBusy()
                && robot.rightBack.isBusy()) {
            this.opMode.telemetry.addData("Left Front Motor Current Position: ", robot.leftFront.getCurrentPosition());
            this.opMode.telemetry.addData("Left Back Motor Current Position: ", robot.leftBack.getCurrentPosition());
            this.opMode.telemetry.addData("Right Front Current Position: ", robot.rightFront.getCurrentPosition());
            this.opMode.telemetry.addData("Right Back Motor Current Position: ", robot.rightBack.getCurrentPosition());
            this.opMode.telemetry.update();
        }
    }
}