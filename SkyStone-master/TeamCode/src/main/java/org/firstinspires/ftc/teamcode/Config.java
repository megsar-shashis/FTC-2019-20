package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by meghn on 10/12/2018.
 */

public class Config{
    //chassis
    DcMotor leftFront = null;
    DcMotor rightFront = null;
    DcMotor leftBack = null;
    DcMotor rightBack = null;

    //arm
    Servo VSWinch = null;
    Servo HSWinch = null;
    Servo VHElbow = null;
    DcMotor Arm = null;
    //claw
    Servo orient = null;
    Servo leftClaw = null;
    Servo rightClaw = null;

    //pull
    Servo leftPull = null;
    Servo rightPull = null;


    BNO055IMU imu;
    //local opMode Members are created
    HardwareMap hwMap = null;
    //hwMap contains all the information

    //constructor
    public Config()
    {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
//        imu = hwMap.get(BNO055IMU.class, "imu");

        leftFront = hwMap.get(DcMotor.class, "left_front");//hub2 - port 0
        rightFront = hwMap.get(DcMotor.class, "right_front");//hub2  - port 1
        leftBack = hwMap.get(DcMotor.class, "left_back");//hub2 - port 2
        rightBack = hwMap.get(DcMotor.class, "right_back");//hub2 - port 3

        VSWinch= hwMap.get(Servo.class, "vs_winch");//hub 1 - port 0
        HSWinch = hwMap.get(Servo.class, "hs_winch"); //hub 1 - port1
        VHElbow = hwMap.get(Servo.class, "vh_elbow"); //hub 1 - port2
        Arm = hwMap.get(DcMotor.class, "arm");//hub 1 - port 0

        orient = hwMap.get(Servo.class, "orient");//hub 1 - port 3
        leftClaw = hwMap.get(Servo.class, "left_claw");//hub 1 - port 4
        rightClaw = hwMap.get(Servo.class, "right_claw");//hub 1 - port 5

        leftPull = hwMap.get(Servo.class, "left_pull");//hub 2 - port 0
        rightPull = hwMap.get(Servo.class, "right_pull");//hub 2 - port 1


        //set direction
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        Arm.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        VSWinch.setPosition(1);
        HSWinch.setPosition(0);
        VHElbow.setPosition(0);
        Arm.setPower(0);

        orient.setPosition(1);
        leftClaw.setPosition(0);
        rightClaw.setPosition(0);

        leftPull.setPosition(-0.5);
        rightPull.setPosition(0.5);


        // May want to use RUN_USING_ENCODERS if encoders are installed.
    }

}
