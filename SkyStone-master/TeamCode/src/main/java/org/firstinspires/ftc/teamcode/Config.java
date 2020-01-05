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
    Servo winch1 = null;
    Servo winch2 = null;
    //claw
    Servo orient = null;
    Servo claw = null;

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

        winch1= hwMap.get(Servo.class, "rwinch");//hub 1
        winch2= hwMap.get(Servo.class, "lwinch");//hub 1 - port 4
        orient = hwMap.get(Servo.class, "orient");//hub 1 - port 0
        claw = hwMap.get(Servo.class, "claw");//hub 1 - port 5

        leftPull = hwMap.get(Servo.class, "left_pull");//hub 2 - port 0
        rightPull = hwMap.get(Servo.class, "right_pull");//hub 2 - port 1



        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        leftPull.setPosition(1.0);
        rightPull.setPosition(0.0);


        // May want to use RUN_USING_ENCODERS if encoders are installed.
    }

}
