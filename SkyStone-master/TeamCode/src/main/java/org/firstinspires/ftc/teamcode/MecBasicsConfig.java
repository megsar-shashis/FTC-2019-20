package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by meghn on 10/12/2018.
 */

public class MecBasicsConfig {
    //chassis
    DcMotor leftFront = null;
    DcMotor rightFront = null;
    DcMotor leftBack = null;
    DcMotor rightBack = null;

    BNO055IMU imu;
    //local opMode Members are created
    HardwareMap hwMap = null;
    //hwMap contains all the information

    //constructor
    public MecBasicsConfig()
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

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);


        // May want to use RUN_USING_ENCODERS if encoders are installed.
    }

}
