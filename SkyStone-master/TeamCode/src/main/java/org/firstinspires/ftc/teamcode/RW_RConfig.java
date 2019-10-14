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

public class RW_RConfig{
    DcMotor leftFront = null;
    DcMotor rightFront = null;
    DcMotor leftBack = null;
    DcMotor rightBack = null;

    DcMotor chainArm = null;
    DcMotor chainIntake = null;
    DcMotor lslideMotorup = null;
    DcMotor leadScrewMotor = null;

    DigitalChannel topMagneticSensor = null;
    DigitalChannel bottomMagneticSensor = null;

    ColorSensor leftColorSensor = null;
    ColorSensor rightColorSensor = null;


    //armMotor controls the movement of the arm
//    Servo rightServo = null;
//    Servo leftServo = null;
//    Servo gateServo = null;
    Servo markerServo = null;
    Servo winchServo = null;
    Servo dropServo = null;

    BNO055IMU imu;
    //local opMode Members are created
    HardwareMap hwMap = null;
    //hwMap contains all the information

    //constructor
    public RW_RConfig()
    {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
//        imu = hwMap.get(BNO055IMU.class, "imu");

        leftFront = hwMap.get(DcMotor.class, "left_front");//chassis hub - port 0
        rightFront = hwMap.get(DcMotor.class, "right_front");//chassis hub  - port 1
        leftBack = hwMap.get(DcMotor.class, "left_back");//chassis hub - port 2
        rightBack = hwMap.get(DcMotor.class, "right_back");//chassis hub - port 3
//
//        leftServo = hwMap.get(Servo.class, "left_servo");//chassis hub - port 0
//        rightServo = hwMap.get(Servo.class, "right_servo");//chassis hub - port 1
//        gateServo = hwMap.get(Servo.class, "gate_servo");//chassis hub - port 2
        markerServo = hwMap.get(Servo.class, "marker_servo");//hub 2 - port 0
        winchServo = hwMap.get(Servo.class, "winch_servo"); //hub2 - port1
        dropServo = hwMap.get(Servo.class, "drop_servo"); //hub2 - port2
//
//        rightColorSensor = hwMap.get(ColorSensor.class, "right_color"); //chassis hub - bus 2
//        leftColorSensor = hwMap.get(ColorSensor.class, "left_color"); //chassis hub - bus 3

        chainArm = hwMap.get(DcMotor.class, "chain_arm");//hub 2 - port 0
        chainIntake = hwMap.get(DcMotor.class, "intake_motor");//hub 2 - port 1
        lslideMotorup = hwMap.get(DcMotor.class, "linear_slide_up");//hub 2 - port 2
        leadScrewMotor = hwMap.get(DcMotor.class, "lead_screw");//hub 2 - port 3

        topMagneticSensor = hwMap.get(DigitalChannel.class, "top_mag_sensor");//hub 2 - port 1
        bottomMagneticSensor = hwMap.get(DigitalChannel.class, "bottom_mag_sensor");//hub 2 - port 3


        //set direction
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        lslideMotorup.setPower(0);
        chainIntake.setPower(0);
        chainArm.setPower(0);
        rightBack.setPower(0);
        leadScrewMotor.setPower(0);

        markerServo.setPosition(1);
        dropServo.setPosition(0);
        winchServo.setPosition(0);

        // May want to use RUN_USING_ENCODERS if encoders are installed.
    }

}
