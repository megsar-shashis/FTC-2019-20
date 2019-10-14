package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.hardware.Sensor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.List;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by meghn on 10/1/2018.
 */

public class MecanumBasedUtils {

    //CONSTANTS
    int minGoldHueRange = 31;
    int maxGoldHueRange = 41;
    int minRedHueLRange = 0;
    int maxRedHueLRange = 20;
    int minRedHueURange = 344;
    int maxRedHueURange = 360;
    int minBlueHueRange = 170;
    int maxBlueHueRange = 270;

    public enum GoldMineralLocation
    {
        NONE,
        RIGHT,
        CENTER,
        LEFT
    }

    private LinearOpMode opMode;
    BNO055IMU imu;
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = "AR/ZUHH/////AAABmUqm26qkS0kJtcAm07xRqe5jkIrpagCp8Mt6fJQLNN3uG4F5Qn6UIwRnhbinYkn+S+rbdFMcS0aEcORq5kSi5hNxMxGq7YB3V2f8pDhtPJFb5DDLwzrhKDIwGI8CST3T+JhN6mQhsHnMI45xtjMASIKs6v2b0ZpYh2YvzNY8ZgDDK4jVSZ9wg7jGlIlOVnUINeMtSoUnrXRCqqQ6OHZSNVkPjcP7pPitJuXgPltX0uz+b90EWWOsxzW4K2R2+3FE5WtUr6/8MOgUHDc+64BqVJe7ird88ctEJ/W0E5rRspZ6BRre1N9/4x19XZDyLJWKIcMiAtepqz8T7F55GbEyPNXTet3KYAbPeCR+Sj7YuOoE";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    public MecanumBasedUtils(LinearOpMode _opMode) {
        opMode = _opMode;
        imu = this.opMode.hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        this.imu.initialize(parameters);

        while(!imu.isGyroCalibrated()) {
            this.opMode.sleep(50);
            //this.logTelemetry("Calibrating now", "");
        }
    }

    public MecanumBasedUtils()
    {
    }

    //*************************************** START OF SAMPLING MINERALS **********************************************
    public GoldMineralLocation GetGoldLocWithTensorFlowTwoObjects()
    {

        GoldMineralLocation goldMineralLocation = GoldMineralLocation.NONE;
        ElapsedTime functionTimer = new ElapsedTime();

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            this.opMode.telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        if (this.opMode.opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }
            //TODO: this code is ASSUMING that phone is focused on the rightmost minerals
            // create timer - 5 seconds or so
            functionTimer.reset();
            while (this.opMode.opModeIsActive() && goldMineralLocation == GoldMineralLocation.NONE && functionTimer.seconds() < 6) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        this.opMode.telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 2) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                    //silverMineral1X = (int) recognition.getLeft();
                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getLeft();
                                    //silverMineral2X = (int) recognition.getLeft();
                                } else {
                                    silverMineral2X = (int) recognition.getLeft();
                                }
                            }
                            this.opMode.telemetry.addData("Position", "After the loop");
                            this.opMode.telemetry.addData("goldmineralx", goldMineralX);
                            this.opMode.telemetry.addData("silverMineral1X", silverMineral1X);
                            this.opMode.telemetry.addData("silverMineral2X", silverMineral2X);
//                            if (goldMineralX != -1 && silverMineral1X != -1) {
//                                // if gold left is left of silver1x, gold is center
//                                if(goldMineralX < silverMineral1X){
//                                    this.opMode.telemetry.addData("Gold Mineral Position", "Center");
//                                    goldMineralLocation = GoldMineralLocation.CENTER;
//                                }
//                                // else gold is right
//                                else {
//                                    this.opMode.telemetry.addData("Gold Mineral Position", "Right");
//                                    goldMineralLocation = GoldMineralLocation.RIGHT;
//                                }
//                            } else if(goldMineralX != -1  && silverMineral2X != -1) {
//                                // if gold left is left of silver1x, gold is center
//                                if(goldMineralX < silverMineral1X){
//                                    this.opMode.telemetry.addData("Gold Mineral Position", "Center");
//                                    goldMineralLocation = GoldMineralLocation.CENTER;
//                                }
//                                // else gold is right
//                                else {
//                                    this.opMode.telemetry.addData("Gold Mineral Position", "Right");
//                                    goldMineralLocation = GoldMineralLocation.RIGHT;
//                                }
//                            } else {
//                                //Case 3: Silver and Silver
//                                //gold is left
//                                this.opMode.telemetry.addData("Gold Mineral Position", "Left");
//                                goldMineralLocation = GoldMineralLocation.LEFT;
//                            }

                            if (goldMineralX != -1 && silverMineral1X != -1) {
                                // if gold left is left of silver1x, gold is center
                                if(goldMineralX < silverMineral1X){
                                    this.opMode.telemetry.addData("Gold Mineral Position", "Center");
                                    goldMineralLocation = GoldMineralLocation.CENTER;
                                    break;
                                }
                                // else gold is right
                                else {
                                    this.opMode.telemetry.addData("Gold Mineral Position", "Right");
                                    goldMineralLocation = GoldMineralLocation.RIGHT;
                                    break;
                                }
                            } else if(silverMineral1X != -1  && silverMineral2X != -1) {
                                this.opMode.telemetry.addData("Gold Mineral Position", "Left");
                                goldMineralLocation = GoldMineralLocation.LEFT;
                                break;
                            }
                        }
                        this.opMode.telemetry.addData("Gold Mineral Position", goldMineralLocation);
                        this.opMode.telemetry.update();
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }

        this.opMode.telemetry.addData("Gold Mineral Position: ", goldMineralLocation);
        this.opMode.telemetry.update();

        return goldMineralLocation;
    }



    public void SampleMineralsWithTensorFlow(DcMotor leftFront, DcMotor rightFront,
                                             DcMotor leftBack, DcMotor rightBack,
                                             double slidePower, int distanceToSlide, double movePower, int distanceToMoveForward)
    {
        GoldMineralLocation goldMineralLocation = GoldMineralLocation.NONE;
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            this.opMode.telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        if (this.opMode.opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            while (this.opMode.opModeIsActive() && goldMineralLocation == GoldMineralLocation.NONE) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        this.opMode.telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 3) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getLeft();
                                } else {
                                    silverMineral2X = (int) recognition.getLeft();
                                }
                            }
                            this.opMode.telemetry.addData("Position", "After the loop");
                            this.opMode.telemetry.addData("goldmineralx", goldMineralX);
                            this.opMode.telemetry.addData("silverMineral1X", silverMineral1X);
                            this.opMode.telemetry.addData("silverMineral2X", silverMineral2X);
                            if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                                if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                    this.opMode.telemetry.addData("Gold Mineral Position", "Left");
                                    goldMineralLocation = GoldMineralLocation.LEFT;
                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                    this.opMode.telemetry.addData("Gold Mineral Position", "Right");
                                    goldMineralLocation = GoldMineralLocation.RIGHT;
                                } else {
                                    this.opMode.telemetry.addData("Gold Mineral Position", "Center");
                                    goldMineralLocation = GoldMineralLocation.CENTER;
                                }
                            }
                        }
                        this.opMode.telemetry.update();
                    }
                }
            }
        }

        switch (goldMineralLocation)
        {
            case RIGHT:
            {
                this.MecanumSlideRightInInches(leftFront, rightFront, leftBack, rightBack, slidePower, distanceToSlide);
            }
            break;
            case LEFT:
            {
                this.MecanumSlideLeftInInches(leftFront, rightFront, leftBack, rightBack, slidePower, distanceToSlide);
            }
            break;
            case CENTER:
                //no sliding
                break;
        }

        this.MecanumMoveForwardInInches(leftFront, rightFront, leftBack, rightBack, movePower, distanceToMoveForward);

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = this.opMode.hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", this.opMode.hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }


//    public void DisplayHSVValues(ColorSensor sensorColor)
//    {
//
//        float hsvValues[] = {0F, 0F, 0F};
//
//        // // values is a reference to the hsvValues array.
//        final float values[] = hsvValues;
//
//        // // sometimes it helps to multiply the raw RGB values with a scale factor
//        // // to amplify/attentuate the measured values.
//        final double SCALE_FACTOR = 255;
//
//        android.graphics.Color.RGBToHSV((int)(sensorColor.red() * SCALE_FACTOR),
//                (int)(sensorColor.green() * SCALE_FACTOR),
//                (int)(sensorColor.blue() * SCALE_FACTOR), hsvValues);
//        this.opMode.telemetry.addData("HSV_H", "Hue=" + hsvValues[0]);
//        this.opMode.telemetry.addData("HSV_H", "Saturation=" + hsvValues[1]);
//        this.opMode.telemetry.addData("HSV_H", "Value=" + hsvValues[2]);
//        this.opMode.telemetry.addData("RGB", "Red=" + sensorColor.red());
//        this.opMode.telemetry.addData("RGB", "Green=" + sensorColor.green());
//        this.opMode.telemetry.addData("RGB", "Blue=" + sensorColor.blue());
//        this.opMode.telemetry.update();
//    }

    public int getHueValue(ColorSensor sensorColor)
    {
        float hsvValues[] = {0F, 0F, 0F};

        // // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // // sometimes it helps to multiply the raw RGB values with a scale factor
        // // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        android.graphics.Color.RGBToHSV((int)(sensorColor.red() * SCALE_FACTOR),
                (int)(sensorColor.green() * SCALE_FACTOR),
                (int)(sensorColor.blue() * SCALE_FACTOR), hsvValues);
        this.opMode.telemetry.addData("GetHue_HSV_H", "Hue=" + hsvValues[0]);
        this.opMode.telemetry.addData("GetHue_HSV_S", "Saturation=" + hsvValues[1]);
        this.opMode.telemetry.addData("GetHue_HSV_V", "Value=" + hsvValues[2]);

        this.opMode.telemetry.update();
        return (int)hsvValues[0];
    }

    //Check if the hue value is within the range to detect the gold mineral
    private boolean isHueGold(int frontHue, int backHue)
    {
        return ((frontHue >= minGoldHueRange && frontHue <= maxGoldHueRange) || (backHue >= minGoldHueRange && backHue <= maxGoldHueRange));
    }

    // **************************** END OF SAMPLING MINERALS ***********************************************************


    // **************************** START OF PICK & DROP MINERALS ***********************************************************

    public void openLeftDoor(Servo doorServo)
    {
        doorServo.setPosition(1);
    }

    //Closes the left door of the scooper
    public void closeLeftDoor(Servo doorServo)
    {
        doorServo.setPosition(0.5);
    }

    //Opens the right door of the scooper
    public void openRightDoor(Servo doorServo)
    {
        doorServo.setPosition(0);
    }

    //Closes the left door of the scooper
    public void closeRightDoor(Servo doorServo)
    {
        doorServo.setPosition(0.5);
    }

    //Opens the gate of the scooper
    public void openGate(Servo gateServo)
    {
        gateServo.setPosition(0.5);
    }

    //Closes the gate of the scooper
    public void closeGate(Servo gateServo)
    {
        gateServo.setPosition(0);
    }


    public double setPoint = 0, minInput = -100, maxInput = 2240;
    public double thresholdPercent = 0;
    private double currentError = 0;
    private double Kp;


    public boolean hasPControllerReachedTarget() {
        double percentDifferenceFromTarget =
                (currentError /(maxInput - minInput))*100;
        if(percentDifferenceFromTarget < thresholdPercent) {
            return true;
        }
        return false;
    }

    public void limitLinearSlide(DcMotor lSlideMotor){
        while(lSlideMotor.isBusy()){
            currentError = lSlideMotor.getCurrentPosition();
            if(hasPControllerReachedTarget() == true)
            {
                //then stop motor
                lSlideMotor.setPower(0);
            }
        }

    }
    // **************************** END OF PICK & DROP MINERALS ***********************************************************

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

    public void MecanumMoveForwardUntilColorDetected
            (DcMotor leftFront, DcMotor rightFront,
             DcMotor leftBack, DcMotor rightBack, double power, ColorSensor leftColorSensor, ColorSensor rightColorSensor)
    {
        //positive slides right, negative slides left

        this.opMode.telemetry.addLine("Inside the MecanumMoveForwardUntilColorDetected");


        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int counter = 0;
        while (this.opMode.opModeIsActive() && counter != 3){
            this.opMode.telemetry.addLine("Inside while loop");
            ColorRanges.ColorFromHue hueLeft = ColorRanges.GetColor(leftColorSensor.red(), leftColorSensor.green(), leftColorSensor.blue());
            ColorRanges.ColorFromHue hueRight = ColorRanges.GetColor(rightColorSensor.red(), rightColorSensor.green(), rightColorSensor.blue());
            while (this.opMode.opModeIsActive() &&
                    (hueLeft != ColorRanges.ColorFromHue.BLUE || hueRight != ColorRanges.ColorFromHue.BLUE)){
                leftFront.setPower(0.1);
                leftBack.setPower(0.1);
                rightFront.setPower(0.1);
                rightBack.setPower(0.1);
            }

            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);

            if (hueLeft == ColorRanges.ColorFromHue.BLUE ) {
                rightFront.setPower(0.1);
                rightBack.setPower(0.1);

            } else {
                // At this point hueColorRight is BLUE
                leftFront.setPower(0.1);
                leftBack.setPower(0.1);
            }

            if(counter<2) {
                MecanumMoveBackwardInInches(leftFront, rightFront, leftBack, rightBack, 0.1, 1);
            }
            // move back the robot by giving negative power to both wheels
            counter++;
        }

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
    public void AlignRobotWithColorSensor(DcMotor leftFront, DcMotor rightFront,
                                          DcMotor leftBack, DcMotor rightBack,
                                          ColorSensor leftColorSensor, ColorSensor rightColorSensor,
                                          double distanceToMoveInInches, double movePower, double turnPower)
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

        leftFront.setTargetPosition((int)(distanceToMoveInInches*positionsPerInch));
        rightFront.setTargetPosition((int)(distanceToMoveInInches*positionsPerInch));
        leftBack.setTargetPosition((int)(distanceToMoveInInches*positionsPerInch));
        rightBack.setTargetPosition((int)(distanceToMoveInInches*positionsPerInch));

        //ensures robot will go to the destination needed
        this.opMode.telemetry.addLine("run to");
        leftFront.setPower(movePower);
        rightFront.setPower(movePower);
        leftBack.setPower(movePower);
        rightBack.setPower(movePower);
        this.opMode.telemetry.addLine("set power");
        //sets speed at which robot will run, 'power' is input

        while(this.opMode.opModeIsActive()
                && leftFront.isBusy()
                && rightFront.isBusy()
                && leftBack.isBusy()
                && rightBack.isBusy()
                && !this.isHueBlue(this.getHueValue(leftColorSensor))
                && !this.isHueBlue(this.getHueValue(rightColorSensor))
                && !this.isHueRed(this.getHueValue(leftColorSensor))
                && !this.isHueRed(this.getHueValue(rightColorSensor)))
        {
            this.opMode.telemetry.addData("Left Front Motor Current Position: ", leftFront.getCurrentPosition());
            this.opMode.telemetry.addData("Right Front Current Position: ", rightFront.getCurrentPosition());
            this.opMode.telemetry.addData("Left Back Motor Current Position: ", leftBack.getCurrentPosition());
            this.opMode.telemetry.addData("Right Back Motor Current Position: ", rightBack.getCurrentPosition());
            this.opMode.telemetry.update();
            //sends data on the positions of where the motor is located
        }

        leftFront.setPower(0);
        rightFront.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        if((this.isHueBlue(this.getHueValue(leftColorSensor)) || this.isHueRed(this.getHueValue(leftColorSensor))))
        {
            // If the left sensor detected red or blue, turn the robot left so that the right sensor can detect the same color
            this.TurnLeftAndAlign(leftFront, rightFront, leftBack, rightBack, rightColorSensor, turnPower);
            this.opMode.telemetry.addLine("left color is found");
        }
        else if((this.isHueBlue(this.getHueValue(rightColorSensor)) || this.isHueRed(this.getHueValue(rightColorSensor))))
        {
            // If the right sensor detected red or blue, turn the robot right so that the left sensor can detect the same color
            this.TurnRightAndAlign(leftFront, rightFront, leftBack, rightBack, leftColorSensor, turnPower);
            this.opMode.telemetry.addLine("right color is found");
        }
    }

    private void TurnLeftAndAlign(DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack,
                                  ColorSensor rightColorSensor, double turnPower)
    {
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while(this.opMode.opModeIsActive() && (!this.isHueBlue(this.getHueValue(rightColorSensor))
                && !this.isHueRed(this.getHueValue(rightColorSensor)))) {

            // turn left
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(turnPower);
            rightBack.setPower(turnPower);
        }

        // stop turning when right color sensor finds the color
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    private void TurnRightAndAlign(DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack,
                                   ColorSensor leftColorSensor, double turnPower)
    {
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while(this.opMode.opModeIsActive() && (!this.isHueBlue(this.getHueValue(leftColorSensor))
                && !this.isHueRed(this.getHueValue(leftColorSensor)))) {

            // turn left
            leftFront.setPower(turnPower);
            leftBack.setPower(turnPower);
            rightFront.setPower(0);
            rightBack.setPower(0);
        }

        // stop turning when right color sensor finds the color
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    public void wallFollow(DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack,
                           DistanceSensor frontDistanceSensor, DistanceSensor backDistanceSensor, double turnPower, double inches)
    {
        //set the wheel directions
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        //reset encoders
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double frontDistance = 0;
        double backDistance = 0;

        while (this.opMode.opModeIsActive()) {
            frontDistance = frontDistanceSensor.getDistance(DistanceUnit.INCH);
            backDistance = backDistanceSensor.getDistance(DistanceUnit.INCH);

            this.opMode.telemetry.addData("Front Distance", frontDistance);
            this.opMode.telemetry.addData("Back Distance", backDistance);
            this.opMode.telemetry.update();

            if (frontDistance > backDistance || (frontDistance > inches && backDistance > inches)) {
                // turn left
                leftFront.setPower(turnPower * 0.65);
                rightFront.setPower(turnPower);
                leftBack.setPower(turnPower * 0.65);
                rightBack.setPower(turnPower);
            } else if (frontDistance < backDistance || (frontDistance < inches && backDistance < inches)) {
                //turn right
                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);

                leftFront.setPower(turnPower);
                rightFront.setPower(turnPower * 0.65);
                leftBack.setPower(turnPower);
                rightBack.setPower(turnPower * 0.65);
            } else if (frontDistance == backDistance) {

                //go straight
                leftFront.setPower(turnPower);
                rightFront.setPower(turnPower);
                leftBack.setPower(turnPower);
                rightBack.setPower(turnPower);
            }
        }
    }

    //Check if the hue value is within the range to detect the gold mineral
    private boolean isHueRed(int hueValue)
    {
        return ((hueValue >= minRedHueLRange && hueValue <= maxRedHueLRange) || (hueValue >= minRedHueURange && hueValue <= maxRedHueURange));
    }

    //Check if the hue value is within the range to detect the gold mineral
    private boolean isHueBlue(int hueValue)
    {
        return (hueValue >= minBlueHueRange && hueValue <= maxBlueHueRange);
    }

    // **************************** END OF ALIGNING OF ROBOT WITH COLOR SENSOR ********************************************

    //*****************************         START OF SERVO FUNCTIONS           *******************************************
    public void dropTeamMarker (Servo markerServo, double position) {
        markerServo.setPosition(position);
    }
}
