/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

/**
 * This 2019-2020 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Skystone game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */

//@Disabled
public class FindSkystoneFunction{

    private LinearOpMode opMode;
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    Config config = new Config();
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;

    public enum SkystoneLocation{
        NONE,
        LEFT,
        CENTER,
        RIGHT,
        //FORWARD,
        //BACKWARD
    }

    SkystoneLocation skystoneLocation = SkystoneLocation.NONE;
    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AR/ZUHH/////AAABmUqm26qkS0kJtcAm07xRqe5jkIrpagCp8Mt6fJQLNN3uG4F5Qn6UIwRnhbinYkn+S+rbdFMcS0aEcORq5kSi5hNxMxGq7YB3V2f8pDhtPJFb5DDLwzrhKDIwGI8CST3T+JhN6mQhsHnMI45xtjMASIKs6v2b0ZpYh2YvzNY8ZgDDK4jVSZ9wg7jGlIlOVnUINeMtSoUnrXRCqqQ6OHZSNVkPjcP7pPitJuXgPltX0uz+b90EWWOsxzW4K2R2+3FE5WtUr6/8MOgUHDc+64BqVJe7ird88ctEJ/W0E5rRspZ6BRre1N9/4x19XZDyLJWKIcMiAtepqz8T7F55GbEyPNXTet3KYAbPeCR+Sj7YuOoE";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;


    public FindSkystoneFunction(LinearOpMode _opMode) {
        opMode = _opMode;
    }

    public void FindSkystoneAndMoveRobot() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        leftFront = this.opMode.hardwareMap.dcMotor.get("left_front");
        rightFront = this.opMode.hardwareMap.dcMotor.get("right_front");
        leftBack = this.opMode.hardwareMap.dcMotor.get("left_back");
        rightBack = this.opMode.hardwareMap.dcMotor.get("right_back");

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            this.opMode.telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }

        if (this.opMode.opModeIsActive()) {
            while (this.opMode.opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        this.opMode.telemetry.addData("# Object Detected", updatedRecognitions.size());
                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        List<Recognition> skystone = new ArrayList<Recognition>();
                        List<Recognition> stone = new ArrayList<Recognition>();
                        float objectHeight = 0;
                        float imageHeight = 0;
                        float ratioHeight = 0;
                        double horizontalAngle = 0;
                        int skystoneUsed = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            this.opMode.telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            this.opMode.telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            this.opMode.telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());

                            objectHeight = recognition.getHeight();
                            imageHeight = recognition.getImageHeight();
                            ratioHeight = objectHeight/imageHeight;
                            horizontalAngle = recognition.estimateAngleToObject(AngleUnit.DEGREES);

                            this.opMode.telemetry.addData("Estimated Angle", "%.3f", horizontalAngle);
                            this.opMode.telemetry.addData("Height Ratio", "%.3f", ratioHeight);
                            this.opMode.telemetry.addData("Object Height", "%.3f", objectHeight);
                            this.opMode.telemetry.addData("Image Height", "%.3f", imageHeight);
                            if (recognition.getLabel().equals("Skystone")){
                                //Add to Skystone List
                                skystone.add(recognition);
                            }else {
                                //Add to Stone List
                                stone.add(recognition);
                            }

                            i++;
                        }

                        //Check if Skystone list is empty
                        if(skystone.size() > 0){
                            //Recognizes Skystone
                            this.opMode.telemetry.addData("Skystone Detected: ", skystone.size());
                            if(skystone.size() == 1){
                                skystoneUsed = 0;
                            }else{
                                if(skystone.get(0).estimateAngleToObject(AngleUnit.DEGREES) > skystone.get(1).estimateAngleToObject(AngleUnit.DEGREES)){
                                    skystoneUsed = 0;
                                }else{
                                    skystoneUsed = 1;
                                }
                            }
                            if(skystone.get(skystoneUsed).estimateAngleToObject(AngleUnit.DEGREES) < -17){
                                //move left
                                skystoneLocation = SkystoneLocation.LEFT;
                                //MecanumMoveInInches(leftFront,rightFront,leftBack,rightBack, 0.2,-0.1);
                            } else if(skystone.get(skystoneUsed).estimateAngleToObject(AngleUnit.DEGREES) > -11){
                                //move right
                                skystoneLocation = skystoneLocation.RIGHT;
                                //MecanumMoveInInches(leftFront,rightFront,leftBack,rightBack, 0.2,0.1);
                            } else {
                                //skystoneLocation = skystoneLocation.CENTER;
                                skystoneLocation = SkystoneLocation.CENTER;
                            }

                            break;
                        }else if(stone.size() > 0){
                            //No Skystones, Move to stone
                            this.opMode.telemetry.addData("No Skystones Detected, Stones Detected: ", stone.size());
                        }else{
                            //Nothing Detected
                            this.opMode.telemetry.addData("Nothing Detected", 0);
                        }

                        this.opMode.telemetry.addData("Location",skystoneLocation.toString());

                        this.opMode.telemetry.update();
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }

        if (skystoneLocation == SkystoneLocation.LEFT) {
            moverobot(leftFront,rightFront,leftBack,rightBack, 0.1);
        } else if (skystoneLocation == SkystoneLocation.RIGHT){
            moverobot(leftFront,rightFront,leftBack,rightBack, -0.1);
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



    public void moverobot
            (DcMotor leftFront, DcMotor rightFront,
             DcMotor leftBack, DcMotor rightBack, double inches)
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

        leftFront.setTargetPosition((int)(inches*positionsPerInch));
        rightFront.setTargetPosition((int)(inches*positionsPerInch));
        leftBack.setTargetPosition((int)(inches*positionsPerInch));
        rightBack.setTargetPosition((int)(inches*positionsPerInch));

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        //ensures robot will go to the destination needed
        this.opMode.telemetry.addLine("run to");
        leftFront.setPower(0.22); //change values
        rightFront.setPower(0.25);
        leftBack.setPower(0.2);
        rightBack.setPower(0.25);
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
}