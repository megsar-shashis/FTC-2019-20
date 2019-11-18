/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

/**
 * This file illustrates the concept of driving up to a line and then stopping.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code shows using two different light sensors:
 *   The Primary sensor shown in this code is a legacy NXT Light sensor (called "sensor_light")
 *   Alternative "commented out" code uses a MR Optical Distance Sensor (called "sensor_ods")
 *   instead of the LEGO sensor.  Chose to use one sensor or the other.
 *
 *   Setting the correct WHITE_THRESHOLD value is key to stopping correctly.
 *   This should be set half way between the light and dark values.
 *   These values can be read on the screen once the OpMode has been INIT, but before it is STARTED.
 *   Move the senso on asnd off the white line and not the min and max readings.
 *   Edit this code to make WHITE_THRESHOLD half way between the min and max.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Skystone Vuforia", group="Auto")
//@Disabled
public class SkystoneVuforia extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private static final String VUFORIA_KEY =
            "AR/ZUHH/////AAABmUqm26qkS0kJtcAm07xRqe5jkIrpagCp8Mt6fJQLNN3uG4F5Qn6UIwRnhbinYkn+S+rbdFMcS0aEcORq5kSi5hNxMxGq7YB3V2f8pDhtPJFb5DDLwzrhKDIwGI8CST3T+JhN6mQhsHnMI45xtjMASIKs6v2b0ZpYh2YvzNY8ZgDDK4jVSZ9wg7jGlIlOVnUINeMtSoUnrXRCqqQ6OHZSNVkPjcP7pPitJuXgPltX0uz+b90EWWOsxzW4K2R2+3FE5WtUr6/8MOgUHDc+64BqVJe7ird88ctEJ/W0E5rRspZ6BRre1N9/4x19XZDyLJWKIcMiAtepqz8T7F55GbEyPNXTet3KYAbPeCR+Sj7YuOoE";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    /* Declare OpMode members. */
    Config         robot   = new Config();   // Use a Pushbot's hardware

    @Override
    public void runOpMode() {

        /* Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        //move forward ~12 in
        //scan for skystone + identify if case 1,2,3
        SkystoneLocation locationPrint = FindSkystone();
        telemetry.addData("location", locationPrint.toString());
        telemetry.update();
        //pick skystone
        //move back ~6 in
        //travel position
        //move under bridge
        //turn 90 deg left
        //hook and pull
        //place block
        //travel position
        //under the bridge
    }

    //find location of skystone
    public enum SkystoneLocation{
        NONE,
        LEFT,
        CENTER,
        RIGHT,
        FORWARD,
        BACKWARD
    }
    private SkystoneLocation FindSkystone(){
        SkystoneLocation skystoneLocation = SkystoneLocation.NONE;
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive() && skystoneLocation == SkystoneLocation.NONE) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
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
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());

                            objectHeight = recognition.getHeight();
                            imageHeight = recognition.getImageHeight();
                            ratioHeight = objectHeight/imageHeight;
                            horizontalAngle = recognition.estimateAngleToObject(AngleUnit.DEGREES);

                            telemetry.addData("Estimated Angle", "%.3f", horizontalAngle);
                            telemetry.addData("Height Ratio", "%.3f", ratioHeight);
                            telemetry.addData("Object Height", "%.3f", objectHeight);
                            telemetry.addData("Image Height", "%.3f", imageHeight);
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
                            telemetry.addData("Skystone Detected: ", skystone.size());
                            if(skystone.size() == 1){
                                skystoneUsed = 0;
                            }else{
                                if(skystone.get(0).estimateAngleToObject(AngleUnit.DEGREES) > skystone.get(1).estimateAngleToObject(AngleUnit.DEGREES)){
                                    skystoneUsed = 0;
                                }else{
                                    skystoneUsed = 1;
                                }
                            }
                            if(skystone.get(skystoneUsed).estimateAngleToObject(AngleUnit.DEGREES) < -5){
                                //move left
                                skystoneLocation = SkystoneLocation.LEFT;
                            } else if(skystone.get(skystoneUsed).estimateAngleToObject(AngleUnit.DEGREES) > 5){
                                //move right
                                skystoneLocation = skystoneLocation.RIGHT;
                            } else {
                                //skystoneLocation = skystoneLocation.CENTERX;
                                if (skystone.get(skystoneUsed).getHeight()/skystone.get(skystoneUsed).getImageHeight() < 0.45){
                                    //move forwards
                                    skystoneLocation = SkystoneLocation.FORWARD;
                                }else if (skystone.get(skystoneUsed).getHeight()/skystone.get(skystoneUsed).getImageHeight() > 0.55){
                                    //move backwards
                                    skystoneLocation = SkystoneLocation.BACKWARD;
                                }else{
                                    skystoneLocation = SkystoneLocation.CENTER;
                                }
                            }
                        }else if(stone.size() > 0){
                            //No Skystones, Move to stone
                            telemetry.addData("No Skystones Detected, Stones Detected: ", stone.size());

                        }else{
                            //Nothing Detected
                            telemetry.addData("Nothing Detected", 0);
                        }

                        telemetry.update();
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
        return skystoneLocation;
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
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.7;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
