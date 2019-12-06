/*
 */
package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

/**
 * Function to find the skystone and move the robot to it
 */

public class FindSkystoneOpenCV {
    private LinearOpMode opMode;
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private double scale;
    private double forwardScale;

    private int valMid = -1;
    private int valLeft = -1;
    private int valRight = -1;

    private static float rectHeight = 1.0f / 8f;
    private static float rectWidth = 1.6f / 8f;

    private float[] midPos = {4f / 8f, 4f / 8f};//0 = col, 1 = row
    private float[] leftPos = {2.2f / 8f, 4f / 8f};
    private float[] rightPos = {5.8f / 8f, 4f / 8f};
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private final int rows = 640;
    private final int cols = 480;

    private OpenCvCamera webCam;

    public FindSkystoneOpenCV(LinearOpMode _opMode) {
        opMode = _opMode;

        leftFront = this.opMode.hardwareMap.dcMotor.get("left_front");
        rightFront = this.opMode.hardwareMap.dcMotor.get("right_front");
        leftBack = this.opMode.hardwareMap.dcMotor.get("left_back");
        rightBack = this.opMode.hardwareMap.dcMotor.get("right_back");

        int cameraMonitorViewId = this.opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", this.opMode.hardwareMap.appContext.getPackageName());
        webCam = OpenCvCameraFactory.getInstance().createWebcam(this.opMode.hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Open the connection to the camera device and start streaming
        webCam.openCameraDevice();
        webCam.setPipeline(new StageSwitchingPipeline());
        webCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
    }

    /*
     * Unitl function to move the robot in front of the skystone
     */
    public void FindSkystoneAndMoveRobot() {

        // detect the skytone, middle, left, or right inside three blocks
        while (!this.opMode.isStopRequested()) {
            this.opMode.telemetry.addData("Values", valLeft + "   " + valMid + "   " + valRight);
            this.opMode.telemetry.update();

            // break when only one value is 0 to filter out the noise
            if (valMid == 0 && valLeft != 0 && valRight != 0
            || valMid != 0 && valLeft == 0 && valRight != 0
            || valMid != 0 && valLeft != 0 && valRight == 0) {
                break;
            }
        }

        float leftOffset = 0.0f;
        // move robot left or right based on position
        if (valLeft == 0){
            //left
            leftOffset = -4;
            scale = 1.5;
            forwardScale = 0;
        } else if (valRight == 0){
            //right
            leftOffset = 10;
            scale = 1.5;
            forwardScale = leftOffset / 8;
        } else {
            // middle
            leftOffset = 3;
            scale = 1.5;
            forwardScale = leftOffset / 8;
        }

        moveRobot(leftFront, rightFront, leftBack, rightBack, leftOffset * scale);
        moveRobotForward(leftFront, rightFront, leftBack, rightBack, 32 + forwardScale);

        webCam.closeCameraDevice();
    }

    private void moveRobot
            (DcMotor leftFront, DcMotor rightFront,
             DcMotor leftBack, DcMotor rightBack, double inches) {
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

        double positionsPerInch = 1120 / 12.605;
        //positionsPerInch is the amount of counts the encoder makes to move 1 inch

        //positionsToMove is the amount of counts the encoder must make to reach the target location

        //target position to run to
        this.opMode.telemetry.addLine("set target");

        leftFront.setTargetPosition((int) (inches * positionsPerInch));
        rightFront.setTargetPosition((int) (inches * positionsPerInch));
        leftBack.setTargetPosition((int) (inches * positionsPerInch));
        rightBack.setTargetPosition((int) (inches * positionsPerInch));

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        //ensures robot will go to the destination needed
        this.opMode.telemetry.addLine("run to");
        leftFront.setPower(0.716); //change values
        rightFront.setPower(0.45);
        leftBack.setPower(0.35);
        rightBack.setPower(0.716);
        this.opMode.telemetry.addLine("set power");
        //sets speed at which robot will run, 'power' is input

        while (this.opMode.opModeIsActive()
                && leftFront.isBusy()
                && rightFront.isBusy()
                && leftBack.isBusy()
                && rightBack.isBusy()) {
            this.opMode.telemetry.addData("Left Front Motor Current Position: ", leftFront.getCurrentPosition());
            this.opMode.telemetry.addData("Right Front Current Position: ", rightFront.getCurrentPosition());
            this.opMode.telemetry.addData("Left Back Motor Current Position: ", leftBack.getCurrentPosition());
            this.opMode.telemetry.addData("Right Back Motor Current Position: ", rightBack.getCurrentPosition());
            this.opMode.telemetry.update();
            //sends data on the positions of where the motor is located
        }
    }

    public void moveRobotForward
            (DcMotor leftFront, DcMotor rightFront,
             DcMotor leftBack, DcMotor rightBack, double inches) {

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double positionsPerInch = 1120 / 12.605;
        //positionsPerInch is the amount of counts the encoder makes to move 1 inch

        //positionsToMove is the amount of counts the encoder must make to reach the target location

        //target position to run to
        this.opMode.telemetry.addLine("set target");

        leftFront.setTargetPosition((int) (inches * positionsPerInch * 7 / 5));
        rightFront.setTargetPosition((int) (inches * positionsPerInch));
        leftBack.setTargetPosition((int) (inches * positionsPerInch));
        rightBack.setTargetPosition((int) (inches * positionsPerInch));

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        //ensures robot will go to the destination needed
        this.opMode.telemetry.addLine("run to");
        leftFront.setPower(0.7); //change values
        rightFront.setPower(0.5);
        leftBack.setPower(0.5);
        rightBack.setPower(0.5);
        this.opMode.telemetry.addLine("set power");
        //sets speed at which robot will run, 'power' is input

        while (this.opMode.opModeIsActive()
                && leftFront.isBusy()
                && rightFront.isBusy()
                && leftBack.isBusy()
                && rightBack.isBusy()) {
            this.opMode.telemetry.addData("Left Front Motor Current Position: ", leftFront.getCurrentPosition());
            this.opMode.telemetry.addData("Right Front Current Position: ", rightFront.getCurrentPosition());
            this.opMode.telemetry.addData("Left Back Motor Current Position: ", leftBack.getCurrentPosition());
            this.opMode.telemetry.addData("Right Back Motor Current Position: ", rightBack.getCurrentPosition());
            this.opMode.telemetry.update();
            //sends data on the positions of where the motor is located
        }
    }

    //detection pipeline
    public class StageSwitchingPipeline extends OpenCvPipeline {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();

        @Override
        public Mat processFrame(Mat input) {

            //color diff cb.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

            //b&w
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

            yCbCrChan2Mat.copyTo(all);//copies mat object

            updateVals(input);

            //create three points
            Point pointMid = new Point((int) (input.cols() * midPos[0]), (int) (input.rows() * midPos[1]));
            Point pointLeft = new Point((int) (input.cols() * leftPos[0]), (int) (input.rows() * leftPos[1]));
            Point pointRight = new Point((int) (input.cols() * rightPos[0]), (int) (input.rows() * rightPos[1]));

            //draw circles on those points
            Imgproc.circle(all, pointMid, 8, new Scalar(255, 0, 0), 1);//draws circle
            Imgproc.circle(all, pointLeft, 8, new Scalar(255, 0, 0), 1);//draws circle
            Imgproc.circle(all, pointRight, 8, new Scalar(255, 0, 0), 1);//draws circle

            //draw 3 rectangles
            Imgproc.rectangle(//1-3
                    all,
                    new Point(
                            input.cols() * (leftPos[0] - rectWidth / 2),
                            input.rows() * (leftPos[1] - rectHeight / 2)),
                    new Point(
                            input.cols() * (leftPos[0] + rectWidth / 2),
                            input.rows() * (leftPos[1] + rectHeight / 2)),
                    new Scalar(0, 255, 0), 1);
            Imgproc.rectangle(//3-5
                    all,
                    new Point(
                            input.cols() * (midPos[0] - rectWidth / 2),
                            input.rows() * (midPos[1] - rectHeight / 2)),
                    new Point(
                            input.cols() * (midPos[0] + rectWidth / 2),
                            input.rows() * (midPos[1] + rectHeight / 2)),
                    new Scalar(0, 255, 0), 1);
            Imgproc.rectangle(//5-7
                    all,
                    new Point(
                            input.cols() * (rightPos[0] - rectWidth / 2),
                            input.rows() * (rightPos[1] - rectHeight / 2)),
                    new Point(
                            input.cols() * (rightPos[0] + rectWidth / 2),
                            input.rows() * (rightPos[1] + rectHeight / 2)),
                    new Scalar(0, 255, 0), 1);

            return all;
        }

        public void updateVals(Mat input) {
            //get values from frame
            double[] pixMid = thresholdMat.get((int) (input.rows() * midPos[1]), (int) (input.cols() * midPos[0]));//gets value at circle
            valMid = (int) pixMid[0];

            double[] pixLeft = thresholdMat.get((int) (input.rows() * leftPos[1]), (int) (input.cols() * leftPos[0]));//gets value at circle
            valLeft = (int) pixLeft[0];

            double[] pixRight = thresholdMat.get((int) (input.rows() * rightPos[1]), (int) (input.cols() * rightPos[0]));//gets value at circle
            valRight = (int) pixRight[0];
        }
    }
}