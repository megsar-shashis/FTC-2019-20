/*
Created by NoahS 11.30.2019
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

/**
 This program uses a webcam to finds the position of the skystone and moves to it. To use this, create an instance and
 call FindSkystoneAndMoveRobot();. The initialization of vuforia happens inside the constructor.
 * Function to find the skystone and move the robot to it
 */
public class FindSkystoneFunction{
    private LinearOpMode opMode;
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private double scale;
    private double forwardScale;
    private VuforiaLocalizer vuforia;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;
    private OpenGLMatrix lastLocation = null;

    private VuforiaTrackables targetsSkyStone;
    private VuforiaTrackable stoneTarget;

    public FindSkystoneFunction(LinearOpMode _opMode) {
        this.opMode = _opMode;
        this.initVuforia();

        leftFront = this.opMode.hardwareMap.dcMotor.get("left_front");
        rightFront = this.opMode.hardwareMap.dcMotor.get("right_front");
        leftBack = this.opMode.hardwareMap.dcMotor.get("left_back");
        rightBack = this.opMode.hardwareMap.dcMotor.get("right_back");
    }

    /*
     * Unitl function to move the robot in front of the skystone
     */
    public void FindSkystoneAndMoveRobot() {
        // first of all, move the robot 8 inches forward from the start position
        moveRobotForward (leftFront, rightFront, leftBack, rightBack, 8);

        //Identifies stone of offset of left or right
        float yOffset = 0.0f;
        while (!this.opMode.isStopRequested()) {
            // check if stone target is visible
            if (((VuforiaTrackableDefaultListener)this.stoneTarget.getListener()).isVisible()) {
                this.opMode.telemetry.addData("Visible Target", this.stoneTarget.getName());

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)this.stoneTarget.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }

                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                yOffset = translation.get(1) / mmPerInch;
                this.opMode.telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                this.opMode.telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);

                break;
            }
            else {
                this.opMode.telemetry.addData("Visible Target", "none");
            }
            this.opMode.telemetry.update();
        }

        // move robot left or right based on yOffset
        if (yOffset < 0) {
            scale = 1.5;
            forwardScale = 0;
        } else {
            scale = 3;
            forwardScale = yOffset/8;
        }

        moveRobot(leftFront,rightFront,leftBack,rightBack, yOffset*scale);
        moveRobotForward (leftFront, rightFront, leftBack, rightBack, 20+forwardScale);

        // Disable Tracking when we are done;
        targetsSkyStone.deactivate();
    }
    private void initVuforia(){
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = this.opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId",
                                                                                                "id",
                                                                                                        this.opMode.hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AR/ZUHH/////AAABmUqm26qkS0kJtcAm07xRqe5jkIrpagCp8Mt6fJQLNN3uG4F5Qn6UIwRnhbinYkn+S+rbdFMcS0aEcORq5kSi5hNxMxGq7YB3V2f8pDhtPJFb5DDLwzrhKDIwGI8CST3T+JhN6mQhsHnMI45xtjMASIKs6v2b0ZpYh2YvzNY8ZgDDK4jVSZ9wg7jGlIlOVnUINeMtSoUnrXRCqqQ6OHZSNVkPjcP7pPitJuXgPltX0uz+b90EWWOsxzW4K2R2+3FE5WtUr6/8MOgUHDc+64BqVJe7ird88ctEJ/W0E5rRspZ6BRre1N9/4x19XZDyLJWKIcMiAtepqz8T7F55GbEyPNXTet3KYAbPeCR+Sj7YuOoE";

        /**
         * We also indicate which camera on the RC we wish to use.
         */
        parameters.cameraName = this.opMode.hardwareMap.get(WebcamName.class, "Webcam 1");;

        //  Instantiate the Vuforia engine
        this.vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        this.targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        this.stoneTarget = this.targetsSkyStone.get(0);
        this.stoneTarget.setName("SkyStone Target");

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        this.stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        float phoneXRotate    = 0;
        float phoneYRotate    = 0;
        float phoneZRotate    = 0;

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 8.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 6.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0.0f * mmPerInch;   // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        ((VuforiaTrackableDefaultListener)this.stoneTarget.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);

        // Note: To use the remote camera preview:
        // AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
        // Tap the preview window to receive a fresh image.
        this.targetsSkyStone.activate();
    }

    private void moveRobot
            (DcMotor leftFront, DcMotor rightFront,
             DcMotor leftBack, DcMotor rightBack, double inches){
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
        leftFront.setPower(0.715); //change values
        rightFront.setPower(0.45);
        leftBack.setPower(0.35);
        rightBack.setPower(0.715);
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

    private void moveRobotForward
            (DcMotor leftFront, DcMotor rightFront,
             DcMotor leftBack, DcMotor rightBack, double inches){

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
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

        leftFront.setTargetPosition((int)(inches*positionsPerInch*7/5));
        rightFront.setTargetPosition((int)(inches*positionsPerInch));
        leftBack.setTargetPosition((int)(inches*positionsPerInch));
        rightBack.setTargetPosition((int)(inches*positionsPerInch));

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