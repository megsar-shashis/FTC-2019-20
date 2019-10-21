

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;



@TeleOp(name="ElbowTest", group="Test")
//@Disabled
public class ElbowTest extends OpMode{

    /* Declare OpMode members. */
    ClawFunctions cf       = new ClawFunctions(); // use the class created to define a Pushbot's hardware
    Config robot = new Config();


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double position = robot.VHElbow.getPosition();
        if(gamepad1.x == true)
        {
            position += 0.01;
            robot.orient.setPosition(position);
        }
        if(gamepad1.y == true)
        {
            position -= 0.01;
            robot.orient.setPosition(position);
        }
        if(gamepad1.x == false && gamepad1.y == false)
        {
            position = position;
            robot.orient.setPosition(position);
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
