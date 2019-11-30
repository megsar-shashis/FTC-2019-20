package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by meghn on 9/23/2018.
 */
@Autonomous(name = "Mecanum Basics", group = "RW_Autonomous")
public class RW_Utils_Test_Autonomous_Mecanum_Basics extends LinearOpMode {
    RW_RConfig robot = new RW_RConfig();
    private ElapsedTime runtime = new ElapsedTime();

    MecanumBasedUtils Utils = new MecanumBasedUtils(this);
    public void runOpMode() {
        robot.init(hardwareMap);


         Utils.MecanumSlideLeftInInches(robot.leftFront, robot.rightFront, robot.leftBack,
             robot.rightBack, 0.2, 14);

         Utils.MecanumSlideRightInInches(robot.leftFront, robot.rightFront, robot.leftBack,
             robot.rightBack, 0.2, 14);

         Utils.MecanumMoveForwardInInches(robot.leftFront, robot.rightFront, robot.leftBack,
                 robot.rightBack, 0.3, 21);

         Utils.MecanumMoveBackwardInInches(robot.leftFront, robot.rightFront, robot.leftBack,
                 robot.rightBack, 0.3, 21);
    }
}

