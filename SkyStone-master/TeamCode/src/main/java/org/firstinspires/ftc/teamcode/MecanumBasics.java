package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by meghn on 9/23/2018.
 */
@Autonomous(name = "MecBasicsREAL", group = "Autonomous")
public class MecanumBasics extends LinearOpMode {
    Config robot = new Config();
    private ElapsedTime runtime = new ElapsedTime();

    AutonomousFunctions Utils = new AutonomousFunctions(this);
    public void runOpMode() {
        robot.init(hardwareMap);

        Utils.MecanumSlideLeftInInches(robot.leftFront, robot.rightFront, robot.leftBack,
                robot.rightBack, 0.716, 0.42, 0.35, 0.716, 20);

        Utils.MecanumSlideRightInInches(robot.leftFront, robot.rightFront, robot.leftBack,
                robot.rightBack, 0.716, 0.42, 0.35, 0.716, 20);

        Utils.MecanumMoveForwardInInches(robot.leftFront, robot.rightFront, robot.leftBack,
                robot.rightBack, 0.7, 0.6, 0.5, 0.5, 20);

        Utils.MecanumMoveBackwardInInches(robot.leftFront, robot.rightFront, robot.leftBack,
                robot.rightBack, 0.8, 0.6, 0.5, 0.5, 20);
    }
}

