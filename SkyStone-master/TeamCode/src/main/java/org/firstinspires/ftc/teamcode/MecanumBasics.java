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

    MecanumFunctions mecanumFunctions = new MecanumFunctions(this);
    //AutonomousFunctions Utils = new AutonomousFunctions(this);
    public void runOpMode() {
        robot.init(hardwareMap);

        waitForStart();
        mecanumFunctions.MoveRobot(robot, MecanumFunctions.MoveAction.MoveForward, 10, 0.5 );

        mecanumFunctions.MoveRobot(robot, MecanumFunctions.MoveAction.MoveBackward, 10, 0.5 );

        mecanumFunctions.MoveRobot(robot, MecanumFunctions.MoveAction.SlideLeft, 10, 0.5 );

        mecanumFunctions.MoveRobot(robot, MecanumFunctions.MoveAction.SlideRight, 10, 0.5 );

        mecanumFunctions.MoveRobot(robot, MecanumFunctions.MoveAction.TurnLeft, 10, 0.5 );

        mecanumFunctions.MoveRobot(robot, MecanumFunctions.MoveAction.TurnRight, 10, 0.5 );
    }
}

