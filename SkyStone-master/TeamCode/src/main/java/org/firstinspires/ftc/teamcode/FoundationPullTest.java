package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Pull Foundation Test", group = "Autonomous")
public class FoundationPullTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    Config c = new Config();
    MecanumFunctions mecanumFunctions = new MecanumFunctions(this);
    //AutonomousFunctions Utils = new AutonomousFunctions(this);
    public void runOpMode() {
        c.init(hardwareMap);

        waitForStart();
        mecanumFunctions.MoveRobot(c, MecanumFunctions.MoveAction.SlideRight, 80, 0.5 );
        telemetry.addLine("done 1");
        telemetry.update();
        sleep(1000);
        telemetry.clearAll();

        mecanumFunctions.MoveRobot(c, MecanumFunctions.MoveAction.MoveForward, 20, 0.5 );
        telemetry.addLine("done 2");
        telemetry.update();
        sleep(1000);
        telemetry.clearAll();

        mecanumFunctions.MoveRobot(c, MecanumFunctions.MoveAction.SlideRight, 5, 0.5 );
        telemetry.addLine("done 3");
        telemetry.update();
        sleep(1000);
        telemetry.clearAll();

        mecanumFunctions.MoveRobot(c, MecanumFunctions.MoveAction.MoveForward, 15, 0.5 );
        telemetry.addLine("done 4");
        telemetry.update();
        sleep(1000);
        telemetry.clearAll();

        c.leftPull.setPosition(0);
        c.rightPull.setPosition(1);
        telemetry.addLine("done 5");
        telemetry.update();
        sleep(1000);
        telemetry.clearAll();

        mecanumFunctions.MoveRobot(c, MecanumFunctions.MoveAction.MoveBackward, 46, 0.5 );
        telemetry.addLine("done 6");
        telemetry.update();
        sleep(1000);
        telemetry.clearAll();

        c.leftPull.setPosition(1);
        c.rightPull.setPosition(0);
        telemetry.addLine("done 7");
        telemetry.update();
        sleep(1000);
        telemetry.clearAll();

        mecanumFunctions.MoveRobot(c, MecanumFunctions.MoveAction.SlideLeft, 55, 0.5 );
        telemetry.addLine("done 8");
        telemetry.update();
        sleep(1000);
        telemetry.clearAll();
    }
}
