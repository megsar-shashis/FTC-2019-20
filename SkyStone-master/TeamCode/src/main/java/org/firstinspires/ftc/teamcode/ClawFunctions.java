package org.firstinspires.ftc.teamcode;

public class ClawFunctions {
    public ClawFunctions()
    {

    }

    public void pickOrient(Config config)
    {
        config.orient.setPosition(.5);
    }

    public void transportOrient(Config config)
    {
        config.orient.setPosition(.75);
    }

    public void open(Config config)
    {
        config.leftClaw.setPosition(.5);
        config.rightClaw.setPosition(.5);
    }

    public void close(Config config)
    {
        config.leftClaw.setPosition(1);
        config.rightClaw.setPosition(0);
    }
}
