package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

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
        config.claw.setPosition(.5);
    }

    public void close(Config config)
    {
        config.claw.setPosition(1);
    }

    ElapsedTime timer = new ElapsedTime();

    public void setTimer(ElapsedTime timer) {
        this.timer = timer;
    }

    public ElapsedTime getTimer() {
        return timer;
    }


}