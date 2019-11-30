package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class ClawFunctions {
    public ClawFunctions()
    {

    }

    public void closePull(Config config)
    {
        config.leftPull.setPosition(.5);
        config.rightPull.setPosition(.5);
    }

    public void openPull(Config config)
    {
        config.leftPull.setPosition(1);
        config.rightPull.setPosition(1);
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
