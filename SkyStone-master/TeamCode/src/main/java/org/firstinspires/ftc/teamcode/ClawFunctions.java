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

    public void openl(Config config)
    {
        config.claw.setPosition(.5);
    }
    public void openf(Config config)
    {
        config.claw.setPosition(.85);
    }
    public void close(Config config)
    {
        config.claw.setPosition(-.7);
    }

    public void pull(Config config)
    {
        config.leftPull.setPosition(1);
        config.rightPull.setPosition(0);
    }
    public void nopull(Config config)
    {
        config.leftPull.setPosition(0   );
        config.rightPull.setPosition(1);
    }

    ElapsedTime timer = new ElapsedTime();

    public void setTimer(ElapsedTime timer) {
        this.timer = timer;
    }

    public ElapsedTime getTimer() {
        return timer;
    }


}