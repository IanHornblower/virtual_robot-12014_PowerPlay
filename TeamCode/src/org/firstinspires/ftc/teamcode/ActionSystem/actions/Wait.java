package org.firstinspires.ftc.teamcode.ActionSystem.actions;

import org.firstinspires.ftc.teamcode.ActionSystem.Action;
import org.firstinspires.ftc.teamcode.util.Timer;

public class Wait extends Action {

    Timer timer;
    double seconds;

    public Wait(double seconds) {
        this.seconds = seconds;
        this.timer = new Timer();
    }

    @Override
    public void startAction() {
        timer.start();
    }

    @Override
    public void runAction() throws InterruptedException {
        isComplete = timer.currentSeconds() > seconds;
    }

    @Override
    public void stopAction() {
        timer.reset();
    }
}
