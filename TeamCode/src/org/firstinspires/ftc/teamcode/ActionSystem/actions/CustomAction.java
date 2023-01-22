package org.firstinspires.ftc.teamcode.ActionSystem.actions;

import org.firstinspires.ftc.teamcode.ActionSystem.Action;

public class CustomAction extends Action {

    Runnable runnable;

    public CustomAction(Runnable runnable) {
        this.runnable = runnable;
    }
    @Override
    public void startAction() {

    }

    @Override
    public void runAction() throws InterruptedException {
        runnable.run();
        isComplete = true;
    }

    @Override
    public void stopAction() {

    }
}
