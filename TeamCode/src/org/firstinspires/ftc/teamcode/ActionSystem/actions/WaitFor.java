package org.firstinspires.ftc.teamcode.ActionSystem.actions;

import org.firstinspires.ftc.teamcode.ActionSystem.Action;

import java.util.function.BooleanSupplier;

public class WaitFor extends Action {

    BooleanSupplier value;
    Runnable runnable;

    public WaitFor(BooleanSupplier value) {
        this.value = value;
    }

    public WaitFor(BooleanSupplier value, Runnable runnable) {
        this.runnable = runnable;
        this.value = value;
    }

    @Override
    public void startAction() {
    }

    @Override
    public void runAction() throws InterruptedException {
        isComplete = value.getAsBoolean();
    }

    @Override
    public void stopAction() {
        if(runnable != null) {
            runnable.run();
        }
    }
}
