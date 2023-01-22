package org.firstinspires.ftc.teamcode.ActionSystem;

import org.firstinspires.ftc.teamcode.ActionSystem.actions.CustomAction;
import org.firstinspires.ftc.teamcode.ActionSystem.actions.Wait;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.Timer;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;

public class TeleOpAction {

    int currentState = 0;
    public boolean start;
    public boolean held_down = false;
    boolean isComplete = false;
    boolean hasStartedAction = false;


    Timer timer = new Timer();

    BooleanSupplier bool;

    ArrayList<Action> actionSequence;

    public TeleOpAction(Robot robot) {
        actionSequence = new ArrayList<>();
    }

    public TeleOpAction(ArrayList<Action> actionSequence) {
        this.actionSequence = actionSequence;
    }

    public ArrayList<Action> getActionList() {
        return actionSequence;
    }

    public void addAction(Action action) {
        actionSequence.add(action);
    }

    public void addCustomAction(Runnable runnable) {
        actionSequence.add(new CustomAction(runnable));
    }

    public void addWait(double seconds) {
        actionSequence.add(new Wait(seconds));
    }

    public boolean isActionRunning() {
        return currentState > 0 && currentState != getActionList().size()-1;
    }

    public void reset() {
        currentState = 0;
        start = false;
        isComplete = false;
        hasStartedAction = false;

        for(Action a : actionSequence) {
            a.isComplete = false;
        }
        timer.reset();
    }

    public void setStartButton(BooleanSupplier bool) {
        this.bool = bool;
    }

    public void start(BooleanSupplier bool) {
        setStartButton(bool);
    }

    public void start() {
        bool = ()-> true;
        held_down = true;
    }

    public void run() throws InterruptedException {
        if(bool != null && bool.getAsBoolean() && !start && timer.currentSeconds() > 0.4) {
            start = true;
            timer.reset();
        }
        else if (bool != null && bool.getAsBoolean() && !isComplete && start && timer.currentSeconds() > 0.4) {
            currentState = 0;
            start = false;
            reset();
        }

        if(held_down) {
            bool = ()-> false;
        }

        if(start) {
            if (!hasStartedAction) {
                actionSequence.get(currentState).startAction();
                hasStartedAction = true;
            }

            if (actionSequence.get(currentState).isActionComplete() && currentState < actionSequence.size() - 1) {
                actionSequence.get(currentState).stopAction();
                currentState += 1;

                hasStartedAction = false;
            } else if (actionSequence.get(currentState).isActionComplete() && currentState == actionSequence.size() - 1) {
                actionSequence.get(currentState).stopAction();
                isComplete = true;
            } else {
                actionSequence.get(currentState).runAction();
            }
        }

        if(isComplete) {
            reset();
        }
    }

}
