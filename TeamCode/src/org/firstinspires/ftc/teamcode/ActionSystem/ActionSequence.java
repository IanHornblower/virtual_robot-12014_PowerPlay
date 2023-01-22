package org.firstinspires.ftc.teamcode.ActionSystem;

import org.firstinspires.ftc.teamcode.ActionSystem.actions.CustomAction;
import org.firstinspires.ftc.teamcode.ActionSystem.actions.Wait;

import java.util.ArrayList;

public class ActionSequence {

    ArrayList<Action> actionSequence;

    public ActionSequence() {
        actionSequence = new ArrayList<>();
    }

    public ActionSequence(ArrayList<Action> actionSequence) {
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

}
