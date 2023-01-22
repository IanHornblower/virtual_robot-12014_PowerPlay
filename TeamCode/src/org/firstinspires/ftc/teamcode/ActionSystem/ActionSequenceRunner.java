package org.firstinspires.ftc.teamcode.ActionSystem;

import org.firstinspires.ftc.teamcode.hardware.Robot;

import java.util.ArrayList;

public class ActionSequenceRunner {

    Robot robotBase;

    ArrayList<Action> actionList;

    int currentState = 0;
    boolean isComplete = false;
    boolean hasStartedAction = false;

    Action currentPersistentAction = null;

    public  ActionSequenceRunner(Robot robotBase) {
        this.robotBase = robotBase;
    }

    public void setActionArrayList(ArrayList<Action> actionList) {
        this.actionList = actionList;
    }

    public void setActionSequence(ActionSequence actionSequence){
        this.actionList = actionSequence.getActionList();
    }

    public boolean isActionComplete(int i) {
        return actionList.get(i).isComplete;
    }

    public int getCurrentAction() {
        return currentState + 1;
    }

    public void update() throws Exception {
        // Run initialization code once
        if (!hasStartedAction) {
            actionList.get(currentState).startAction();
            hasStartedAction = true;
        }

        // Run through each of the actions after the previous action has completed
        if (actionList.get(currentState).isActionComplete() && currentState < actionList.size() - 1) {
            actionList.get(currentState).stopAction();
            currentState += 1;
            if (actionList.get(currentState).isActionPersistent()) {
                currentPersistentAction = actionList.get(currentState);
            }
            hasStartedAction = false;

        // Stop Action if the required criteria is reached
        } else if (actionList.get(currentState).isActionComplete() && currentState == actionList.size() - 1) {
            actionList.get(currentState).stopAction();
            isComplete = true;
        } else {
            actionList.get(currentState).runAction();
        }

        // If the action is persistent continue it running
        if (currentPersistentAction != null
                && (!actionList.get(currentState).isActionPersistent()
                || currentState == actionList.size() - 1) && hasStartedAction
                && !currentPersistentAction.isAMultipleAction()) {
            currentPersistentAction.runAction();
        }

        robotBase.update();
    }

    public boolean isComplete() {
        return isComplete;
    }

}
