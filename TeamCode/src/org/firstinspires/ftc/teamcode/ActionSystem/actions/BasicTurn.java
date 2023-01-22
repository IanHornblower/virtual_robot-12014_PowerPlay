package org.firstinspires.ftc.teamcode.ActionSystem.actions;

import org.firstinspires.ftc.teamcode.ActionSystem.Action;
import org.firstinspires.ftc.teamcode.control.AsymStuff.TurnOnlyControl;
import org.firstinspires.ftc.teamcode.hardware.Robot;

public class BasicTurn extends Action {

    Robot robot;
    double referenceAngle;
    TurnOnlyControl controller;

    public BasicTurn(Robot robot, double referenceAngle) {
        this.robot = robot;
        this.referenceAngle = referenceAngle;
        this.controller = new TurnOnlyControl(()-> robot.imu.getHeadingInRadians(),referenceAngle);
    }

    @Override
    public void startAction() {
    }

    @Override
    public void runAction() {
        robot.driveTrain.setMotorPowers(controller.calculate().scalarMultiply(-1));
        isComplete = Math.abs(controller.getEndGoalError()) < Math.toRadians(2);
                //&& Math.abs(robot.imu.getAccel()) < Math.toRadians(10);

    }

    @Override
    public void stopAction() {
        robot.driveTrain.setMotorPowers(0,0);
    }
}