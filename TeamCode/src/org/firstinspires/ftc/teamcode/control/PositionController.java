package org.firstinspires.ftc.teamcode.control;

import java.util.function.DoubleSupplier;

public class PositionController {

    double kP = 0, kV = 0;
    double targetPosition, targetVelocity = 0;

    public PositionController(double kP, double kV) {
        this.kP = kP;
        this.kV = kV;
    }

    public void setTargets(double targetPosition, double targetVelocity) {
        this.targetPosition = targetPosition;
        this.targetVelocity = targetVelocity;
    }

    public double calculate(double position, double velocity) {
        double positionError = targetPosition - position;
        double velocityError = targetVelocity - velocity;

        return positionError * kP + targetVelocity * kV;
    }

    public double calculateWithError(double error, double targetVelocity) {
        return error * kP + targetVelocity * kV;
    }

}
