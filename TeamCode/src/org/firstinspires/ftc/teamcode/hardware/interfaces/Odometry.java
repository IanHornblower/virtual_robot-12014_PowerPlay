package org.firstinspires.ftc.teamcode.hardware.interfaces;

import org.firstinspires.ftc.teamcode.math.Pose2D;

public abstract class Odometry implements Subsystem {

    public Pose2D position;
    public Pose2D veloPos = new Pose2D(0, 0, 0);
    public double accumulatedDistance = 0;

    public void setStartPosition(Pose2D start) {
        position = start;
    }

    public double getAccumulatedDistance() {
        return accumulatedDistance;
    }

    public Pose2D getPose() {
        return position;
    }

    public Pose2D getRawVelocityPos() {
        return veloPos;
    }

    public Pose2D getRotatedVelocityPos() {
        return veloPos.rotate(position.heading);
    }

}
