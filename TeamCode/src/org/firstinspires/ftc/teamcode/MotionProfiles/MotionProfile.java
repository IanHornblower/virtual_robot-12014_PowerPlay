package org.firstinspires.ftc.teamcode.MotionProfiles;

public abstract class MotionProfile {

    double distance;

    double minVelocity;
    double maxVelocity;

    double duration;
    double averageVelocity;

    public abstract void create();

    public abstract double calculate(double currentTime);
}
