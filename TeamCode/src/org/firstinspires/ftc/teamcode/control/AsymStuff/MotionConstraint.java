package org.firstinspires.ftc.teamcode.control.AsymStuff;

public class MotionConstraint {

    /**
     * the maximum initial acceleration we allow the robot to experience
     */
    public double max_acceleration;
    /**
     * the maximum final deceleration we allow the robot to experience
     *
     * Generally this is lower than {@link #max_acceleration}
     */
    public double max_deceleration;

    /**
     * the maximum velocity we can reasonably reach in this given trajectory
     */
    public double max_velocity;


    public MotionConstraint(double max_acceleration, double max_deceleration, double max_velocity) {
        this.max_acceleration = max_acceleration;
        this.max_deceleration = max_deceleration;
        this.max_velocity = max_velocity;
    }


}