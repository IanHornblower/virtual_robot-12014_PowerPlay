package org.firstinspires.ftc.teamcode.hardware.subsystems;

import org.firstinspires.ftc.teamcode.hardware.DriveConstraints;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.interfaces.Odometry;
import org.firstinspires.ftc.teamcode.hardware.interfaces.Subsystem;
import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.Timer;

public class ThreeWheelOdometry extends Odometry implements Subsystem {
    Encoder left = null;
    Encoder right = null;
    Encoder lateral = null;

    double xVelo = 0;
    double yVelo = 0;
    double hVelo = 0;
    double previousLeft = 0;
    double previousRight = 0;
    double previousLateral = 0;

    Robot robot;

    Timer timer = new Timer();

    public ThreeWheelOdometry(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void setStartPosition(Pose2D start) {
        position = start;
    }

    private double convertTicks(double ticks) {
        return ticks * (2.0 * Math.PI * DriveConstraints.wheelRadius / DriveConstraints.ticksPerRevolution);
    }

    private void correctAngle() {
        if(position.heading > Math.PI * 2.0) {
            position.heading -= 2.0 * Math.PI;
        }
        else if(position.heading < 0) {
            position.heading += 2.0 * Math.PI;
        }
    }

    @Override
    public void init() throws InterruptedException {
        this.left = robot.driveTrain.left;
        this.right = robot.driveTrain.right;
        this.lateral = robot.driveTrain.lateral;

        left.setDirection(Encoder.Direction.REVERSE);
        //right.setDirection(Encoder.Direction.REVERSE);
        lateral.setDirection(Encoder.Direction.REVERSE);

        timer.start();
    }

    @Override
    public void update() throws InterruptedException { // Invert Left, Lateral
        double currentLeft = convertTicks(left.getCurrentPosition());
        double currentRight = convertTicks(right.getCurrentPosition());
        double currentLateral = convertTicks(lateral.getCurrentPosition());

        double currentLeftVelo = convertTicks(left.getRawVelocity());
        double currentRightVelo = convertTicks(right.getRawVelocity());
        double currentLateralVelo = convertTicks(lateral.getRawVelocity());

        double dLeft = currentLeft - previousLeft;
        double dRight = currentRight - previousRight;
        double dLateral = currentLateral - previousLateral;

        previousLeft = currentLeft;
        previousRight = currentRight;
        previousLateral = currentLateral;

        // Pose
        double dTheta = (dRight - dLeft) / (DriveConstraints.trackWidth);
        //double dTheta = (dLeft - dRight) / (DriveConstraints.trackWidth);
        double dx = dLateral + DriveConstraints.lateralOffset * dTheta;
        double dy = (dLeft + dRight) / 2.0;

        double newAngle = position.heading + dTheta / 2;

        Point deltaPoint = new Point(dx, dy).rotate(newAngle);

        deltaPoint.invertX(); // Not Inverse Field Axes
        deltaPoint.invertPoint();

        position.addPoint(deltaPoint);
        position.heading += dTheta;

        correctAngle();

        // Velocity
        hVelo = (currentLeftVelo - currentRightVelo) / DriveConstraints.trackWidth;
        yVelo = (currentLeftVelo + currentRightVelo) / 2.0;
        xVelo = currentLateralVelo - DriveConstraints.lateralOffset * dTheta;

        veloPos = new Pose2D(xVelo, yVelo, hVelo);

        // Accumulated Distance
        accumulatedDistance += deltaPoint.hypot();
    }

    @Override
    public double getAccumulatedDistance() {
        return accumulatedDistance;
    }

    public void resetAccumulatedDistance() {
        accumulatedDistance = 0;
    }

    @Override
    public Pose2D getPose() {
        return position;
    }

    @Override
    public Pose2D getRawVelocityPos() {
        return veloPos;
    }

    @Override
    public Pose2D getRotatedVelocityPos() {
        return veloPos.rotate(position.heading);
    }
}
