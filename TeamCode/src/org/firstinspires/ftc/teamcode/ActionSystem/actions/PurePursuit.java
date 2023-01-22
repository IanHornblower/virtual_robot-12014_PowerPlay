package org.firstinspires.ftc.teamcode.ActionSystem.actions;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.ActionSystem.Action;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.math.CruiseLib;
import org.firstinspires.ftc.teamcode.math.Curve;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.math.Pose2D;

import java.util.ArrayList;

import static java.lang.Math.*;

public class PurePursuit extends Action {

    Robot robot;
    ArrayList<Pose> path;
    public static double divider = 8;

    double maxSpeed = 0, timeout = -1, endTheta = 1000;
    ElapsedTime timer = new ElapsedTime();

    Pose2D tolerance = new Pose2D(2, 2, Math.toRadians(3));
    int step = 1;

    boolean overshot = false;

    public PurePursuit(Robot robot, ArrayList<Pose> path) {
        this.robot = robot;
        this.path = path;
    }

    public PurePursuit(Robot robot, ArrayList<Pose> path, double endTheta) {
        this.robot = robot;
        this.path = path;
        this.endTheta = endTheta;
    }

    public PurePursuit(Robot robot, ArrayList<Pose> path, double endTheta, double timeout) {
        this.robot = robot;
        this.path = path;
        this.timeout = timeout;
        this.endTheta = endTheta;
    }

    public PurePursuit(Robot robot, ArrayList<Pose> path, Pose2D tolerance) {
        this.robot = robot;
        this.path = path;
        this.tolerance = tolerance;
    }

    public PurePursuit(Robot robot, ArrayList<Pose> path, Pose2D tolerance, double timeout) {
        this.robot = robot;
        this.path = path;
        this.timeout = timeout;
        this.tolerance = tolerance;
    }

    public PurePursuit(Robot robot, ArrayList<Pose> path, Pose2D tolerance, double endTheta, double timeout) {
        this.robot = robot;
        this.path = path;
        this.timeout = timeout;
        this.tolerance = tolerance;
        this.endTheta = endTheta;
    }


    @Override
    public void startAction() {
        Pose2D pose = robot.localizer.getPose();
        path.add(0, new Pose(pose.x, pose.y, pose.getHeading()));

        timer.reset();
    }

    int count;
    @Override
    public void runAction() throws InterruptedException {
            Pose2D pose = robot.localizer.getPose();

            if (path.size() - 1 > step) {
                if (RobotToLine(path.get(step), path.get(step + 1), pose, path.get(step).radius).x != 10000) {
                    step += 1;
                }
            }

            Pose point = RobotToLine(path.get(step - 1), path.get(step), pose, path.get(step).radius);

            if (point.x == 10000) {
                point = path.get(step);
            }


            // Speed
            maxSpeed = path.get(step).maxSpeed;
            double xSpeed = 1, ySpeed = 1;

            Pose error = Pose.getError(Pose.Pose2dToPose(robot.localizer.getPose()),
                    path.get(path.size() - 1));

            if (((step == path.size() - 1) && abs(sqrt(pow(error.x, 2) + pow(error.y, 2))) < path.get(step).radius) || overshot) {
                double x = CruiseLib.limitValue(((point.x - pose.x) / path.get(step).radius), 0, -1, 0, 1);
                x *= CruiseLib.limitValue((abs(point.x - pose.x) / divider), 0, -1, 0, 1);
                x = CruiseLib.limitValue(-x, -0.15, -maxSpeed, 0.15, maxSpeed);

                double y = CruiseLib.limitValue(((point.y - pose.y) / path.get(step).radius), 0, -1, 0, 1);
                y *= CruiseLib.limitValue((abs(point.y - pose.y) / divider), 0, -1, 0, 1);
                y = CruiseLib.limitValue(y, -0.15, -maxSpeed, 0.15, maxSpeed);

                xSpeed = x * Math.cos(pose.getHeading()) - y * Math.sin(pose.getHeading());
                ySpeed = y * Math.cos(pose.getHeading()) + x * Math.sin(pose.getHeading());

                overshot = true;
            } else {
                double relativeDist = sqrt(pow(point.x - pose.x, 2) + pow(point.y - pose.y, 2));

                double speed = Range.clip((abs(relativeDist) / path.get(step).radius), 0, 1);
                speed *= Range.clip((abs(relativeDist) / ((divider / 10) * path.get(step).radius)), 0, 1);
                speed = CruiseLib.limitValue(speed, -0.1, -maxSpeed, 0.1, maxSpeed);

                xSpeed = speed * Math.sin(path.get(step).theta);
                ySpeed = speed * Math.cos(path.get(step).theta);
            }

            //Turn
            double relativePointAngle;

            if (((step == path.size() - 1) && abs(sqrt(pow(error.x, 2) + pow(error.y, 2))) < path.get(step).radius) || overshot) {
                double angleToPoint;
                if (endTheta == 1000) {
                    angleToPoint = atan2(path.get(path.size() - 1).x - path.get(path.size() - 2).x,
                            path.get(path.size() - 1).y - path.get(path.size() - 2).y) + (path.get(step).theta);
                } else {
                    angleToPoint = endTheta;
                }
                relativePointAngle = CruiseLib.angleWrapRad(angleToPoint + pose.getHeading());

                overshot = true;
            } else {
                double angleToPoint = atan2(point.x - pose.x, point.y - pose.y) + (path.get(step).theta);
                relativePointAngle = CruiseLib.angleWrapRad(angleToPoint + pose.getHeading());
            }

            double turnSpeed = CruiseLib.limitValue(relativePointAngle / Math.toRadians(70.0),
                    0, -path.get(step).turnSpeed, 0, path.get(step).turnSpeed);

            xSpeed = CruiseLib.limitValue(xSpeed, 1 - abs(turnSpeed * 1.5));
            ySpeed = CruiseLib.limitValue(ySpeed, 1 - abs(turnSpeed * 1.5));

            robot.driveTrain.driveFieldCentric(xSpeed, ySpeed, turnSpeed);

        isComplete =
                Math.abs(error.x) < 2 &&
                        Math.abs(error.y) < 2 &&
                        Math.abs(error.theta) < Math.toRadians(5) &&
                        //Math.abs(robot.localizer.getRawVelocityPos().heading) < Math.toRadians(15) &&
                        //Math.abs(robot.localizer.getRawVelocityPos().x) < 1  &&
                        //Math.abs(robot.localizer.getRawVelocityPos().y) < 1  &&
                        timeout != -1 &&
                        timer.milliseconds() > timeout;
        ;
    }

    @Override
    public void stopAction() {
        robot.driveTrain.stopDriveTrain();
    }


    Pose RobotToLine (Pose point1, Pose point2, Pose2D pose, double radius) {
            double xDiff = point2.x - point1.x;
            if (xDiff <= 0.1)
                xDiff = 0.1;

            double slope = (point2.y - point1.y) / xDiff;
            double yIntercept = point2.y - (slope * point2.x);

            double a = 1 + pow(slope, 2);
            double b = ((slope * yIntercept) * 2) + (-pose.x * 2) + ((-pose.y * 2) * slope);
            double c = pow(yIntercept, 2) + ((-pose.y * 2) * yIntercept) + (-pow(radius, 2) + pow(pose.x, 2) + pow(pose.y, 2));

            double d = pow(b, 2) - (4 * a * c);

            if (d >= 0) {
                double sol1 = (-b - sqrt(d)) / (2 * a);
                double sol2 = (-b + sqrt(d)) / (2 * a);

                double y1 = slope * sol1 + yIntercept;
                double y2 = slope * sol2 + yIntercept;

                if (y1 - point1.y < 0 && point2.y - y1 > 0)
                    if (y2 - point1.y < 0 && point2.y - y2 > 0)
                        return new Pose(10000, 10000, 10000);

                if (sol1 - point1.x < 0 && point2.x - sol1 > 0)
                    if (sol2 - point1.x < 0 && point2.x - sol2 > 0)
                        return new Pose(10000, 10000, 10000);

                double error1 = abs(point2.x - sol1) + abs(point2.y - y1);
                double error2 = abs(point2.x - sol2) + abs(point2.y - y2);

                Pose follow = new Pose(sol1, y1, 0);
                if (error1 > error2)
                    follow = new Pose(sol2, y2, 0);

                //            follow = Pose.limitPoseInLine(follow, point1, point2);

                if (sqrt(pow(point2.x - pose.x, 2) + pow(point2.y - pose.y, 2)) < radius) {
                    follow = point2;
                }

                return follow;
            } else {
                return new Pose(10000, 10000, 10000);
            }
    }
}
