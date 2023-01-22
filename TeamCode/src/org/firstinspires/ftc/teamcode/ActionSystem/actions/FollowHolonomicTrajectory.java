package org.firstinspires.ftc.teamcode.ActionSystem.actions;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.Angle;
import org.firstinspires.ftc.teamcode.ActionSystem.Action;
import org.firstinspires.ftc.teamcode.control.AsymStuff.SqrtCoefficients;
import org.firstinspires.ftc.teamcode.control.AsymStuff.SqrtControl;
import org.firstinspires.ftc.teamcode.control.AsymStuff.TurnOnlyControl;
import org.firstinspires.ftc.teamcode.control.PositionController;
import org.firstinspires.ftc.teamcode.control.Trajectory;
import org.firstinspires.ftc.teamcode.hardware.DriveConstraints;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.interfaces.Subsystem;
import org.firstinspires.ftc.teamcode.math.Curve;
import org.firstinspires.ftc.teamcode.math.LinearInterpolator;
import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.util.Timer;

import javax.swing.*;
import java.util.ArrayList;
import java.util.Arrays;

public class FollowHolonomicTrajectory extends Action {

    Robot robot;
    Trajectory trajectory;
    Pose2D end;

    double timeout = 1e+9;
    Timer timer = new Timer();

    AngleController hController = new AngleController(new BasicPID(new PIDCoefficients(6,0,0)));
    double kP = 0.15;

    SqrtControl xController = new SqrtControl(new SqrtCoefficients(kP, 0.0, 0.0));
    SqrtControl yController = new SqrtControl(new SqrtCoefficients(kP, 0.0, 0.0));

    Pose2D robotPose;

    MotionProfile xProfile;
    MotionProfile yProfile;

    LinearInterpolator interpolator;
    Pose2D start;

    ArrayList<Pose2D> path;
    double[] lengths;
    int lastIndex;
    Pose2D lastPoint;
    Point relativePoint;

    public FollowHolonomicTrajectory(Robot robot, Trajectory trajectory) {
        this.robot = robot;
        this.trajectory = trajectory;
        end = trajectory.end();
    }

    public FollowHolonomicTrajectory(Robot robot, Trajectory trajectory, double timeout) {
        this.robot = robot;
        this.trajectory = trajectory;
        end = trajectory.end();
        this.timeout = timeout;
    }


    @Override
    public void startAction() {
        path = trajectory.getPath();
        robotPose = robot.localizer.getPose();
        start = path.get(path.size()-2);

        lengths = Trajectory.getSegmentLengths(path);

        interpolator = new LinearInterpolator(new Point(path.get(path.size()-2).x, path.get(path.size()-2).heading), new Point(end.x, end.heading));

        robot.localizer.resetAccumulatedDistance();

        lastIndex = path.size()-1;
        lastPoint = path.get(lastIndex);

        relativePoint = new Point(end.x - start.x, end.y - start.y);

        if(relativePoint.x >= relativePoint.y) {
            interpolator = new LinearInterpolator(new Point(start.x, 0), new Point(end.x, Angle.normDelta(path.get(1).heading - start.heading)));
        }
        else {
            interpolator = new LinearInterpolator(new Point(start.y, start.heading), new Point(end.y, Angle.normDelta(path.get(1).heading - start.heading)));
        }

        xProfile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(path.get(path.size()-2).x, 13, 0), new MotionState(end.x, 0, 0), 13, 30);
        yProfile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(path.get(path.size()-2).y, 13, 0), new MotionState(end.y, 0, 0), 13, 30);


        timer.start();
    }

    // Start at first point
    int index = 1;
    double lerpHeading;

    @Override
    public void runAction() throws InterruptedException {
        robotPose = robot.localizer.getPose();

        if(robot.localizer.getAccumulatedDistance() > lengths[index-1]) {
            robot.localizer.resetAccumulatedDistance();
            if(!(index >= lengths.length)) {
                index++;
            }

            if(relativePoint.x >= relativePoint.y) {
                interpolator = new LinearInterpolator(new Point(start.x, 0), new Point(end.x, Angle.normDelta(path.get(index).heading - start.heading)));
            }
            else {
                interpolator = new LinearInterpolator(new Point(start.y, start.heading), new Point(end.y, Angle.normDelta(path.get(index).heading - start.heading)));
            }
        }

        Point pointToFollow = path.get(index).toPoint();
        //Point error = pointToFollow.subtract(robotPose.toPoint());
        //Point power = error.scalar(kP);

        Point power = new Point(
                xController.calculate(pointToFollow.x, robotPose.x),
                yController.calculate(pointToFollow.y, robotPose.y)
        );


        if(relativePoint.x >= relativePoint.y) {
            lerpHeading = interpolator.getWeighted(new Point(robotPose.x, robotPose.heading));
        }
        else {
            lerpHeading = interpolator.getWeighted(new Point(robotPose.y, robotPose.heading));
        }

        robot.driveTrain.driveFieldCentric(power.x, power.y, -hController.calculate(lerpHeading, robotPose.heading), Math.toRadians(0));

        isComplete =
                        Math.abs(end.x - robotPose.x) < 3 &&
                        Math.abs(end.y - robotPose.y) < 3 &&
                        Math.abs(Curve.getShortestDistance(end.heading, robotPose.heading)) < Math.toRadians(5) &&
                        Math.abs(robot.localizer.getRawVelocityPos().heading) < Math.toRadians(12) &&
                        Math.abs(robot.localizer.getRawVelocityPos().x) < 2 &&
                        Math.abs(robot.localizer.getRawVelocityPos().y) < 2 ||
                                timer.currentSeconds() > timeout;
    }

    @Override
    public void stopAction() {
        robot.driveTrain.stopDriveTrain();
    }
}
