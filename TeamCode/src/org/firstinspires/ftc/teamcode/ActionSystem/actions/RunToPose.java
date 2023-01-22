package org.firstinspires.ftc.teamcode.ActionSystem.actions;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.Angle;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.ActionSystem.Action;
import org.firstinspires.ftc.teamcode.control.PositionController;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.math.Curve;
import org.firstinspires.ftc.teamcode.math.LinearInterpolator;
import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.util.Timer;

public class RunToPose extends Action {

    Robot robot;
    Pose2D end;

    Timer timer = new Timer();

    PositionController xController = new PositionController(0.15, 1/50.0);
    PositionController yController = new PositionController(0.15, 1/50.0);

    AngleController hController = new AngleController(new BasicPID(new PIDCoefficients(5,0,0)));

    Pose2D robotPose;

    MotionProfile xProfile;
    MotionProfile yProfile;

    double startVel = 0, maxVel = 13, maxAccel = 30;

    double timeout = 1e+9;

    public RunToPose(Robot robot, Pose2D end) {
        this.robot = robot;
        this.end = end;
    }

    public RunToPose(Robot robot, Pose2D end, double timeout) {
        this.robot = robot;
        this.end = end;
        this.timeout = timeout;
    }

    public RunToPose(Robot robot, Pose2D end, double maxVel, double maxAccel) {
        this.robot = robot;
        this.end = end;
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
    }

    public RunToPose(Robot robot, Pose2D end, double startVel, double maxVel, double maxAccel) {
        this.robot = robot;
        this.end = end;
        this.startVel = startVel;
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
    }

    public RunToPose(Robot robot, Pose2D end, double startVel, double maxVel, double maxAccel, double kP, double kV, double hP) {
        this.robot = robot;
        this.end = end;

        this.startVel = startVel;
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;

        xController = new PositionController(kP, kV);
        yController = new PositionController(kP, kV);

        hController = new AngleController(new BasicPID(new PIDCoefficients(hP, 0, 0)));
    }

    LinearInterpolator interpolator;

    Pose2D start;
    Point relativePoint;

    @Override
    public void startAction() {
        robotPose = robot.localizer.getPose();
        start = robotPose;

        //end.heading -= Math.toRadians(180);

        xProfile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(robotPose.x, startVel, 0), new MotionState(end.x, 0, 0), maxVel, maxAccel);
        yProfile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(robotPose.y, startVel, 0), new MotionState(end.y, 0, 0), maxVel, maxAccel);
        //hProfile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(robotPose.heading, 0, 0), new MotionState(end.heading, 0, 0), Math.toRadians(100), Math.toRadians(180));

        relativePoint = new Point(end.x - start.x, end.y - start.y);

        if(relativePoint.x >= relativePoint.y) {
            interpolator = new LinearInterpolator(new Point(start.x, 0), new Point(end.x, AngleUnit.normalizeRadians(end.heading - start.heading)));
        }
        else {
            interpolator = new LinearInterpolator(new Point(start.y, start.heading), new Point(end.y, AngleUnit.normalizeRadians(end.heading - start.heading)));
        }


        timer.start();
    }

    double lerpHeading;

    @Override
    public void runAction() throws InterruptedException {
        robotPose = robot.localizer.getPose();

        MotionState xMotionState = xProfile.get(timer.currentSeconds());
        MotionState yMotionState = yProfile.get(timer.currentSeconds());
        //MotionState hMotionState = hProfile.get(timer.currentSeconds());

        xController.setTargets(xMotionState.getX(), xMotionState.getV());
        yController.setTargets(yMotionState.getX(), yMotionState.getV());

        if(relativePoint.x >= relativePoint.y) {
            lerpHeading = interpolator.getWeighted(new Point(robotPose.x, robotPose.heading));
        }
        else {
            lerpHeading = interpolator.getWeighted(new Point(robotPose.y, robotPose.heading));
        }

        Point xyPower = new Point(
                xController.calculate(
                        robotPose.x, robot.localizer.getRotatedVelocityPos().x),
                yController.calculate(
                        robotPose.y, robot.localizer.getRotatedVelocityPos().y));

        robot.driveTrain.driveFieldCentric(xyPower.x, xyPower.y, -hController.calculate(end.heading, robotPose.heading), Math.toRadians(0));

        System.out.println(Math.toDegrees(lerpHeading));

        isComplete =
                Math.abs(end.x - robotPose.x) < 1 &&
                Math.abs(end.y - robotPose.y) < 1 &&
                //Math.abs(Curve.getShortestDistance(end.heading, robotPose.heading)) < Math.toRadians(5) &&
                       // Math.abs(robot.localizer.getRawVelocityPos().heading) < Math.toRadians(15) &&
               Math.abs(robot.localizer.getRawVelocityPos().x) < 1  &&
               Math.abs(robot.localizer.getRawVelocityPos().y) < 1 ||
                timer.currentSeconds() > timeout;

        //robot.update();
    }

    @Override
    public void stopAction() {
        robot.driveTrain.stopDriveTrain();
    }
}
