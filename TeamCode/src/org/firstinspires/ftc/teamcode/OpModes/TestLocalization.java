package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.ActionSystem.ActionSequence;
import org.firstinspires.ftc.teamcode.ActionSystem.ActionSequenceRunner;
import org.firstinspires.ftc.teamcode.ActionSystem.actions.*;
import org.firstinspires.ftc.teamcode.control.Trajectory;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.math.Pose2D;

import java.util.ArrayList;
import java.util.Arrays;

@TeleOp
public class TestLocalization extends LinearOpMode {
    @Override
    public void runOpMode() throws Exception {

        Robot robot = new Robot(hardwareMap, Robot.OPMODE_TYPE.AUTO);

        robot.localizer.setStartPosition(new Pose2D(0, 0, Math.toRadians(180)));
        robot.init();

        ActionSequence as = new ActionSequence();

        Trajectory trajectory = new Trajectory(new Pose2D(3.5, 24, 0));
        trajectory.add(new Point(3.5, 48));
        trajectory.add(new Pose2D(11, 58, Math.toRadians(150)));

        Trajectory toConeStack = new Trajectory(trajectory.end());
        toConeStack.add(new Point(5, 55));
        toConeStack.add(new Point(0, 50.5));
        toConeStack.add(new Point(-8, 50.5));
        //toConeStack.add(new Pose2D(-14, 50.5, Math.toRadians(90)));
        toConeStack.add(new Pose2D(-26, 50.5, Math.toRadians(90)));

        Trajectory toPole = new Trajectory(toConeStack.end());
        toPole.add(new Point(0, 51));
        toPole.add(trajectory.end());

        //as.addAction(new RunToPose(robot, new Pose2D(3.5, 24, Math.toRadians(180))));
        //as.addAction(new FollowPurePursuitTrajectory(robot, trajectory, 6, true)); //
        //as.addAction(new FollowPurePursuitTrajectory(robot, toConeStack, 6, false));
        //as.addAction(new FollowPurePursuitTrajectory(robot, toPole, 6, true));
        //as.addAction(new RunToPose(robot, new Pose2D(5, 50.5, Math.toRadians(150))));
        //as.addAction(new RunToPose(robot, new Pose2D(-24, 50.5, Math.toRadians(90))));
        //as.addAction(new RunToPose(robot, new Pose2D(5, 50.5, Math.toRadians(90    ))));
        //as.addAction(new RunToPose(robot, new Pose2D(-24, 50.5, Math.toRadians(90))));
        //as.addAction(new RunToPose(robot, new Pose2D(-5, 51, Math.toRadians(90))));


        ActionSequenceRunner asr = new ActionSequenceRunner(robot);

        asr.setActionSequence(as);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
           if(!asr.isComplete()) {
               asr.update();
               robot.update();
           }
           else {
               stop();
           }
            //robot.driveTrain.driveFieldCentric(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, Math.toRadians(0));
            //robot.update();

            telemetry.addData("cuurent aciton", asr.getCurrentAction());

            telemetry.addData("left", robot.localizer.getWheelPositions().get(0));
            telemetry.addData("right", robot.localizer.getWheelPositions().get(1));
            telemetry.addData("x", robot.localizer.getWheelPositions().get(2));

            telemetry.addData("Pos", robot.localizer.getPose().toString());
            telemetry.addData("velo Pos", robot.localizer.getRotatedVelocityPos().toString());
            telemetry.addData("acc", robot.localizer.getAccumulatedDistance());
            telemetry.update();
        }
    }
}
