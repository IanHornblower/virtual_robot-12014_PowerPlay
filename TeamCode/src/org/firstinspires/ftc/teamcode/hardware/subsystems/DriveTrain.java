package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.ThermalEquilibrium.homeostasis.Utils.Vector;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.teamcode.hardware.DriveConstraints;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.interfaces.Subsystem;
import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.util.Encoder;

public class DriveTrain implements Subsystem {
    public DcMotorEx frontLeft, frontRight, backLeft, backRight;
    public Encoder left, right, lateral;

    Robot robot;

    double fl, fr, bl, br;

    public DcMotorEx[] motors;
    public Encoder[] deadWheels;

    public ColorSensor cl;
    public ColorSensor cr;

    public static double trackWidth = 15;
    public static double wheelBase = 12;
    public static double lateralMultiplier = 0;

    HardwareMap hwMap;

    public DriveTrain(Robot robot) {
        this.robot = robot;
        this.hwMap = robot.hwMap;

        frontLeft = hwMap.get(DcMotorEx.class, "front_left_motor");
        frontRight = hwMap.get(DcMotorEx.class, "front_right_motor");
        backLeft = hwMap.get(DcMotorEx.class, "back_left_motor");
        backRight = hwMap.get(DcMotorEx.class, "back_right_motor");

        left = new Encoder(hwMap.get(DcMotorEx.class, "enc_left"));
        right = new Encoder(hwMap.get(DcMotorEx.class, "enc_right"));
        lateral = new Encoder(hwMap.get(DcMotorEx.class, "enc_x"));

        motors = new DcMotorEx[] {frontLeft, frontRight, backLeft, backRight};
        deadWheels = new Encoder[] {left, right, lateral};
    }

    public HardwareMap getHardwareMap() {
        return hwMap;
    }

    public void resetEncoders() {
        for(DcMotorEx motors:motors) {
            motors.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motors.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void setWeightedDrivePower(double x, double y, double heading) {
        Pose2D vel = new Pose2D(x, y, heading);

        if (Math.abs(x) + Math.abs(y)
                + Math.abs(heading) > 1) {
            // re-normalize the powers according to the weights
            double denom = DriveConstraints.VX_WEIGHT * Math.abs(x)
                    + DriveConstraints.VY_WEIGHT * Math.abs(y)
                    + DriveConstraints.TURN_WEIGHT * Math.abs(heading);

            vel = new Pose2D(
                    DriveConstraints.VX_WEIGHT * x,
                    DriveConstraints.VY_WEIGHT * y,
                    DriveConstraints.TURN_WEIGHT * heading
            ).div(denom);
        }

        setMotorPowers(vel.x, vel.y, vel.heading);
    }

    public void setWeightedDrivePower(Pose2D pose) {
        setWeightedDrivePower(pose.x, pose.y, pose.heading);
    }

    public void setMotorPowers(double x, double y, double t) {
        double k = (DriveConstraints.dt_trackWidth + DriveConstraints.wheelBase) / 2.0;

        double power = Math.hypot(x, y) * Math.sqrt(2.0); // sqrt(2) == 2 * sin/cos(45)
        double angle = Math.atan2(y, x) - Math.PI / 4.0;

        double fl = power * Math.cos(angle) + Math.toRadians(k * t) * (DriveConstraints.maxAngularVelocity / DriveConstraints.maxVelocity);
        double bl = power * Math.sin(angle) + Math.toRadians(k * t) * (DriveConstraints.maxAngularVelocity / DriveConstraints.maxVelocity);
        double fr = power * Math.sin(angle) - Math.toRadians(k * t) * (DriveConstraints.maxAngularVelocity / DriveConstraints.maxVelocity);
        double br = power * Math.cos(angle) - Math.toRadians(k * t) * (DriveConstraints.maxAngularVelocity / DriveConstraints.maxVelocity);

        setMotorPowers(fl, fr, bl, br);
    }


    public void driveFieldCentric(double x, double y, double h) {
        Point power = new Point(x, y).rotate(robot.localizer.getPose().heading);
        setMotorPowers(power.x, power.y, h);
    }

    public void driveFieldCentric(double x, double y, double h, double modTheta) {
        Point power = new Point(x, y).rotate(modTheta - robot.localizer.getPose().heading);
        setMotorPowers(power.x, power.y, h);
    }

    public void setMotorPowers(double fl, double fr, double bl, double br) {
        this.fl = fl;
        this.fr = fr;
        this.bl = bl;
        this.br = br;
    }

    public void setMotorPowers(Vector vector) {
        setMotorPowers(vector.get(0), vector.get(1));
    }

    public void setMotorPowers(double left, double right) {
        setMotorPowers(left, right, left, right);
    }

    public void stopDriveTrain() {
        setMotorPowers(0, 0, 0, 0);
    }

    @Override
    public void init() throws InterruptedException {
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        for(DcMotorEx m : motors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        resetEncoders();
    }

    @Override
    public void update() throws InterruptedException {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }
}
