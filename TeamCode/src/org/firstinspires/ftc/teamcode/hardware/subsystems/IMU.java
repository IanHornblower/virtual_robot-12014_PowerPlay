package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.interfaces.Subsystem;
import org.firstinspires.ftc.teamcode.util.Timer;

public class IMU implements Subsystem {

    BNO055IMU imu;
    HardwareMap hwMap;
    Orientation angles;

    AxesOrder HubConfig = AxesOrder.ZYX;

    public float velocity = 0.0F;

    double previousHeading = 0.0;
    double accumulatedHeading = 0.0;

    double rawHeading = 0.0;
    double normalHeading = 0.0;

    double startHeading = 0.0;

    public IMU(Robot robot) {
        this.hwMap = robot.hwMap;
    }

    public void setStartHeading(double heading) {
        startHeading = heading;
    }

    private double correctAngle(double heading) {
        if(heading > Math.PI * 2.0) {
            heading -= 2.0 * Math.PI;
        }
        else if(heading < 0) {
            heading += 2.0 * Math.PI;
        }
        return heading;
    }

    private double getRawIMUHeading() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, HubConfig, AngleUnit.RADIANS);
        return -angles.firstAngle;
    }

    public double getHeadingInRadians() {
        return normalHeading;
    }

    public double getHeadingInDegrees() {
        return Math.toDegrees(normalHeading);
    }

    public double getRawHeadingInRadians() {
        return rawHeading;
    }

    public double getRawHeadingInDegrees() {
        return Math.toDegrees(rawHeading);
    }

    public double getAccumulatedHeadingInRadians() {
        return accumulatedHeading;
    }

    public double getAccumulatedHeadingInDegrees() {
        return Math.toDegrees(accumulatedHeading);
    }

    public void resetHeading(){
        previousHeading = rawHeading;
        accumulatedHeading = 0;
    }

    public static double getDelta(double a1, double a2) {
        double x1=Math.cos(a1), y1=Math.sin(a1), x2=Math.cos(a2), y2=Math.sin(a2);

        return Math.acos(x1*x2 + y1*y2);
    }

    Timer timer = new Timer();

    @Override
    public void init() throws InterruptedException {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;  // :)
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        timer.start();
    }

    @Override
    public void update() throws InterruptedException {
        rawHeading = getRawIMUHeading() + startHeading;
        normalHeading = correctAngle(rawHeading);

        double dHeading = rawHeading - previousHeading;

        velocity = (float) dHeading / (float) timer.currentSeconds();

        if (dHeading < -Math.PI) {
            dHeading += Math.PI * 2;
        }
        else if (dHeading >= Math.PI) {
            dHeading -= Math.PI * 2;
        }

        accumulatedHeading += dHeading;
        previousHeading = rawHeading;
    }
}
