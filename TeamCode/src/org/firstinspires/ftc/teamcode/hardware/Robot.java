package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.hardware.interfaces.Subsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IMU;
import org.firstinspires.ftc.teamcode.hardware.subsystems.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.hardware.subsystems.ThreeWheelOdometry;

public class Robot {

    public DriveTrain driveTrain;
    public IMU imu;
    //public ThreeWheelOdometry localizer;
    public StandardTrackingWheelLocalizer localizer;

    public Subsystem[] subsystems = {};

    public HardwareMap hwMap;

    public enum OPMODE_TYPE {
        AUTO(0),
        TELEOP(1),
        DASHBOARD_TESTING(2);

        int v;

        OPMODE_TYPE(int v) {
            this.v = v;
        }

        public int getValue() {
            return v;
        }
    }

    public Robot (HardwareMap hwMap, OPMODE_TYPE type) {
        this.hwMap = hwMap;

        switch (type.getValue()) {
            case 0: // Auto
                driveTrain = new DriveTrain(this);
                imu = new IMU(this);
                localizer = new StandardTrackingWheelLocalizer(this);

                subsystems = new Subsystem[] {driveTrain, imu, localizer};
                break;
            case 1: // TeleOp
                driveTrain = new DriveTrain(this);
                imu = new IMU(this);
                subsystems = new Subsystem[] {imu, driveTrain};
                break;
            case 2: // Testing
                break;

        }
    }

    public void init() throws InterruptedException {
        for(Subsystem subsystem : subsystems) {
            subsystem.init();
        }
    }

    public void update() throws InterruptedException {
        for(Subsystem subsystem : subsystems) {
            subsystem.update();
        }
    }
}
