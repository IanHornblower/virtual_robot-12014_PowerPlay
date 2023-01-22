package org.firstinspires.ftc.teamcode.control.AsymStuff;

import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.config.Config;

@Config
public class ControlConstants {
    public static PIDCoefficients distanceControl = new PIDCoefficients(0.05,0,0);
    public static SqrtCoefficients angleControl = new SqrtCoefficients(2.5, 0.0,0.0);
    public static SqrtCoefficients coneAngleControl = new SqrtCoefficients(0.0128, 0.05,0.1);
    public static MotionConstraint driveConstraintsNew = new MotionConstraint(65,
            -30,65);
    public static MotionConstraint turnConstraintsNew =
            new MotionConstraint(Math.toRadians(300),
                    Math.toRadians(100),
                    Math.toRadians(300));

    public static MotionConstraint coneTurnConstraints =
            new MotionConstraint(Math.toRadians(300),
                    Math.toRadians(100),
                    Math.toRadians(300));

}
