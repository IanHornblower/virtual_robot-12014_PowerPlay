package org.firstinspires.ftc.teamcode.control.AsymStuff;


import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController;
import com.ThermalEquilibrium.homeostasis.Utils.MathUtils;
import com.ThermalEquilibrium.homeostasis.Utils.Vector;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.DoubleSupplier;

public class ConeTurnOnlyControl {
    protected double endGoalError = 1000;
    protected DoubleSupplier robotAngle;
    protected double previousReference = 100000000;
    ElapsedTime timer = new ElapsedTime();
    AsymmetricMotionProfile profile_n;

    SqrtControl angleController = new SqrtControl(ControlConstants.coneAngleControl);

    double trackingError = 0;

    public ConeTurnOnlyControl(DoubleSupplier robotAngle) {
        this.robotAngle = robotAngle;
    }

    /**
     * returns the wheel powers as a vector
     * @return 2 state vector, item 0 is left, item 1 is right
     */
    public Vector calculate() {
        regenerateProfile(robotAngle.getAsDouble(), 0);
        double profileState = profile_n.calculate(timer.seconds()).getX(); //profile.calculate(timer.seconds()).position;
        Vector output = new Vector(2);

        endGoalError = 0 - robotAngle.getAsDouble();
        trackingError = profileState - robotAngle.getAsDouble();
        double heading = -angleController.calculate(0,  endGoalError);

        double left = + heading;
        double right = - heading;

        output.set(left, 0);
        output.set(right, 1);

        return output;
    }

    public double getTrackingError() {
        return trackingError;
    }

    public double getEndGoalError() {
        return endGoalError;
    }

    public void setCoefficients(SqrtCoefficients coefficients) {
        angleController.setCoefficients(coefficients);
    }

    public void regenerateProfile(double reference, double state) {
        if (reference != previousReference) {
            profile_n = new AsymmetricMotionProfile(
                    state,
                    0,
                    ControlConstants.coneTurnConstraints);
            timer.reset();
        }
        previousReference = reference;
    }

}
