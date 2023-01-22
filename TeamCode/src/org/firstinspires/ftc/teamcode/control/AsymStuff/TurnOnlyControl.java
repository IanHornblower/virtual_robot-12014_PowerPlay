package org.firstinspires.ftc.teamcode.control.AsymStuff;


import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController;
import com.ThermalEquilibrium.homeostasis.Utils.MathUtils;
import com.ThermalEquilibrium.homeostasis.Utils.Vector;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.DoubleSupplier;

public class TurnOnlyControl {
    protected double headingReference;
    protected double endGoalError = 1000;
    protected DoubleSupplier robotAngle;
    protected double previousReference = 100000000;
    ElapsedTime timer = new ElapsedTime();
    AsymmetricMotionProfile profile_n;

    SqrtControl angleController = new SqrtControl(ControlConstants.angleControl);
    AngleController angleControl = new AngleController(angleController);

    double trackingError = 0;

    public TurnOnlyControl(DoubleSupplier robotAngle, double headingReference) {
        this.robotAngle = robotAngle;
        this.headingReference = headingReference;
    }

    /**
     * returns the wheel powers as a vector
     * @return 2 state vector, item 0 is left, item 1 is right
     */
    public Vector calculate() {

        regenerateProfile(robotAngle.getAsDouble(), headingReference);
        double profileState = profile_n.calculate(timer.seconds()).getX(); //profile.calculate(timer.seconds()).position;
        Vector output = new Vector(2);
        endGoalError = MathUtils.normalizedHeadingError(headingReference, robotAngle.getAsDouble());
        trackingError = MathUtils.normalizedHeadingError(profileState, robotAngle.getAsDouble());
        double heading = -angleControl.calculate(0,  endGoalError);

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

    public void setHeadingReference(double reference) {
        this.headingReference = reference;
    }

    public void setCoefficients(SqrtCoefficients coefficients) {
        angleController.setCoefficients(coefficients);
    }

    public void regenerateProfile(double reference, double state) {
        if (reference != previousReference) {
            profile_n = new AsymmetricMotionProfile(
                    MathUtils.normalizedHeadingError(headingReference, state),
                    0,
                    ControlConstants.turnConstraintsNew);
            timer.reset();
        }
        previousReference = reference;
    }

}
