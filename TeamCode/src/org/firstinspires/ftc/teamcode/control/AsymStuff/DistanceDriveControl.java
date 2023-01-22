package org.firstinspires.ftc.teamcode.control.AsymStuff;

import android.os.Build;
import androidx.annotation.RequiresApi;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Utils.Vector;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.function.DoubleSupplier;

public class DistanceDriveControl  {



    BasicPID distanceControl = new BasicPID(ControlConstants.distanceControl);
    TurnOnlyControl turnControl;
    double trackingError = 0;
    double endPoseError = 0;
    double previousReference = 0;
    AsymmetricMotionProfile profile_n = new AsymmetricMotionProfile(10,10,ControlConstants.driveConstraintsNew);
    ElapsedTime timer = new ElapsedTime();

    public DistanceDriveControl(DoubleSupplier robotAngle, double headingReference) {
        turnControl = new TurnOnlyControl(robotAngle, headingReference);
    }

    /**
     * returns the wheel powers as a vector
     * @param reference target distance
     * @param state current traveled distance
     * @return 2 state vector, item 0 is left, item 1 is right
     */
    @RequiresApi(api = Build.VERSION_CODES.N)
    public Vector calculate(double reference, double state) {

        double direction = Math.signum(reference);
        if (direction == 0) direction = 1;
        reference = Math.abs(reference);
        regenerateProfile(reference,state);
        double reference_p = profile_n.calculate(timer.seconds()).getX();//profile.calculate(timer.seconds()).position;

        trackingError = reference_p - state;
        endPoseError = reference - state;

        Vector output = new Vector(2);

        double forward = distanceControl.calculate(reference_p,state) * direction;
        Vector turn = turnControl.calculate();
        double scalar = Math.cos(Range.clip(turnControl.getEndGoalError(),-Math.PI / 2, Math.PI / 2));
        output.set(forward * scalar, 0);
        output.set(forward * scalar, 1);

        try {
            return output.add(turn);
        } catch (Exception e) {
            e.printStackTrace();
            return output;
        }
    }

    public double getTrackingError() {
        return trackingError;
    }

    public double endPoseError() {
        return endPoseError;
    }

    public void setHeadingReference(double reference) {
        this.turnControl.setHeadingReference(reference);
    }

    public void setTurnCoefficients(SqrtCoefficients coefficients) {
        turnControl.setCoefficients(coefficients);
    }


    public void regenerateProfile(double reference, double state) {
        if (reference != previousReference) {
            profile_n = new AsymmetricMotionProfile(state, reference, ControlConstants.driveConstraintsNew);
            timer.reset();
        }
        previousReference = reference;
    }


}
