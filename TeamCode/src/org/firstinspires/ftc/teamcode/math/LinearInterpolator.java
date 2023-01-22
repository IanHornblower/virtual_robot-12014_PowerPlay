package org.firstinspires.ftc.teamcode.math;

import com.acmerobotics.roadrunner.util.Angle;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import javax.swing.plaf.IconUIResource;

public class LinearInterpolator {
    double slope;
    public Point start, end;
    public LinearInterpolator(Point start, Point end) {
        this.start = start;
        this.end = end;
        slope = Curve.getShortestDistance(start.y, end.y) / Math.abs(end.x - start.x);
    }

    public double get(Point current) {
        return Math.signum(end.y - current.y) * slope * (current.x - end.x) + end.y;
    }

    public double getWeighted(Point current) {
       double startWeight = start.y * ((end.x - current.x)/(end.x - start.x));
       double endWeight = end.y * ((current.x - start.x)/(end.x - start.x));

       return startWeight + endWeight;
    }

    public double getUnwaited(Point current) {
        double div = end.x - start.x;
        double startWeight = start.y * (end.x - current.x);
        double endWeight = end.y * (current.x - start.x);

        return (startWeight + endWeight) / div;
    }

    public double getHeading(Point current) {
        return Angle.normDelta(end.y - start.y) * ((current.x - start.x)/(end.x - start.x));
    }

    public static double lerp(double current, Point start, Point end) {
        double slope = (end.y - start.y) / (end.x - start.x);

        return slope * (current - end.x) + end.y;
    }
}
