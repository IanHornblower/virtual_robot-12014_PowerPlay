package org.firstinspires.ftc.teamcode.MotionProfiles;

public class NonStandardAysemetricMotionProfile extends MotionProfile {

    double slope;
    double initialLength;

    public NonStandardAysemetricMotionProfile(double distance, double minVelocity, double maxVelocity, double slope, double initialLength) {
        this.distance = distance;
        this.minVelocity = minVelocity;
        this.maxVelocity = maxVelocity;
        this.slope = slope;
        this.initialLength = initialLength;
    }

    public double normalizeTime(double time, double duration, double max) {
        return max * (time/duration);
    }

    public double getAverageVelocity() { // Less Accurate but within a reasonable approximation, and a lot faster
        double distanceOfSeg2 = (minVelocity - maxVelocity)/-slope;

        double seg1 = initialLength * maxVelocity;
        double seg2 = ((maxVelocity + minVelocity) / 2) * (distanceOfSeg2 / maxVelocity);
        double seg3 = (maxVelocity - distanceOfSeg2 + seg1)/maxVelocity * minVelocity;

        return seg1 + seg2 + seg3;
    }

    public double segment1(double currentNormalizedTime, double max, double length) {
        double veloctiy = 0;
        if(currentNormalizedTime <= max * length && currentNormalizedTime >= 0 && currentNormalizedTime <= max) {
            veloctiy = max;
        }
        return veloctiy;
    }

    public double segment2(double currentNormalizedTime, double min, double max, double length, double slope) {
        double veloctiy = 0;
        if(currentNormalizedTime <= max) {
            veloctiy = -(currentNormalizedTime - (max * length)) * slope + max;
        }

        if(veloctiy  <= min || veloctiy  >= max) {
            veloctiy  = 0;
        }

        return veloctiy;
    }

    public double segment3(double currentTime, double currentNormalizedTime, double min, double max, double length, double slope) {
        double veloctiy = 0;
        if(currentNormalizedTime >= ((max - min) * 1 / slope) + (max * length) && currentNormalizedTime <= max || currentTime >= duration) {
            veloctiy = min;
        }

        return veloctiy;
    }

    @Override
    public void create() {
        averageVelocity = getAverageVelocity();
        duration = distance/averageVelocity;
    }

    @Override
    public double calculate(double currentTime) {
        double normalizedTime = normalizeTime(currentTime, duration, maxVelocity);

        return segment1(normalizedTime, maxVelocity, duration) +
               segment2(normalizedTime, minVelocity, maxVelocity, duration, slope) +
               segment3(currentTime, normalizedTime, minVelocity, maxVelocity, duration, slope);
    }
}
