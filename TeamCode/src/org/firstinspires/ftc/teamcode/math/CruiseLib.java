package org.firstinspires.ftc.teamcode.math;

import com.acmerobotics.roadrunner.geometry.Vector2d;

public class CruiseLib {
    private CruiseLib(){}

    public static double Average(double first, double second){
        return (first+second)/2;
    }

    public static int Average(int first, int second){
        return (first+second)/2;
    }

    public static boolean isBetween(double sample, double min, double max){
        return min < sample && sample < max;
    }

    public static double limitValue(double val) {
        return limitValue(val, 1.0);
    }

    public static double limitValue(double val, double max) {
        if(val > Math.abs(max)) {
            return Math.abs(max);
        } else if(val < -Math.abs(max)) {
            return -Math.abs(max);
        } else {
            return val;
        }
    }

    public static double limitValue(double val, double max, double min) {
        if(val > max) {
            return max;
        } else if(val < min) {
            return min;
        } else {
            return val;
        }
    }

    public static double limitValue(double val, double negMin, double negMax,
                                    double posMin, double posMax){
        if(val>negMin && val<0)
            return negMin;
        else if(val<negMax && val<0)
            return negMax;
        else if(val<posMin && val>0)
            return posMin;
        else if(val>posMax)
            return posMax;
        else
            return val;
    }

    public static double angleWrap(double angle) {
        if (angle > 180){
            return angle - 360;
        }else if (angle < -180){
            return angle + 360;
        }
        return angle;
    }

    public static double angleWrapRad(double angle) {
        if (angle > Math.PI){
            return angle - Math.PI * 2;
        }else if (angle < -Math.PI){
            return angle + Math.PI * 2;
        }
        return angle;
    }

    public static int getSign(double input) {
        return (int)(input / Math.abs(input));
    }

    public static double squareMaintainSign(double val) {
        double output = val * val;

        //was originally negative
        if(val < 0) {
            output = -output;
        }

        return output;
    }

    public static double power3MaintainSign(double val){
        return val*val*val;
    }

    public static double calcLeftTankDrive(double x, double y) {
        return limitValue(y + x);
    }

    public static double calcRightTankDrive(double x, double y) {
        return limitValue(y - x);
    }

    public static double max(double a, double b, double c) {
        a = Math.abs(a);
        b = Math.abs(b);
        c = Math.abs(c);
        if(a > b && a > c) {
            return a;
        } else if(b > c) {
            return b;
        } else {
            return c;
        }
    }

    /**
     * Use Math.toRadians
     * @param deg Degree measure
     * @return Radian value
     */
    @Deprecated
    public static double degreesToRadians(double deg) {
        return (deg * Math.PI) / 180.0;
    }

    /**
     * Use Math.toDegrees
     * @param rad Radian measure
     * @return Degree value
     */
    @Deprecated
    public static double radianToDegrees(double rad) {
        return (rad / Math.PI) * 180.0;
    }

    /**
     * @param measured Measured Value
     * @param expected Expected Value
     * @param tol Tolerance
     * @return if the measured value is within a tolerance
     */
    public static boolean isWithinRange(double measured, double expected, double tol){
        return measured-Math.abs(tol)<expected && measured+Math.abs(tol)>expected;
    }

    public static double getDistanceBetweenTwoPosition2d(Vector2d pose1, Vector2d pose2){
        double x = (pose1.getX() - pose2.getX()) * (pose1.getX() - pose2.getX());
        double y = (pose1.getY() - pose2.getY()) * (pose1.getY() - pose2.getY());

        return Math.sqrt(x+y);
    }

    /**
     * @param x Number you want to round
     * @param digits Number of digits after zero
     * @return Rounded Number
     */
    public  static double round(double x, int digits){
        double num = 10 * digits;
        return ((int)(x * num))/num;
    }

    /**
     * @param array Array to print
     */
    private void printArray(String[] array){
        for (String anArray : array) {
            System.out.println(anArray);
        }
    }

    public class RunningAverage {
        private double[] inputs;
        private int index;
        double sum = 0;

        public RunningAverage(int size) {
            inputs = new double[size];
            index = 0;
        }

        public double calculate(double input) {
            index++;
            if(index == inputs.length) {
                index = 0;
            }
            inputs[index] = input;

            sum = 0;
            for(int i = 0; i < inputs.length; i++) {
                sum += inputs[i];
            }

            return sum / (inputs.length * 1.0);
        }

        public double get() {
            return sum / (inputs.length * 1.0);
        }
    }
}