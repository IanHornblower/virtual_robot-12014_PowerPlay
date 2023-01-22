package org.firstinspires.ftc.teamcode.math;

import static org.firstinspires.ftc.teamcode.util.MathUtil.roundPlaces;

public class Point {

    public double x, y;

    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Point invertPoint() {
        double tempX = x, tempY = y;
        x = tempY;
        y = tempX;
        return new Point(x,y);
    }

    public void scalePose(double scaleFactor) {
        x *= scaleFactor;
        y *= scaleFactor;
    }

    public void invertX() {
        x *= -1;
    }

    public void invertY() {
        y *= -1;
    }

    public Point add(Point point) {
        return new Point(x+point.x, y+point.y);
    }

    public Point add(double value) {
        return new Point(x + value, y + value);
    }

    public Point subtract(Point point) {
        return new Point(x-point.x, y-point.y);
    }

    public Point subtract(double value) {
        return new Point(x - value, y - value);
    }

    public Point scalar(double scaleFactor) {
        return new Point(x * scaleFactor, y * scaleFactor);
    }

    public Point div(double n) {
        return new Point(x/n, y/n);
    }

    public double dot(Point other) {
        return x * other.x + y * other.y;
    }

    public double hypot() {
        return Math.hypot(x, y);
    }

    public double atan2() { // Inverted and Negated Y
        return Math.atan2(x, y);
    }

    public Point rotate(double angleRadians) {
        double cosA = Math.cos(angleRadians);
        double sinA = Math.sin(angleRadians);

        double x = this.x * cosA - this.y * sinA;
        double y = this.x * sinA + this.y * cosA;

        return new Point(x, y);
    }

    public String toString() {
        return String.format(
                roundPlaces(x, 1) +
                        " " + roundPlaces(y, 1));
    }
}