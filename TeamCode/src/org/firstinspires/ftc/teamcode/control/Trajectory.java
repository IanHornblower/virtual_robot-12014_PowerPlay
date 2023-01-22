package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.math.Pose2D;

import java.util.ArrayList;

public class Trajectory extends LinearOpMode {
    ArrayList<Pose2D> path = new ArrayList<>();

    public enum controlType {FIELD, ROBOT}

    public Trajectory(Pose2D start) {
        path.add(start);
    }

    public Trajectory(ArrayList<Pose2D> path) {
        this.path = path;
    }

    public ArrayList<Pose2D> getPath() {
        return path;
    }

    public void add(Pose2D pose) {
        path.add(pose);
    }

    public void add(Point point) {
        path.add(new Pose2D(point, 0));
    }

    public Pose2D end() {
        return path.get(path.size()-1);
    }

    public static double[] getSegmentLengths(ArrayList<Pose2D> path) {
        double[] lengths = new double[path.size()-1];
        for(int i = 0; i < path.size()-1; i++) {
            lengths[i] = path.get(i).getDistanceFrom(path.get(i+1));
        }

        return lengths;
    }

    public static double[] getSegmentLengthsFromIndex(ArrayList<Pose2D> path, int index) {
        double[] lengths = new double[path.size()-1];
        for(int i = index - 1; i < path.size()-1; i++) {
            lengths[i] = path.get(i).getDistanceFrom(path.get(i+1));
        }

        return lengths;
    }

    public void forward(double distance, controlType style) {
        Pose2D previous = path.get(path.size()-1);
        switch (style) {
            default:
                Pose2D point;
            case FIELD:
                point = new Pose2D(path.get(path.size()-1).x, path.get(path.size()-1).y+distance, path.get(path.size()-1).heading);
                path.add(point);
                break;
            case ROBOT:
                double x = distance * Math.cos(previous.heading);
                double y = distance * Math.sin(previous.heading);
                point = new Pose2D(previous.x + x, previous.y + y, previous.heading);
                path.add(point);
                break;
        }
    }

    public void backward(double distance, controlType style) {
        Pose2D previous = path.get(path.size()-1);
        distance *= -1;
        switch (style) {
            default:
                Pose2D point;
            case FIELD:
                point = new Pose2D(previous.x, previous.y-distance, previous.heading);
                path.add(point);
                break;
            case ROBOT:
                double x = distance * Math.cos(previous.heading);
                double y = distance * Math.sin(previous.heading);
                point = new Pose2D(previous.x + x, previous.y + y, previous.heading);
                path.add(point);
                break;
        }
    }

    public void right(double distance, controlType style) {
        Pose2D previous = path.get(path.size()-1);
        switch (style) {
            default:
                Pose2D point;
            case FIELD:
                point = new Pose2D(previous.x + distance, previous.y, previous.heading);
                path.add(point);
                break;
            case ROBOT:
                double x = distance * Math.cos(previous.heading + Math.PI/2.0);
                double y = distance * Math.sin(previous.heading + Math.PI/2.0);
                point = new Pose2D(previous.x + x, previous.y + y, previous.heading);
                path.add(point);
                break;
        }
    }

    public void left(double distance, controlType style) {
        Pose2D previous = path.get(path.size()-1);
        switch (style) {
            default:
                Pose2D point;
            case FIELD:
                point = new Pose2D(previous.x, previous.y+distance, previous.heading);
                path.add(point);
                break;
            case ROBOT:
                double x = distance * Math.cos(previous.heading - Math.PI/2.0);
                double y = distance * Math.sin(previous.heading - Math.PI/2.0);
                point = new Pose2D(previous.x + x, previous.y + y, previous.heading);
                path.add(point);
                break;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
