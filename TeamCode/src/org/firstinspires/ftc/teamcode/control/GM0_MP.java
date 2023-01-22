package org.firstinspires.ftc.teamcode.control;

import org.firstinspires.ftc.teamcode.hardware.Robot;

public class GM0_MP {

    double MAXIMUM_SPEED = 10;
    double MAX_ACCELERATION = 10;

    double position_error;
    double direction_multiplier;
    double output_velocity;
    double output_acceleration;
    double target_position;
    double previous_time = 0;

    public GM0_MP (double target_position) {
        this.target_position = target_position;
    }

    public double[] calcuate(double current_position, double current_velocity, double current_time) {
        position_error = target_position - current_position;

        direction_multiplier = 1;

        if (position_error < 0) {
            direction_multiplier = -1;
        }
        if (MAXIMUM_SPEED > Math.abs(current_velocity)) {
            output_velocity = current_velocity + direction_multiplier * MAX_ACCELERATION * (current_time - previous_time);
            output_acceleration = MAX_ACCELERATION;
        }


        else {
            output_velocity = MAXIMUM_SPEED;
            output_acceleration = 0;
            }

        if (position_error <= (output_velocity * output_velocity) / (2 * MAX_ACCELERATION)) {
            output_velocity = current_velocity - direction_multiplier * MAX_ACCELERATION * (current_time - previous_time);
            output_acceleration = -MAX_ACCELERATION;
        }

        previous_time = current_time;

        return new double[] {position_error, output_velocity, output_acceleration};
    }
}
