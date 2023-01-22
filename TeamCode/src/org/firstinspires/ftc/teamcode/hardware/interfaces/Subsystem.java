package org.firstinspires.ftc.teamcode.hardware.interfaces;

public interface Subsystem {
    boolean isInitDone = false;

    default boolean isInitDone() {
        return isInitDone;
    }

    void init() throws InterruptedException;

    void update() throws InterruptedException;
}