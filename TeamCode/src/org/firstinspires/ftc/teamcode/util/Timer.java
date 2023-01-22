package org.firstinspires.ftc.teamcode.util;

public class Timer {

    private long startTime;
    private long previousTime;
    private long elapsedTime;
    boolean paused = false;

    public Timer() {
        startTime = 0;
        previousTime = 0;
        elapsedTime = 0;
    }

    public void addSeconds(double seconds) {
        startTime = startTime-(long)(seconds*1e+9);
    }
    
    public void removeSeconds(double seconds) {
        startTime = startTime+(long)(seconds*1e+9);
    }

    public void start() {
        startTime = (long) (System.nanoTime());
    }

    public void reset() {
        startTime = (long) (System.nanoTime());
    }

    public void pause() {
        paused = true;
    }

    public void resume() {
        paused = false;
    }

    public double currentMills() {
        if(!paused) {
            previousTime = System.nanoTime();
            return (System.nanoTime() - startTime + elapsedTime)*1e-6;

        }
        else {
            elapsedTime = previousTime - System.nanoTime();
            return (previousTime - startTime)*1e-6;
        }
    }

    public double currentSeconds() {
        if(!paused) {
            previousTime = System.nanoTime();
            return (System.nanoTime() - startTime + elapsedTime)*1e-9;

        }
        else {
            elapsedTime = previousTime - System.nanoTime();
            return (previousTime - startTime)*1e-9;
        }
    }
}
