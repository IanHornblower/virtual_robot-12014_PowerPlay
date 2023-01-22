package org.firstinspires.ftc.teamcode.control.AsymStuff;

public class SqrtCoefficients {

    public double K;
    public double Kd;
    public double H;

    /**
     * Sqrt feedback controller
     *
     * Take a look at: https://www.desmos.com/calculator/qiqimyikle before use
     *
     * @param K 'proportional' constant
     * @param H hysteresis or minimum power
     *
     */
    public SqrtCoefficients(double K, double Kd, double H) {
        this.K = K;
        this.Kd = Kd;
        this.H = H;
    }


    public double getKd() {
        return Kd;
    }

    public double getH() {
        return H;
    }

    public double getK() {
        return K;
    }
}