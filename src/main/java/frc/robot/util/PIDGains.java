package frc.robot.util;

public class PIDGains {
    private double _kP;
    private double _kI;
    private double _kD;

    public PIDGains(double p, double i, double d) {
        _kP = p;
        _kI = i;
        _kD = d;
    }

    public double getP() { return _kP; }
    public double getI() { return _kI; }
    public double getD() { return _kD; }

    public void setP(double p) { _kP = p; }
    public void setI(double i) { _kI = i; }
    public void setD(double d) { _kD = d; }
}