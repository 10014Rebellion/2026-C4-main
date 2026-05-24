package frc.lib.units;

import edu.wpi.first.math.geometry.Rotation2d;

public class Rot2d {
    private double mValueRad = 0;

    public Rot2d(Rotation2d pRads) {
        this.mValueRad = pRads.getRadians();
    }

    public Rot2d(double pRads) {
        this.mValueRad = pRads;
    }

    public static Rot2d fromRev(double pRevs) {
        return new Rot2d(revToRad(pRevs));
    }

    public static Rot2d fromDeg(double pDegs) {
        return new Rot2d(degToRad(pDegs));
    }

    public static Rot2d fromRad(double pRads) {
        return new Rot2d(pRads);
    }

    ////////// GETTERS \\\\\\\\\\
    public double rev() {
        return radToRev(mValueRad);
    }

    public double deg() {
        return radToDeg(mValueRad);
    }

    public double rad() {
        return mValueRad;
    }

    public Rotation2d getAsWPILibR2d() {
        return new Rotation2d(mValueRad);
    }

    ////////// PLUS \\\\\\\\\\
    public void plusRev(double pRevs) {
        mValueRad += revToRad(pRevs);
    }

    public void plusDeg(double pDegs) {
        mValueRad += degToRad(pDegs);
    }

    public void plusRad(double pRads) {
        mValueRad += pRads;
    }

    ////////// MINUS \\\\\\\\\\
    public void minusRev(double pRevs) {
        mValueRad -= revToRad(pRevs);
    }

    public void minusDeg(double pDegs) {
        mValueRad -= degToRad(pDegs);
    }

    public void minusRad(double pRads) {
        mValueRad -= pRads;
    }

    ////////// TRIG \\\\\\\\\\
    public void cosValue() {
        mValueRad = Math.cos(mValueRad);
    }

    public void sinValue() {
        mValueRad = Math.sin(mValueRad);
    }

    ////////// SCALAR OPERATIONS \\\\\\\\\\
    public void multiply(double scalar) {
        mValueRad /= scalar;
    }

    public void divide(double scalar) {
        mValueRad /= scalar;
    }

    public void exponentiateValue(int scalar) {
        mValueRad = Math.pow(mValueRad, scalar);
    }

    ////////// HELPER METHODS \\\\\\\\\\
    private static double radToRev(double pRads) {
        return (pRads) / (2.0 * Math.PI);
    }

    private static double radToDeg(double pRads) {
        return (pRads * 180.0) / (Math.PI);
    }

    private static double revToRad(double pRevs) {
        return pRevs * 2.0 * Math.PI;
    }

    private static double degToRad(Double pDegs) {
        return (pDegs * Math.PI) / (180.0);
    }
}