package frc.lib.units;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;

public class Volts {
    private final double mVoltage;

    /*
     * Creates volts class that clamps passed in value between -12.0 and 12.0 volts
     */
    public Volts(double pVoltage) {
        this.mVoltage = MathUtil.clamp(pVoltage, -12.0, 12.0);
    }

    public double get() {
        return mVoltage;
    }

    public DoubleSupplier getSup() {
        return () -> mVoltage;
    }
}
