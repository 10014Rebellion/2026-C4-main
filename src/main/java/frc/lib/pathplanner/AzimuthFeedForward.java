package frc.lib.pathplanner;

public record AzimuthFeedForward(double[] azimuthSpeedRadiansPS) {
    public static AzimuthFeedForward zeros() {
        return new AzimuthFeedForward(new double[] {
            0.0, 0.0, 0.0, 0.0
        });
    }
}

