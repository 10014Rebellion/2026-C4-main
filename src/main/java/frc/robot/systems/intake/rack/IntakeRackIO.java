// REBELLION 10014

package frc.robot.systems.intake.rack;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeRackIO {
    @AutoLog
    public static class IntakeRackInputs {
        public boolean iIsIntakeRackConnected = false;
        public double iIntakeRackPositionM = 0.0;
        public double iIntakeRackVelocityMS = 0.0;
        public double iIntakeRackMotorVolts = 0.0;
        public double iIntakeRackSupplyCurrentAmps = 0.0;
        public double iIntakeRackStatorCurrentAmps = 0.0;
        public double iIntakeRackTempCelsius = 0.0;
        public double iIntakeClosedLoopReference = 0.0;
        public double iIntakeClosedLoopReferenceSlope = 0.0;
    }

    public default void updateInputs(IntakeRackInputs pInputs) {}

    public default void setMotorVolts(double pVolts) {}

    public default void setMotorAmps(double pAmps) {}

    public default void resetPPID() {}

    public default void setMotorPosition(double pPositionM, double feedforward) {}

    public default void setPDConstants(double pKP, double pKD) {}

    public default void setMotionMagicConstants(double pCruiseVelRPS, double pMaxAccelRPSS, double pMaxJerkRPSSS) {}

    public default void stopMotor() {}

}
