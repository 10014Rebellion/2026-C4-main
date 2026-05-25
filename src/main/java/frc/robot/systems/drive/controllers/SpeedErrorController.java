package frc.robot.systems.drive.controllers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.tuning.LoggedTunableNumber;

public class SpeedErrorController {
    public static final LoggedTunableNumber tXP = new LoggedTunableNumber("SpeedErrorController/X/kP", 0.0);
    public static final LoggedTunableNumber tXI = new LoggedTunableNumber("SpeedErrorController/X/kI", 0.0);
    public static final LoggedTunableNumber tXIZone = new LoggedTunableNumber("SpeedErrorController/X/kIZone", 0.0);
    public static final LoggedTunableNumber tXIRange = new LoggedTunableNumber("SpeedErrorController/X/kIRange", 0.0);

    public static final LoggedTunableNumber tYP = new LoggedTunableNumber("SpeedErrorController/Y/kP", 0.0);
    public static final LoggedTunableNumber tYI = new LoggedTunableNumber("SpeedErrorController/Y/kI", 0.0);
    public static final LoggedTunableNumber tYIZone = new LoggedTunableNumber("SpeedErrorController/Y/kIZone", 0.0);
    public static final LoggedTunableNumber tYIRange = new LoggedTunableNumber("SpeedErrorController/Y/kIRange", 0.0);

    public static final LoggedTunableNumber tOmegaP = new LoggedTunableNumber("SpeedErrorController/Omega/kP", 0.0);
    public static final LoggedTunableNumber tOmegaI = new LoggedTunableNumber("SpeedErrorController/Omega/kI", 0.0);
    public static final LoggedTunableNumber tOmegaIZone = new LoggedTunableNumber("SpeedErrorController/Omega/kIZone", 0.0);
    public static final LoggedTunableNumber tOmegaIRange = new LoggedTunableNumber("SpeedErrorController/Omega/kIRange", 0.0);

    private final PIDController tXController;
    private final PIDController tYController;
    private final PIDController tOmegaController;

    public SpeedErrorController() {
        this.tXController = new PIDController(tXP.get(), tXI.get(), 0.0);
        tXController.setIntegratorRange(-tXIRange.get(), tXIRange.get());
        tXController.setIZone(tXIZone.get());

        this.tYController = new PIDController(tYP.get(), tYI.get(), 0.0);
        tYController.setIntegratorRange(-tYIRange.get(), tYIRange.get());
        tYController.setIZone(tYIZone.get());

        this.tOmegaController = new PIDController(tOmegaP.get(), tOmegaI.get(), 0.0);
        tOmegaController.setIntegratorRange(-tOmegaIRange.get(), tOmegaIRange.get());
        tOmegaController.setIZone(tOmegaIZone.get());
    }

    public ChassisSpeeds correctSpeed(ChassisSpeeds measured, ChassisSpeeds setpoint) {
        double vXCorrectionMPS = tXController.calculate(measured.vxMetersPerSecond, setpoint.vxMetersPerSecond);
        double vYCorrectionMPS = tYController.calculate(measured.vyMetersPerSecond, setpoint.vyMetersPerSecond);
        double omegaCorrectionRadPS = tOmegaController.calculate(measured.omegaRadiansPerSecond, setpoint.omegaRadiansPerSecond);

        return new ChassisSpeeds(
            setpoint.vxMetersPerSecond + vXCorrectionMPS,
            setpoint.vyMetersPerSecond + vYCorrectionMPS,
            setpoint.omegaRadiansPerSecond + omegaCorrectionRadPS
        );
    }

    public void updateControllers() {
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> {
                tXController.setPID(tXP.get(), tXI.get(), 0.0);
                tXController.setIntegratorRange(-tXIRange.get(), tXIRange.get());
                tXController.setIZone(tXIZone.get());
            },
            tXP,
            tXI,
            tXIRange,
            tXIZone);

        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> {
                tYController.setPID(tYP.get(), tYI.get(), 0.0);
                tYController.setIntegratorRange(-tYIRange.get(), tYIRange.get());
                tYController.setIZone(tYIZone.get());
            },
            tYP,
            tYI,
            tYIRange,
            tYIZone);

        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> {
                tOmegaController.setPID(tOmegaP.get(), tOmegaI.get(), 0.0);
                tOmegaController.setIntegratorRange(-tOmegaIRange.get(), tOmegaIRange.get());
                tOmegaController.setIZone(tOmegaIZone.get());
            },
            tOmegaP,
            tOmegaI,
            tOmegaIRange,
            tOmegaIZone);
    }
}
