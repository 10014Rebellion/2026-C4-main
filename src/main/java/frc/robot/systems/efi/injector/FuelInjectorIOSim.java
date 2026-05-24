package frc.robot.systems.efi.injector;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.systems.intake.IntakeConstants;

public class FuelInjectorIOSim implements FuelInjectorIO {
    private final double kLoopPeriodSec = 0.02;

    private final DCMotorSim kFuelInjector;

    private double appliedVoltage = 0.0;

    public FuelInjectorIOSim() {
        kFuelInjector = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(1), 
                0.5,
                FuelInjectorConstants.kFuelInjectorConfig.rotorToMechanismRatio()), 
            DCMotor.getKrakenX60(1).withReduction(FuelInjectorConstants.kFuelInjectorConfig.rotorToMechanismRatio()), 
            0.0, 0.0);
  }

    @Override
    public void updateInputs(FuelInjectorInputs inputs) {
        kFuelInjector.update(kLoopPeriodSec);

        inputs.iIsIntakeRollerConnected = true;

        inputs.iIntakeRollerRPS = Rotation2d.fromRotations(kFuelInjector.getAngularVelocityRPM() / 60.0);
        inputs.iIntakeRollerMotorVolts = appliedVoltage;
        inputs.iIntakeRollerStatorCurrentAmps = kFuelInjector.getCurrentDrawAmps();
        inputs.iIntakeRollerSupplyCurrentAmps = kFuelInjector.getCurrentDrawAmps();
        inputs.iIntakeRollerTempCelsius = 25.0;
    }

    @Override
    public void setMotorVolts(double pVolts) {
        appliedVoltage = MathUtil.clamp(pVolts, -12.0, 12.0);
        kFuelInjector.setInputVoltage(appliedVoltage);
    }

    @Override
    public void stopMotor() {
        setMotorVolts(0.0);
    }
}