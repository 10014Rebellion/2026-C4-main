package frc.robot.systems.intake.roller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.systems.intake.IntakeConstants;

public class IntakeRollerIOSim implements IntakeRollerIO {
    private final double kLoopPeriodSec = 0.02;

    private final DCMotorSim kIntakeRoller;

    private double appliedVoltage = 0.0;

    public IntakeRollerIOSim() {
        kIntakeRoller = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(1), 
                0.5,
                IntakeConstants.RollerConstants.kRollerMotorConfig.rotorToMechanismRatio()), 
            DCMotor.getKrakenX60(1).withReduction(IntakeConstants.RollerConstants.kRollerMotorConfig.rotorToMechanismRatio()), 
            0.0, 0.0);
  }

    @Override
    public void updateInputs(IntakeRollerInputs inputs) {
        kIntakeRoller.update(kLoopPeriodSec);

        inputs.iIsIntakeRollerConnected = true;

        inputs.iIntakeRollerRPS = Rotation2d.fromRotations(kIntakeRoller.getAngularVelocityRPM() / 60.0);
        inputs.iIntakeRollerMotorVolts = appliedVoltage;
        inputs.iIntakeRollerStatorCurrentAmps = kIntakeRoller.getCurrentDrawAmps();
        inputs.iIntakeRollerSupplyCurrentAmps = kIntakeRoller.getCurrentDrawAmps();
        inputs.iIntakeRollerTempCelsius = 25.0;
    }

    @Override
    public void setMotorVolts(double pVolts) {
        appliedVoltage = MathUtil.clamp(pVolts, -12.0, 12.0);
        kIntakeRoller.setInputVoltage(appliedVoltage);
    }

    @Override
    public void stopMotor() {
        setMotorVolts(0.0);
    }
}