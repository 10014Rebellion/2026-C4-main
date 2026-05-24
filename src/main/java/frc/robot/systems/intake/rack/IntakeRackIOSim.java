package frc.robot.systems.intake.rack;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.lib.hardware.HardwareRecords.BasicMotorHardware;
import frc.lib.simulation.SimulationRecords.SimulatedElevator;
import frc.robot.systems.intake.IntakeConstants;

public class IntakeRackIOSim implements IntakeRackIO{

    private double mAppliedVolts;
    private final ElevatorSim mRackSim;
    private ProfiledPIDController mIntakeRackController;

    public IntakeRackIOSim(SimulatedElevator pConfig, BasicMotorHardware pHardware) {

        mRackSim = new ElevatorSim(
            pConfig.kMotor(),
            pHardware.rotorToMechanismRatio(), 
            pConfig.kCarriageMassKg(), 
            pConfig.kDrumRadiusMeters(), 
            IntakeConstants.RackConstants.kRackLimitsMeters.backwardLimitM(),
            IntakeConstants.RackConstants.kRackLimitsMeters.forwardLimitM(),
            pConfig.kSimulateGravity(), 
            pConfig.kStartingHeightMeters(), 
            pConfig.kMeasurementStdDevs(),
            pConfig.kMeasurementStdDevs());

        mIntakeRackController = 
            new ProfiledPIDController(
                IntakeConstants.RackConstants.kRackController.pdController().kP(), 
                0.0, 
                IntakeConstants.RackConstants.kRackController.pdController().kD(),
                new TrapezoidProfile.Constraints(
                    IntakeConstants.RackConstants.kRackController.motionMagicConstants().maxVelocity(), 
                    IntakeConstants.RackConstants.kRackController.motionMagicConstants().maxAcceleration()));

        mAppliedVolts = 0.0;
    }

    @Override
    public void updateInputs(IntakeRackInputs pInputs){
        mRackSim.update(0.02);
        pInputs.iIsIntakeRackConnected = true;
        pInputs.iIntakeRackPositionM = mRackSim.getPositionMeters();
        pInputs.iIntakeRackVelocityMS = mRackSim.getVelocityMetersPerSecond();
        pInputs.iIntakeRackMotorVolts = mAppliedVolts;
        pInputs.iIntakeRackSupplyCurrentAmps = 0.0;
        pInputs.iIntakeRackStatorCurrentAmps = Math.abs(mRackSim.getCurrentDrawAmps());
        pInputs.iIntakeRackTempCelsius = 0.0;
        pInputs.iIntakeClosedLoopReference = mIntakeRackController.getSetpoint().position;
        pInputs.iIntakeClosedLoopReferenceSlope = mIntakeRackController.getSetpoint().velocity;
    }

    @Override
    public void setMotorVolts(double pVolts){
        mAppliedVolts = MathUtil.clamp(pVolts, -12.0, 12.0);
        mRackSim.setInputVoltage(mAppliedVolts);
    }

    @Override
    public void resetPPID() {
        mIntakeRackController.reset(getPos());
    }

    @Override
    public void setMotorPosition(double pPositionM, double feedforward) {
        setMotorVolts(mIntakeRackController.calculate(getPos(), pPositionM) + feedforward);
    }

    @Override
    public void setPDConstants(double pKP, double pKD) {
        mIntakeRackController.setPID(pKP, 0.0, pKD);
    }

    @Override
    public void setMotionMagicConstants(double pCruiseVelRPS, double pMaxAccelRPSS, double pMaxJerkRPSSS) {
        mIntakeRackController.setConstraints(new TrapezoidProfile.Constraints(pCruiseVelRPS, pMaxAccelRPSS));
    }

    public double getPos(){
        return mRackSim.getPositionMeters();
    }

    @Override
    public void stopMotor(){
        setMotorVolts(0.0);
    }
}
