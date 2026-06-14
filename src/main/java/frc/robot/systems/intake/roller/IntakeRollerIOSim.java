package frc.robot.systems.intake.roller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.lib.hardware.HardwareRecords.BasicMotorHardware;
import frc.lib.hardware.HardwareRecords.FollowerMotorHardware;
import frc.robot.systems.intake.IntakeConstants;

public class IntakeRollerIOSim implements IntakeRollerIO {
    private final double kLoopPeriodSec = 0.02;
    private boolean mIsFollower;

    private final DCMotorSim kIntakeRoller;

    private double appliedVoltage = 0.0;

    // FOLLOWER CONSTRUCTOR
    public IntakeRollerIOSim(FollowerMotorHardware pFollowerConfig) {
        this(pFollowerConfig.motorID(), pFollowerConfig.leaderConfig());
        mIsFollower = true;
    }
    
    // LEADER CONSTRUCTOR
    public IntakeRollerIOSim(BasicMotorHardware pLeaderConfig) {
        this(pLeaderConfig.motorID(), pLeaderConfig);
        mIsFollower = false;
    }
    private IntakeRollerIOSim(int pMotorID, BasicMotorHardware pHardware) {
        kIntakeRoller = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(1), 
                0.5,
                pHardware.rotorToMechanismRatio()), 
            DCMotor.getKrakenX60(1).withReduction(pHardware.rotorToMechanismRatio()), 
            0.0, 0.0);
    }

    @Override
    public void updateInputs(IntakeRollerInputs inputs) {
        kIntakeRoller.update(kLoopPeriodSec);
        inputs.iIsLeader = !mIsFollower;
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