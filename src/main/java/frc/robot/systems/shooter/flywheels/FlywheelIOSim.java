package frc.robot.systems.shooter.flywheels;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.lib.hardware.HardwareRecords.BasicMotorHardware;
import frc.lib.hardware.HardwareRecords.FollowerMotorHardware;

public class FlywheelIOSim implements FlywheelIO{

    private FlywheelSim mFlywheelSim;
    private boolean mIsFollower;
    private double mAppliedVoltage;
    private final PIDController mFlywheelController;

    // FOLLOWER CONSTRUCTOR
    public FlywheelIOSim(FollowerMotorHardware pFollowerConfig) {
        this(pFollowerConfig.motorID(), pFollowerConfig.leaderConfig());
        mIsFollower = true;
    }
    
    // LEADER CONSTRUCTOR
    public FlywheelIOSim(BasicMotorHardware pLeaderConfig) {
        this(pLeaderConfig.motorID(), pLeaderConfig);
        mIsFollower = false;
    }

    private FlywheelIOSim(int pMotorID, BasicMotorHardware pHardware){
        mFlywheelSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX44Foc(1), 0.004, pHardware.rotorToMechanismRatio()),
            DCMotor.getKrakenX44Foc(1).withReduction(pHardware.rotorToMechanismRatio()),
            0.0009
        ); 

        mFlywheelController = new PIDController(
            FlywheelConstants.kFlywheelControlConfig.pdController().kP(),
            0.0, 
             FlywheelConstants.kFlywheelControlConfig.pdController().kD());
    }

    public void updateInputs(FlywheelInputs pInputs) {
        mFlywheelSim.update(0.02);
        pInputs.iIsLeader = !mIsFollower;
        pInputs.iIsFlywheelConnected = true;
        pInputs.iFlywheelRotorAccelerationRPSS = Rotation2d.fromRotations(mFlywheelSim.getAngularAccelerationRadPerSecSq() / (Math.PI * 2));
        pInputs.iFlywheelMotorVolts = mAppliedVoltage;
        pInputs.iFlywheelStatorCurrentAmps = Math.abs(mFlywheelSim.getCurrentDrawAmps());
        pInputs.iFlywheelSupplyCurrentAmps = 0.0;
        pInputs.iFlywheelTempCelsius = 0.0;
        pInputs.iFlywheelRotorVelocityRPS = Rotation2d.fromRotations(mFlywheelSim.getAngularVelocityRPM() / 60.0);
        pInputs.iFlywheelClosedLoopReference = Rotation2d.kZero;
        pInputs.iFlywheelClosedLoopReferenceSlope = Rotation2d.kZero;
    }

    public void setPDConstants(double pKP, double pKD) {
        mFlywheelController.setPID(pKP, 0.0, pKD);
    }

    public void setMotionMagicConstants(double pCruiseVel, double pMaxAccel, double pMaxJerk) {
        return;
    }

    public void setMotorVelAndAccel(double pVelocityRPS, double pAccelerationRPSS, double pFeedforward) {
        Logger.recordOutput("Flywheel/PIDVoltage", mFlywheelController.calculate(mFlywheelSim.getAngularVelocityRPM() / 60.0, pVelocityRPS) + pFeedforward);
        setMotorVolts(mFlywheelController.calculate(mFlywheelSim.getAngularVelocityRPM() / 60.0, pVelocityRPS) + pFeedforward);
    }

    // Inverts voltage if follower as the rest of the close loop control runs of this same method //
    public void setMotorVolts(double pVolts) {
        mAppliedVoltage = MathUtil.clamp(pVolts, -12.0, 12.0);
        mFlywheelSim.setInputVoltage(mAppliedVoltage);
    }

    public void stopMotor() {
        setMotorVolts(0.0);
    }

    // Will not be needed as I just used a 
    public void enforceFollower() {
        return;
    }
}
