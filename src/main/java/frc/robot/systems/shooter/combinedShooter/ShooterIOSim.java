package frc.robot.systems.shooter.combinedShooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.lib.hardware.HardwareRecords.BasicMotorHardware;
import frc.lib.hardware.HardwareRecords.FollowerMotorHardware;

public class ShooterIOSim implements ShooterIO{

    private FlywheelSim mShooterSim;
    private boolean mIsFollower;
    private double mAppliedVoltage;
    private final PIDController mShooterController;

    // FOLLOWER CONSTRUCTOR
    public ShooterIOSim(FollowerMotorHardware pFollowerConfig) {
        this(pFollowerConfig.motorID(), pFollowerConfig.leaderConfig());
        mIsFollower = true;
    }
    
    // LEADER CONSTRUCTOR
    public ShooterIOSim(BasicMotorHardware pLeaderConfig) {
        this(pLeaderConfig.motorID(), pLeaderConfig);
        mIsFollower = false;
    }

    private ShooterIOSim(int pMotorID, BasicMotorHardware pHardware){
        mShooterSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX44Foc(1), 0.004, pHardware.rotorToMechanismRatio()),
            DCMotor.getKrakenX44Foc(1).withReduction(pHardware.rotorToMechanismRatio()),
            0.0009
        ); 

        mShooterController = new PIDController(
            ShooterConstants.kFlywheelControlConfig.pdController().kP(),
            0.0, 
             ShooterConstants.kFlywheelControlConfig.pdController().kD());
    }

    public void updateInputs(ShooterInputs pInputs) {
        mShooterSim.update(0.02);
        pInputs.iIsLeader = !mIsFollower;
        pInputs.iIsShooterConnected = true;
        pInputs.iShooterRotorAccelerationRPSS = Rotation2d.fromRotations(mShooterSim.getAngularAccelerationRadPerSecSq() / (Math.PI * 2));
        pInputs.iShooterMotorVolts = mAppliedVoltage;
        pInputs.iShooterStatorCurrentAmps = Math.abs(mShooterSim.getCurrentDrawAmps());
        pInputs.iShooterSupplyCurrentAmps = 0.0;
        pInputs.iShooterTempCelsius = 0.0;
        pInputs.iShooterRotorVelocityRPS = Rotation2d.fromRotations(mShooterSim.getAngularVelocityRPM() / 60.0);
        pInputs.iShooterClosedLoopReference = Rotation2d.kZero;
        pInputs.iShooterClosedLoopReferenceSlope = Rotation2d.kZero;
    }

    public void setPDConstants(double pKP, double pKD) {
        mShooterController.setPID(pKP, 0.0, pKD);
    }

    public void setMotionMagicConstants(double pCruiseVel, double pMaxAccel, double pMaxJerk) {
        return;
    }

    public void setMotorVelAndAccel(double pVelocityRPS, double pAccelerationRPSS, double pFeedforward) {
        Logger.recordOutput("Shooter/PIDVoltage", mShooterController.calculate(mShooterSim.getAngularVelocityRPM() / 60.0, pVelocityRPS) + pFeedforward);
        setMotorVolts(mShooterController.calculate(mShooterSim.getAngularVelocityRPM() / 60.0, pVelocityRPS) + pFeedforward);
    }

    // Inverts voltage if follower as the rest of the close loop control runs of this same method //
    public void setMotorVolts(double pVolts) {
        mAppliedVoltage = MathUtil.clamp(pVolts, -12.0, 12.0);
        mShooterSim.setInputVoltage(mAppliedVoltage);
    }

    public void stopMotor() {
        setMotorVolts(0.0);
    }

    // Will not be needed as I just used a 
    public void enforceFollower() {
        return;
    }
}
