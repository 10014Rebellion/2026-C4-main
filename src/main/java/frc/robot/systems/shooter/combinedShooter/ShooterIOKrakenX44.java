package frc.robot.systems.shooter.combinedShooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.PhoenixUtil;
import frc.lib.PhoenixUtil.CanivoreBus;
import frc.lib.hardware.HardwareRecords.BasicMotorHardware;
import frc.lib.hardware.HardwareRecords.BasicShooterMotorHardware;
import frc.lib.hardware.HardwareRecords.FollowerMotorHardware;
import frc.lib.hardware.HardwareRecords.FollowerShooterMotorHardware;
import frc.lib.telemetry.Telemetry;
import frc.robot.logging.MotorErrors;

public class ShooterIOKrakenX44 implements ShooterIO{
    private final TalonFX mShooterMotor;
    private final VoltageOut mShooterVoltageControl = new VoltageOut(0.0);
    private final MotionMagicVelocityTorqueCurrentFOC mShooterVelocityControl = new MotionMagicVelocityTorqueCurrentFOC(0.0);
    private final StatusSignal<ControlModeValue> mShooterControlMode;
    private final StatusSignal<AngularVelocity> mShooterVelocityRPS;
    private final StatusSignal<Voltage> mShooterVoltage;
    private final StatusSignal<Current> mShooterSupplyCurrent;
    private final StatusSignal<Current> mShooterStatorCurrent;
    private final StatusSignal<Temperature> mShooterTempCelsius;
    private final StatusSignal<AngularAcceleration> mShooterAccelerationRPSS;
    private final StatusSignal<Double> mShooterClosedLoopReference;
    private final StatusSignal<Double> mShooterClosedLoopReferenceSlope;

    private Follower mFollowerController = null;


    // FOLLOWER CONSTRUCTOR
    public ShooterIOKrakenX44(FollowerMotorHardware pFollowerConfig) {
        this(pFollowerConfig.motorID(), pFollowerConfig.leaderConfig());
        this.mFollowerController = new Follower(pFollowerConfig.leaderConfig().motorID(), pFollowerConfig.alignmentValue()); 
        enforceFollower();
    }
    
    // LEADER CONSTRUCTOR
    public ShooterIOKrakenX44(BasicMotorHardware pLeaderConfig) {
        this(pLeaderConfig.motorID(), pLeaderConfig);
    }

    private ShooterIOKrakenX44(int pMotorID, BasicMotorHardware pHardware) {
            this.mShooterMotor = new TalonFX(pMotorID, pHardware.canBus());
    
            TalonFXConfiguration ShooterConfig = new TalonFXConfiguration();
    
            ShooterConfig.Voltage.PeakForwardVoltage = 12;
            ShooterConfig.Voltage.PeakReverseVoltage = -12;
    
            ShooterConfig.Slot0.kP = ShooterConstants.kFlywheelControlConfig.pdController().kP();
            ShooterConfig.Slot0.kD = ShooterConstants.kFlywheelControlConfig.pdController().kD();
            ShooterConfig.MotionMagic.MotionMagicCruiseVelocity = ShooterConstants.kFlywheelControlConfig.motionMagicConstants().maxVelocity();
            ShooterConfig.MotionMagic.MotionMagicAcceleration = ShooterConstants.kFlywheelControlConfig.motionMagicConstants().maxAcceleration();
            ShooterConfig.MotionMagic.MotionMagicJerk = ShooterConstants.kFlywheelControlConfig.motionMagicConstants().maxJerk();
    
            ShooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
            ShooterConfig.CurrentLimits.SupplyCurrentLimit = pHardware.currentLimit().supplyCurrentLimit();
            ShooterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
            ShooterConfig.CurrentLimits.StatorCurrentLimit = pHardware.currentLimit().statorCurrentLimit();
    
            ShooterConfig.MotorOutput.NeutralMode = pHardware.neutralMode();
            ShooterConfig.MotorOutput.Inverted = pHardware.direction();
    
            ShooterConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
            ShooterConfig.Feedback.SensorToMechanismRatio = pHardware.rotorToMechanismRatio();
    
            mShooterMotor.getConfigurator().apply(ShooterConfig);
    
            mShooterControlMode = mShooterMotor.getControlMode();
            mShooterVelocityRPS = mShooterMotor.getVelocity();
            mShooterAccelerationRPSS = mShooterMotor.getAcceleration();
            mShooterVoltage = mShooterMotor.getMotorVoltage();
            mShooterSupplyCurrent = mShooterMotor.getSupplyCurrent();
            mShooterStatorCurrent = mShooterMotor.getStatorCurrent();
            mShooterTempCelsius = mShooterMotor.getDeviceTemp();
            mShooterClosedLoopReference = mShooterMotor.getClosedLoopReference();
            mShooterClosedLoopReferenceSlope = mShooterMotor.getClosedLoopReferenceSlope();
    
            BaseStatusSignal.setUpdateFrequencyForAll(
                50.0, 
                mShooterControlMode,
                mShooterVelocityRPS, 
                mShooterAccelerationRPSS,
                mShooterVoltage,
                mShooterSupplyCurrent,
                mShooterStatorCurrent,
                mShooterTempCelsius,
                mShooterClosedLoopReference,
                mShooterClosedLoopReferenceSlope
            );
    
            mShooterMotor.optimizeBusUtilization(0.0);
    
            PhoenixUtil.registerSignals(
                CanivoreBus.OVERWORLD, 
                mShooterControlMode,
                mShooterVelocityRPS, 
                mShooterAccelerationRPSS,
                mShooterVoltage,
                mShooterSupplyCurrent,
                mShooterStatorCurrent,
                mShooterTempCelsius,
                mShooterClosedLoopReference,
                mShooterClosedLoopReferenceSlope);
    }

    @Override
    public void updateInputs(ShooterInputs pInputs) {
        pInputs.iIsShooterConnected = BaseStatusSignal.refreshAll(
            mShooterControlMode,
            mShooterVelocityRPS,
            mShooterAccelerationRPSS,
            mShooterVoltage,
            mShooterSupplyCurrent,
            mShooterStatorCurrent,
            mShooterTempCelsius,
            mShooterClosedLoopReference,
            mShooterClosedLoopReferenceSlope
        ).isOK();
        pInputs.iIsLeader = isLeader();
        pInputs.iShooterControlMode = mShooterControlMode.getValue().toString();
        pInputs.iShooterRotorVelocityRPS = Rotation2d.fromRotations(mShooterVelocityRPS.getValueAsDouble());
        pInputs.iShooterRotorAccelerationRPSS = Rotation2d.fromRotations(mShooterAccelerationRPSS.getValueAsDouble());
        pInputs.iShooterMotorVolts = mShooterVoltage.getValueAsDouble();
        pInputs.iShooterSupplyCurrentAmps = mShooterSupplyCurrent.getValueAsDouble();
        pInputs.iShooterStatorCurrentAmps = mShooterStatorCurrent.getValueAsDouble();
        pInputs.iShooterTempCelsius = mShooterTempCelsius.getValueAsDouble();
        pInputs.iShooterClosedLoopReference = Rotation2d.fromRotations(mShooterClosedLoopReference.getValueAsDouble());
        pInputs.iShooterClosedLoopReferenceSlope = Rotation2d.fromRotations(mShooterClosedLoopReferenceSlope.getValueAsDouble());
    }

    public boolean isLeader() {
        return mFollowerController == null;
    }

    @Override 
    public void setPDConstants(double pKP, double pKD) {
        Slot0Configs slotConfig = new Slot0Configs();
        slotConfig.kP = pKP;
        slotConfig.kD = pKD;
        mShooterMotor.getConfigurator().apply(slotConfig);
    }

    @Override 
    public void setMotionMagicConstants(double pCruiseVel, double pMaxAccel, double pMaxJerk) {
        MotionMagicConfigs motionMagicConfig = new MotionMagicConfigs();
        motionMagicConfig.MotionMagicCruiseVelocity = pCruiseVel;
        motionMagicConfig.MotionMagicAcceleration = pMaxAccel;
        motionMagicConfig.MotionMagicJerk = pMaxJerk;
        mShooterMotor.getConfigurator().apply(motionMagicConfig);
    }

    @Override 
    public void enforceFollower() {
        if(!isLeader()) mShooterMotor.setControl(mFollowerController);
        else Telemetry.reportIssue(new MotorErrors.EnforcingLeaderAsFollower(this));
    }

    @Override
    public void setMotorVelAndAccel(double pVelocityRPS, double pAccelerationRPSS, double pFeedforward) {
        if(isLeader()) mShooterMotor.setControl(mShooterVelocityControl.withVelocity(pVelocityRPS).withAcceleration(pAccelerationRPSS).withFeedForward(pFeedforward));
        else Telemetry.reportIssue(new MotorErrors.SettingControlToFollower(this));
    }

    @Override
    public void setMotorAmperage(double amps) {
        mShooterMotor.setControl(new TorqueCurrentFOC(amps));
    }

    @Override
    public void setMotorVolts(double pVolts) {
        if(isLeader()) mShooterMotor.setControl(mShooterVoltageControl.withOutput(pVolts));
        else Telemetry.reportIssue(new MotorErrors.SettingControlToFollower(this));
    }

    @Override
    public void stopMotor() {
        if(isLeader()) mShooterMotor.stopMotor(); 
        else Telemetry.reportIssue(new MotorErrors.SettingControlToFollower(this));
    }
}