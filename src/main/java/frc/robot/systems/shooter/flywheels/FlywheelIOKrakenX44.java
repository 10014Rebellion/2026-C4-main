package frc.robot.systems.shooter.flywheels;

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
import frc.lib.hardware.HardwareRecords.FollowerMotorHardware;
import frc.lib.telemetry.Telemetry;
import frc.robot.logging.MotorErrors;

public class FlywheelIOKrakenX44 implements FlywheelIO{
    private final TalonFX mFlywheelMotor;
    private final VoltageOut mFlywheelVoltageControl = new VoltageOut(0.0);
    private final MotionMagicVelocityTorqueCurrentFOC mFlywheelVelocityControl = new MotionMagicVelocityTorqueCurrentFOC(0.0);
    private final StatusSignal<ControlModeValue> mFlywheelControlMode;
    private final StatusSignal<AngularVelocity> mFlywheelVelocityRPS;
    private final StatusSignal<Voltage> mFlywheelVoltage;
    private final StatusSignal<Current> mFlywheelSupplyCurrent;
    private final StatusSignal<Current> mFlywheelStatorCurrent;
    private final StatusSignal<Temperature> mFlywheelTempCelsius;
    private final StatusSignal<AngularAcceleration> mFlywheelAccelerationRPSS;
    private final StatusSignal<Double> mFlywheelClosedLoopReference;
    private final StatusSignal<Double> mFlywheelClosedLoopReferenceSlope;
    private Follower mFollowerController = null;


    // FOLLOWER CONSTRUCTOR
    public FlywheelIOKrakenX44(FollowerMotorHardware pFollowerConfig) {
        this(pFollowerConfig.motorID(), pFollowerConfig.leaderConfig());
        this.mFollowerController = new Follower(pFollowerConfig.leaderConfig().motorID(), pFollowerConfig.alignmentValue()); 
        enforceFollower();
    }
    
    // LEADER CONSTRUCTOR
    public FlywheelIOKrakenX44(BasicMotorHardware pLeaderConfig) {
        this(pLeaderConfig.motorID(), pLeaderConfig);
    }

    private FlywheelIOKrakenX44(int pMotorID, BasicMotorHardware pHardware) {
        this.mFlywheelMotor = new TalonFX(pMotorID, pHardware.canBus());

        TalonFXConfiguration FlywheelConfig = new TalonFXConfiguration();

        FlywheelConfig.Voltage.PeakForwardVoltage = 12;
        FlywheelConfig.Voltage.PeakReverseVoltage = -12;

        FlywheelConfig.Slot0.kP = FlywheelConstants.kFlywheelControlConfig.pdController().kP();
        FlywheelConfig.Slot0.kD = FlywheelConstants.kFlywheelControlConfig.pdController().kD();
        FlywheelConfig.MotionMagic.MotionMagicCruiseVelocity = FlywheelConstants.kFlywheelControlConfig.motionMagicConstants().maxVelocity();
        FlywheelConfig.MotionMagic.MotionMagicAcceleration = FlywheelConstants.kFlywheelControlConfig.motionMagicConstants().maxAcceleration();
        FlywheelConfig.MotionMagic.MotionMagicJerk = FlywheelConstants.kFlywheelControlConfig.motionMagicConstants().maxJerk();

        FlywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        FlywheelConfig.CurrentLimits.SupplyCurrentLimit = pHardware.currentLimit().supplyCurrentLimit();
        FlywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        FlywheelConfig.CurrentLimits.StatorCurrentLimit = pHardware.currentLimit().statorCurrentLimit();

        FlywheelConfig.MotorOutput.NeutralMode = pHardware.neutralMode();
        FlywheelConfig.MotorOutput.Inverted = pHardware.direction();

        FlywheelConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        FlywheelConfig.Feedback.SensorToMechanismRatio = pHardware.rotorToMechanismRatio();

        mFlywheelMotor.getConfigurator().apply(FlywheelConfig);

        mFlywheelControlMode = mFlywheelMotor.getControlMode();
        mFlywheelVelocityRPS = mFlywheelMotor.getVelocity();
        mFlywheelAccelerationRPSS = mFlywheelMotor.getAcceleration();
        mFlywheelVoltage = mFlywheelMotor.getMotorVoltage();
        mFlywheelSupplyCurrent = mFlywheelMotor.getSupplyCurrent();
        mFlywheelStatorCurrent = mFlywheelMotor.getStatorCurrent();
        mFlywheelTempCelsius = mFlywheelMotor.getDeviceTemp();
        mFlywheelClosedLoopReference = mFlywheelMotor.getClosedLoopReference();
        mFlywheelClosedLoopReferenceSlope = mFlywheelMotor.getClosedLoopReferenceSlope();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0, 
            mFlywheelControlMode,
            mFlywheelVelocityRPS, 
            mFlywheelAccelerationRPSS,
            mFlywheelVoltage,
            mFlywheelSupplyCurrent,
            mFlywheelStatorCurrent,
            mFlywheelTempCelsius,
            mFlywheelClosedLoopReference,
            mFlywheelClosedLoopReferenceSlope
        );

        mFlywheelMotor.optimizeBusUtilization(0.0);

        PhoenixUtil.registerSignals(
            CanivoreBus.OVERWORLD, 
            mFlywheelControlMode,
            mFlywheelVelocityRPS, 
            mFlywheelAccelerationRPSS,
            mFlywheelVoltage,
            mFlywheelSupplyCurrent,
            mFlywheelStatorCurrent,
            mFlywheelTempCelsius,
            mFlywheelClosedLoopReference,
            mFlywheelClosedLoopReferenceSlope);
    }

    @Override
    public void updateInputs(FlywheelInputs pInputs) {
        pInputs.iIsFlywheelConnected = BaseStatusSignal.refreshAll(
            mFlywheelControlMode,
            mFlywheelVelocityRPS,
            mFlywheelAccelerationRPSS,
            mFlywheelVoltage,
            mFlywheelSupplyCurrent,
            mFlywheelStatorCurrent,
            mFlywheelTempCelsius,
            mFlywheelClosedLoopReference,
            mFlywheelClosedLoopReferenceSlope
        ).isOK();
        pInputs.iIsLeader = isLeader();
        pInputs.iFlywheelControlMode = mFlywheelControlMode.getValue().toString();
        pInputs.iFlywheelRotorVelocityRPS = Rotation2d.fromRotations(mFlywheelVelocityRPS.getValueAsDouble());
        pInputs.iFlywheelRotorAccelerationRPSS = Rotation2d.fromRotations(mFlywheelAccelerationRPSS.getValueAsDouble());
        pInputs.iFlywheelMotorVolts = mFlywheelVoltage.getValueAsDouble();
        pInputs.iFlywheelSupplyCurrentAmps = mFlywheelSupplyCurrent.getValueAsDouble();
        pInputs.iFlywheelStatorCurrentAmps = mFlywheelStatorCurrent.getValueAsDouble();
        pInputs.iFlywheelTempCelsius = mFlywheelTempCelsius.getValueAsDouble();
        pInputs.iFlywheelClosedLoopReference = Rotation2d.fromRotations(mFlywheelClosedLoopReference.getValueAsDouble());
        pInputs.iFlywheelClosedLoopReferenceSlope = Rotation2d.fromRotations(mFlywheelClosedLoopReferenceSlope.getValueAsDouble());
    }

    public boolean isLeader() {
        return mFollowerController == null;
    }

    @Override 
    public void setPDConstants(double pKP, double pKD) {
        Slot0Configs slotConfig = new Slot0Configs();
        slotConfig.kP = pKP;
        slotConfig.kD = pKD;
        mFlywheelMotor.getConfigurator().apply(slotConfig);
    }

    @Override 
    public void setMotionMagicConstants(double pCruiseVel, double pMaxAccel, double pMaxJerk) {
        MotionMagicConfigs motionMagicConfig = new MotionMagicConfigs();
        motionMagicConfig.MotionMagicCruiseVelocity = pCruiseVel;
        motionMagicConfig.MotionMagicAcceleration = pMaxAccel;
        motionMagicConfig.MotionMagicJerk = pMaxJerk;
        mFlywheelMotor.getConfigurator().apply(motionMagicConfig);
    }

    @Override 
    public void enforceFollower() {
        if(!isLeader()) mFlywheelMotor.setControl(mFollowerController);
        else Telemetry.reportIssue(new MotorErrors.EnforcingLeaderAsFollower(this));
    }

    @Override
    public void setMotorVelAndAccel(double pVelocityRPS, double pAccelerationRPSS, double pFeedforward) {
        if(isLeader()) mFlywheelMotor.setControl(mFlywheelVelocityControl.withVelocity(pVelocityRPS).withAcceleration(pAccelerationRPSS).withFeedForward(pFeedforward));
        else Telemetry.reportIssue(new MotorErrors.SettingControlToFollower(this));
    }

    @Override
    public void setMotorAmperage(double amps) {
        mFlywheelMotor.setControl(new TorqueCurrentFOC(amps));
    }

    @Override
    public void setMotorVolts(double pVolts) {
        if(isLeader()) mFlywheelMotor.setControl(mFlywheelVoltageControl.withOutput(pVolts));
        else Telemetry.reportIssue(new MotorErrors.SettingControlToFollower(this));
    }

    @Override
    public void stopMotor() {
        if(isLeader()) mFlywheelMotor.stopMotor(); 
        else Telemetry.reportIssue(new MotorErrors.SettingControlToFollower(this));
    }
}