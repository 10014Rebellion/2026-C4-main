package frc.robot.systems.intake.roller;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.hardware.HardwareRecords.BasicMotorHardware;
import frc.lib.hardware.HardwareRecords.FollowerMotorHardware;
import frc.lib.telemetry.Telemetry;
import frc.lib.util.PhoenixUtil;
import frc.lib.util.PhoenixUtil.CanivoreBus;
import frc.robot.logging.MotorErrors;

public class IntakeRollerIOKrakenX60 implements IntakeRollerIO{
    private final TalonFX mIntakeRollerMotor;
    private final VoltageOut mIntakeRollerVoltageControl = new VoltageOut(0.0);

    private final StatusSignal<AngularVelocity> mIntakeRollerVelocityMPS;
    private final StatusSignal<Voltage> mIntakeRollerVoltage;
    private final StatusSignal<Current> mIntakeRollerSupplyCurrent;
    private final StatusSignal<Current> mIntakeRollerStatorCurrent;
    private final StatusSignal<Temperature> mIntakeRollerTempCelsius;
    private final StatusSignal<AngularAcceleration> mIntakeRollerAccelerationMPSS;
    private Follower mFollowerController = null;

    // FOLLOWER CONSTRUCTOR
    public IntakeRollerIOKrakenX60(FollowerMotorHardware pFollowerConfig) {
        this(pFollowerConfig.motorID(), pFollowerConfig.leaderConfig());
        this.mFollowerController = new Follower(pFollowerConfig.leaderConfig().motorID(), pFollowerConfig.alignmentValue()); 
        enforceFollower();
    }
    
    // LEADER CONSTRUCTOR
    public IntakeRollerIOKrakenX60(BasicMotorHardware pLeaderConfig) {
        this(pLeaderConfig.motorID(), pLeaderConfig);
    }

    private IntakeRollerIOKrakenX60(int pMotorID, BasicMotorHardware pConfig) {
        mIntakeRollerMotor = new TalonFX(pMotorID, pConfig.canBus());
        var IntakeConfig = new TalonFXConfiguration();

        IntakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        IntakeConfig.CurrentLimits.SupplyCurrentLimit = pConfig.currentLimit().supplyCurrentLimit();
        IntakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        IntakeConfig.CurrentLimits.StatorCurrentLimit = pConfig.currentLimit().statorCurrentLimit();

        IntakeConfig.MotorOutput.NeutralMode = pConfig.neutralMode();
        IntakeConfig.MotorOutput.Inverted = pConfig.direction();

        IntakeConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        IntakeConfig.Feedback.SensorToMechanismRatio = pConfig.rotorToMechanismRatio();
        
        mIntakeRollerMotor.getConfigurator().apply(IntakeConfig);
        mIntakeRollerVelocityMPS = mIntakeRollerMotor.getVelocity();
        mIntakeRollerAccelerationMPSS = mIntakeRollerMotor.getAcceleration();
        mIntakeRollerVoltage = mIntakeRollerMotor.getMotorVoltage();
        mIntakeRollerSupplyCurrent = mIntakeRollerMotor.getSupplyCurrent();
        mIntakeRollerStatorCurrent = mIntakeRollerMotor.getStatorCurrent();
        mIntakeRollerTempCelsius = mIntakeRollerMotor.getDeviceTemp();
        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0, 
            mIntakeRollerVelocityMPS,
            mIntakeRollerAccelerationMPSS, 
            mIntakeRollerVoltage,
            mIntakeRollerSupplyCurrent,
            mIntakeRollerStatorCurrent,
            mIntakeRollerTempCelsius
        );

        mIntakeRollerMotor.optimizeBusUtilization(0.0);

        PhoenixUtil.registerSignals(
            CanivoreBus.OVERWORLD, 
            mIntakeRollerVelocityMPS,
            mIntakeRollerAccelerationMPSS, 
            mIntakeRollerVoltage,
            mIntakeRollerSupplyCurrent,
            mIntakeRollerStatorCurrent,
            mIntakeRollerTempCelsius);
    }

    @Override
    public void updateInputs(IntakeRollerInputs pInputs) {
        pInputs.iIsIntakeRollerConnected = BaseStatusSignal.isAllGood(
            mIntakeRollerVelocityMPS,
            mIntakeRollerAccelerationMPSS,
            mIntakeRollerVoltage,
            mIntakeRollerSupplyCurrent,
            mIntakeRollerStatorCurrent,
            mIntakeRollerTempCelsius
        );
        pInputs.iIsLeader = isLeader();
        pInputs.iIntakeRollerRPS = Rotation2d.fromRotations(mIntakeRollerVelocityMPS.getValueAsDouble());
        pInputs.iIntakeRollerAccelerationMPSS = mIntakeRollerAccelerationMPSS.getValueAsDouble();
        pInputs.iIntakeRollerMotorVolts = mIntakeRollerVoltage.getValueAsDouble();
        pInputs.iIntakeRollerSupplyCurrentAmps = mIntakeRollerSupplyCurrent.getValueAsDouble();
        pInputs.iIntakeRollerStatorCurrentAmps = mIntakeRollerStatorCurrent.getValueAsDouble();
        pInputs.iIntakeRollerTempCelsius = mIntakeRollerTempCelsius.getValueAsDouble();
    }
    public boolean isLeader() {
        return mFollowerController == null;
    }

    @Override 
    public void enforceFollower() {
        if(!isLeader()) mIntakeRollerMotor.setControl(mFollowerController);
        else Telemetry.reportIssue(new MotorErrors.EnforcingLeaderAsFollower(this));
    }
    @Override
    public void setMotorVolts(double pVolts) {
        if(isLeader()) mIntakeRollerMotor.setControl(mIntakeRollerVoltageControl.withOutput(pVolts));
        else Telemetry.reportIssue(new MotorErrors.SettingControlToFollower(this));
    }

    @Override
    public void stopMotor() {
        if(isLeader()) mIntakeRollerMotor.stopMotor(); 
        else Telemetry.reportIssue(new MotorErrors.SettingControlToFollower(this));
    }
}