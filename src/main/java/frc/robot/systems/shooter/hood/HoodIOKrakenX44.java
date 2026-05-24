package frc.robot.systems.shooter.hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.PhoenixUtil;
import frc.lib.PhoenixUtil.CanivoreBus;
import frc.lib.hardware.HardwareRecords.ArmControllerMotionMagic;
import frc.lib.hardware.HardwareRecords.BasicMotorHardware;

public class HoodIOKrakenX44 implements HoodIO{
    private final TalonFX mHoodMotor;
    private final VoltageOut mHoodVoltageControl = new VoltageOut(0.0);
    private final TorqueCurrentFOC mHoodTorqueControl = new TorqueCurrentFOC(0.0);
    private final MotionMagicVoltage mHoodPositionControl = new MotionMagicVoltage(0.0);
    private final StatusSignal<ControlModeValue> mHoodControlMode;
    private final StatusSignal<Angle> mHoodPosition;
    private final StatusSignal<AngularVelocity> mHoodVelocityRPS;
    private final StatusSignal<AngularAcceleration> mHoodAccelerationRPSS;
    private final StatusSignal<Voltage> mHoodVoltage;
    private final StatusSignal<Current> mHoodSupplyCurrent;
    private final StatusSignal<Current> mHoodStatorCurrent;
    private final StatusSignal<Temperature> mHoodTempCelsius;
    private final StatusSignal<Double> mHoodClosedLoopReference;
    private final StatusSignal<Double> mHoodClosedLoopReferenceSlope;

    public HoodIOKrakenX44(BasicMotorHardware pMotorHardware, ArmControllerMotionMagic pController) {
        this.mHoodMotor = new TalonFX(pMotorHardware.motorID(), pMotorHardware.canBus());

        TalonFXConfiguration HoodConfig = new TalonFXConfiguration();

        HoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        HoodConfig.CurrentLimits.SupplyCurrentLimit = pMotorHardware.currentLimit().supplyCurrentLimit();
        HoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        HoodConfig.CurrentLimits.StatorCurrentLimit = pMotorHardware.currentLimit().statorCurrentLimit();

        HoodConfig.MotorOutput.NeutralMode = pMotorHardware.neutralMode();
        HoodConfig.MotorOutput.Inverted = pMotorHardware.direction();

        HoodConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        HoodConfig.Feedback.RotorToSensorRatio = 1;
        HoodConfig.Feedback.SensorToMechanismRatio = pMotorHardware.rotorToMechanismRatio();

        // HoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        // HoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = mRotationSoftLimits.forwardLimit().getRotations();
        // HoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        // HoodConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = mRotationSoftLimits.backwardLimit().getRotations();


        // TODO: CHECK WITH TAHA 
        // Originally was 0.0 for all of these constraints
        // Swithced over to use the controller
        HoodConfig.MotionMagic.MotionMagicAcceleration = pController.motionMagicConstants().maxAcceleration();
        HoodConfig.MotionMagic.MotionMagicCruiseVelocity = pController.motionMagicConstants().maxVelocity();
        HoodConfig.MotionMagic.MotionMagicJerk = pController.motionMagicConstants().maxAcceleration();

        HoodConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        HoodConfig.Slot0.kP = pController.pdController().kP();
        HoodConfig.Slot0.kD = pController.pdController().kD();

        mHoodMotor.getConfigurator().apply(HoodConfig);

        mHoodControlMode = mHoodMotor.getControlMode();
        mHoodPosition = mHoodMotor.getPosition();
        mHoodVelocityRPS = mHoodMotor.getVelocity();
        mHoodAccelerationRPSS = mHoodMotor.getAcceleration();
        mHoodVoltage = mHoodMotor.getMotorVoltage();
        mHoodSupplyCurrent = mHoodMotor.getSupplyCurrent();
        mHoodStatorCurrent = mHoodMotor.getStatorCurrent();
        mHoodTempCelsius = mHoodMotor.getDeviceTemp();
        mHoodClosedLoopReference = mHoodMotor.getClosedLoopReference();
        mHoodClosedLoopReferenceSlope = mHoodMotor.getClosedLoopReferenceSlope();

        mHoodMotor.setPosition(0.0);

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0, 
            mHoodControlMode,
            mHoodPosition, 
            mHoodVelocityRPS,
            mHoodAccelerationRPSS,
            mHoodVoltage,
            mHoodSupplyCurrent,
            mHoodStatorCurrent,
            mHoodTempCelsius,
            mHoodClosedLoopReference,
            mHoodClosedLoopReferenceSlope
        );

        mHoodMotor.optimizeBusUtilization(0.0);

        PhoenixUtil.registerSignals(
            CanivoreBus.OVERWORLD, 
            mHoodControlMode,
            mHoodPosition, 
            mHoodVelocityRPS,
            mHoodAccelerationRPSS,
            mHoodVoltage,
            mHoodSupplyCurrent,
            mHoodStatorCurrent,
            mHoodTempCelsius,
            mHoodClosedLoopReference,
            mHoodClosedLoopReferenceSlope);
    }


    @Override
    public void updateInputs(HoodInputs pInputs) {
        pInputs.iIsHoodConnected = BaseStatusSignal.refreshAll(
            mHoodControlMode,
            mHoodPosition,
            mHoodVelocityRPS,
            mHoodAccelerationRPSS,
            mHoodVoltage,
            mHoodSupplyCurrent,
            mHoodStatorCurrent,
            mHoodTempCelsius,
            mHoodClosedLoopReference,
            mHoodClosedLoopReferenceSlope
        ).isOK();
        pInputs.iHoodControlMode = mHoodControlMode.getValue().toString();
        pInputs.iHoodAngle = getPos();
        pInputs.iHoodVelocityRotPS = Rotation2d.fromRotations(mHoodVelocityRPS.getValueAsDouble());
        pInputs.iHoodAccelerationRPSS = Rotation2d.fromRotations(mHoodAccelerationRPSS.getValueAsDouble());
        pInputs.iHoodMotorVolts = mHoodVoltage.getValueAsDouble();
        pInputs.iHoodSupplyCurrentAmps = mHoodSupplyCurrent.getValueAsDouble();
        pInputs.iHoodStatorCurrentAmps = mHoodStatorCurrent.getValueAsDouble();
        pInputs.iHoodTempCelsius = mHoodTempCelsius.getValueAsDouble();
        pInputs.iHoodReferenceValue = Rotation2d.fromRotations(mHoodClosedLoopReference.getValueAsDouble());
        pInputs.iHoodReferenceValueSlope = Rotation2d.fromRotations(mHoodClosedLoopReferenceSlope.getValueAsDouble());
    }

    private Rotation2d getPos() {
        return Rotation2d.fromRotations(mHoodPosition.getValueAsDouble());
    }

    @Override 
    public void setPDConstants(double pKP, double pKD) {
        Slot0Configs slotConfig = new Slot0Configs();
        slotConfig.kP = pKP;
        slotConfig.kD = pKD;
        mHoodMotor.getConfigurator().apply(slotConfig);
    }

    @Override 
    public void setMotionMagicConstants(double pCruiseVelRPS, double pMaxAccelRPS2, double pMaxJerkRPS3) {
        MotionMagicConfigs motionMagicConfig = new MotionMagicConfigs();
        motionMagicConfig.MotionMagicCruiseVelocity = pCruiseVelRPS;
        motionMagicConfig.MotionMagicAcceleration = pMaxAccelRPS2;
        motionMagicConfig.MotionMagicJerk = pMaxJerkRPS3;
        mHoodMotor.getConfigurator().apply(motionMagicConfig);
    }

    @Override
    public void setMotorPosition(Rotation2d pSetpoint, double pArbFF) {
        mHoodMotor.setControl(mHoodPositionControl.withPosition(pSetpoint.getRotations()).withFeedForward(pArbFF));
    }

    /*
     * When called, it should be run periodically
     */
    @Override
    public void setMotorVolts(double pVolts) {
        mHoodMotor.setControl(mHoodVoltageControl.withOutput(pVolts));
    }

    @Override
    public void setMotorAmps(double pAmps) {
        mHoodMotor.setControl(mHoodTorqueControl.withOutput(pAmps));
    }

    @Override
    public void stopMotor() {
        mHoodMotor.stopMotor();
    }
}