// REBELLION 10014

package frc.robot.systems.climb;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.PhoenixUtil;
import frc.lib.PhoenixUtil.CanivoreBus;
import frc.lib.hardware.HardwareRecords.BasicMotorHardware;

public class ClimbIOKrakenx44 implements ClimbIO {
    private final TalonFX mClimbMotor;
    private final VoltageOut mClimbVoltageControl = new VoltageOut(0.0);
    private final PositionDutyCycle mClimbPositionControl = new PositionDutyCycle(0.0);

    private final StatusSignal<AngularVelocity> mClimbVelocityMPS;
    private final StatusSignal<Voltage> mClimbVoltage;
    private final StatusSignal<Current> mClimbSupplyCurrent;
    private final StatusSignal<Current> mClimbStatorCurrent;
    private final StatusSignal<Temperature> mClimbTempCelsius;
    private final StatusSignal<AngularAcceleration> mClimbAccelerationMPSS;
    private final StatusSignal<Angle> mClimbPosition;
    private final StatusSignal<Double> mClosedLoopReference;
    private final BasicMotorHardware mConfigPlaceHolder;

    public ClimbIOKrakenx44(BasicMotorHardware pConfig) {
        mConfigPlaceHolder = pConfig;
        mClimbMotor = new TalonFX(pConfig.motorID(), pConfig.canBus());
        var ClimbConfig = new TalonFXConfiguration();

        ClimbConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        ClimbConfig.CurrentLimits.SupplyCurrentLimit = pConfig.currentLimit().supplyCurrentLimit();
        ClimbConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        ClimbConfig.CurrentLimits.StatorCurrentLimit = pConfig.currentLimit().statorCurrentLimit();

        ClimbConfig.Voltage.PeakForwardVoltage = 12;
        ClimbConfig.Voltage.PeakReverseVoltage = -12;

        ClimbConfig.MotorOutput.NeutralMode = pConfig.neutralMode();
        ClimbConfig.MotorOutput.Inverted = pConfig.direction();

        ClimbConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        ClimbConfig.Feedback.SensorToMechanismRatio = pConfig.rotorToMechanismRatio();
        
        
        mClimbMotor.getConfigurator().apply(ClimbConfig);
        
        mClimbVelocityMPS = mClimbMotor.getVelocity();
        mClimbAccelerationMPSS = mClimbMotor.getAcceleration();
        mClimbVoltage = mClimbMotor.getMotorVoltage();
        mClimbSupplyCurrent = mClimbMotor.getSupplyCurrent();
        mClimbStatorCurrent = mClimbMotor.getStatorCurrent();
        mClimbTempCelsius = mClimbMotor.getDeviceTemp();
        mClimbPosition = mClimbMotor.getPosition();
        mClosedLoopReference = mClimbMotor.getClosedLoopReference();

        mClimbMotor.setPosition(0.0);

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0, 
            mClimbVelocityMPS, 
            mClimbAccelerationMPSS,
            mClimbVoltage,
            mClimbSupplyCurrent,
            mClimbStatorCurrent,
            mClimbTempCelsius,
            mClimbPosition,
            mClosedLoopReference);

        // mClimbMotor.setPosition(0.0);
        mClimbMotor.optimizeBusUtilization(0.0);

        PhoenixUtil.registerSignals(
            CanivoreBus.OVERWORLD, 
            mClimbVelocityMPS, 
            mClimbAccelerationMPSS,
            mClimbVoltage,
            mClimbSupplyCurrent,
            mClimbStatorCurrent,
            mClimbTempCelsius,
            mClimbPosition,
            mClosedLoopReference);
    }

    @Override
    public void updateInputs(ClimbInputs pInputs) {
        pInputs.iIsClimbConnected = BaseStatusSignal.isAllGood(
            mClimbVelocityMPS,
            mClimbAccelerationMPSS,
            mClimbVoltage,
            mClimbPosition,
            mClimbSupplyCurrent,
            mClimbStatorCurrent,
            mClimbTempCelsius,
            mClosedLoopReference
        );
        pInputs.iClimbMotorVolts = mClimbVoltage.getValueAsDouble();
        pInputs.iClimbSupplyCurrentAmps = mClimbSupplyCurrent.getValueAsDouble();
        pInputs.iClimbStatorCurrentAmps = mClimbStatorCurrent.getValueAsDouble();
        pInputs.iClimbTempCelsius = mClimbTempCelsius.getValueAsDouble();
        pInputs.iClimbPositionMeters = mClimbPosition.getValueAsDouble();
        pInputs.iClimbReferenceValue = mClosedLoopReference.getValueAsDouble();
    }

    @Override
    public void setMotorPosition(double pPositionM, double pFeedforward) {
        mClimbMotor.setControl(mClimbPositionControl.withPosition(pPositionM).withFeedForward(pFeedforward));
    }

    @Override
    public void setMotorVolts(double pVolts) {
        mClimbMotor.setControl(mClimbVoltageControl.withOutput(pVolts));
    }

    @Override
    public void stopMotor() {
        mClimbMotor.stopMotor();
    }

    @Override
    public void changeClimbNeutralMode(NeutralModeValue pNeutralMode) {
        MotorOutputConfigs mConfig = new MotorOutputConfigs();
        mConfig.NeutralMode = pNeutralMode;
        mConfig.Inverted = mConfigPlaceHolder.direction();
        mClimbMotor.getConfigurator().apply(mConfig);
    }
}
