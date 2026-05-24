package frc.robot.systems.efi.injector;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
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

public class FuelInjectorIOKrakenX60 implements FuelInjectorIO{
    private final TalonFX mFuelInjectorMotor;
    private final VoltageOut mFuelInjectorVoltageOut = new VoltageOut(0.0);

    private final StatusSignal<AngularVelocity> mIntakeRollerVelocityMPS;
    private final StatusSignal<Voltage> mIntakeRollerVoltage;
    private final StatusSignal<Current> mIntakeRollerSupplyCurrent;
    private final StatusSignal<Current> mIntakeRollerStatorCurrent;
    private final StatusSignal<Temperature> mIntakeRollerTempCelsius;
    private final StatusSignal<AngularAcceleration> mIntakeRollerAccelerationMPSS;
    
    public FuelInjectorIOKrakenX60(BasicMotorHardware pConfig) {
        mFuelInjectorMotor = new TalonFX(pConfig.motorID(), pConfig.canBus());
        var FuelInjectorConfig = new TalonFXConfiguration();

        FuelInjectorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        FuelInjectorConfig.CurrentLimits.SupplyCurrentLimit = pConfig.currentLimit().supplyCurrentLimit();
        FuelInjectorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        FuelInjectorConfig.CurrentLimits.StatorCurrentLimit = pConfig.currentLimit().statorCurrentLimit();

        FuelInjectorConfig.MotorOutput.NeutralMode = pConfig.neutralMode();
        FuelInjectorConfig.MotorOutput.Inverted = pConfig.direction();

        FuelInjectorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        FuelInjectorConfig.Feedback.SensorToMechanismRatio = pConfig.rotorToMechanismRatio();

        mIntakeRollerVelocityMPS = mFuelInjectorMotor.getVelocity();
        mIntakeRollerAccelerationMPSS = mFuelInjectorMotor.getAcceleration();
        mIntakeRollerVoltage = mFuelInjectorMotor.getMotorVoltage();
        mIntakeRollerSupplyCurrent = mFuelInjectorMotor.getSupplyCurrent();
        mIntakeRollerStatorCurrent = mFuelInjectorMotor.getStatorCurrent();
        mIntakeRollerTempCelsius = mFuelInjectorMotor.getDeviceTemp();
        
        mFuelInjectorMotor.getConfigurator().apply(FuelInjectorConfig);

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0, 
            mIntakeRollerVelocityMPS,
            mIntakeRollerAccelerationMPSS, 
            mIntakeRollerVoltage,
            mIntakeRollerSupplyCurrent,
            mIntakeRollerStatorCurrent,
            mIntakeRollerTempCelsius
        );

        mFuelInjectorMotor.optimizeBusUtilization(0.0);

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
    public void updateInputs(FuelInjectorInputs pInputs) {
        pInputs.iIsIntakeRollerConnected = BaseStatusSignal.isAllGood(
            mIntakeRollerVelocityMPS,
            mIntakeRollerAccelerationMPSS,
            mIntakeRollerVoltage,
            mIntakeRollerSupplyCurrent,
            mIntakeRollerStatorCurrent,
            mIntakeRollerTempCelsius
        );
        pInputs.iIntakeRollerRPS = Rotation2d.fromRotations(mIntakeRollerVelocityMPS.getValueAsDouble());
        pInputs.iIntakeRollerAccelerationMPSS = mIntakeRollerAccelerationMPSS.getValueAsDouble();
        pInputs.iIntakeRollerMotorVolts = mIntakeRollerVoltage.getValueAsDouble();
        pInputs.iIntakeRollerSupplyCurrentAmps = mIntakeRollerSupplyCurrent.getValueAsDouble();
        pInputs.iIntakeRollerStatorCurrentAmps = mIntakeRollerStatorCurrent.getValueAsDouble();
        pInputs.iIntakeRollerTempCelsius = mIntakeRollerTempCelsius.getValueAsDouble();
    }

    @Override
    public void setMotorVolts(double pVolts) {
        mFuelInjectorMotor.setControl(mFuelInjectorVoltageOut.withOutput(pVolts));
    }

    @Override
    public void stopMotor() {
        mFuelInjectorMotor.stopMotor();
    }

}