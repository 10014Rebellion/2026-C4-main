package frc.robot.systems.intake.roller;

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

public class IntakeRollerIOKrakenX44 implements IntakeRollerIO{
    private final TalonFX mIntakeRollerMotor;
    private final VoltageOut mIntakeRollerVoltageControl = new VoltageOut(0.0);

    private final StatusSignal<AngularVelocity> mIntakeRollerVelocityMPS;
    private final StatusSignal<Voltage> mIntakeRollerVoltage;
    private final StatusSignal<Current> mIntakeRollerSupplyCurrent;
    private final StatusSignal<Current> mIntakeRollerStatorCurrent;
    private final StatusSignal<Temperature> mIntakeRollerTempCelsius;
    private final StatusSignal<AngularAcceleration> mIntakeRollerAccelerationMPSS;
    
    public IntakeRollerIOKrakenX44(BasicMotorHardware pConfig) {
        mIntakeRollerMotor = new TalonFX(pConfig.motorID(), pConfig.canBus());
        var IntakeConfig = new TalonFXConfiguration();

        IntakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        IntakeConfig.CurrentLimits.SupplyCurrentLimit = pConfig.currentLimit().supplyCurrentLimit();
        IntakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        IntakeConfig.CurrentLimits.StatorCurrentLimit = pConfig.currentLimit().statorCurrentLimit();

        IntakeConfig.MotorOutput.NeutralMode = pConfig.neutralMode();
        IntakeConfig.MotorOutput.Inverted = pConfig.direction();

        IntakeConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        IntakeConfig.Feedback.SensorToMechanismRatio = pConfig.rotorToMechanismRatio();

        mIntakeRollerVelocityMPS = mIntakeRollerMotor.getVelocity();
        mIntakeRollerAccelerationMPSS = mIntakeRollerMotor.getAcceleration();
        mIntakeRollerVoltage = mIntakeRollerMotor.getMotorVoltage();
        mIntakeRollerSupplyCurrent = mIntakeRollerMotor.getSupplyCurrent();
        mIntakeRollerStatorCurrent = mIntakeRollerMotor.getStatorCurrent();
        mIntakeRollerTempCelsius = mIntakeRollerMotor.getDeviceTemp();
        
        mIntakeRollerMotor.getConfigurator().apply(IntakeConfig);

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
        pInputs.iIntakeRollerRPS = Rotation2d.fromRotations(mIntakeRollerVelocityMPS.getValueAsDouble());
        pInputs.iIntakeRollerAccelerationMPSS = mIntakeRollerAccelerationMPSS.getValueAsDouble();
        pInputs.iIntakeRollerMotorVolts = mIntakeRollerVoltage.getValueAsDouble();
        pInputs.iIntakeRollerSupplyCurrentAmps = mIntakeRollerSupplyCurrent.getValueAsDouble();
        pInputs.iIntakeRollerStatorCurrentAmps = mIntakeRollerStatorCurrent.getValueAsDouble();
        pInputs.iIntakeRollerTempCelsius = mIntakeRollerTempCelsius.getValueAsDouble();
    }

    @Override
    public void setMotorVolts(double pVolts) {
        mIntakeRollerMotor.setControl(mIntakeRollerVoltageControl.withOutput(pVolts).withEnableFOC(true));
    }

    @Override
    public void stopMotor() {
        mIntakeRollerMotor.stopMotor();
    }

}