package frc.robot.systems.intake.rack;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.PhoenixUtil;
import frc.lib.PhoenixUtil.CanivoreBus;
import frc.lib.hardware.HardwareRecords.BasicMotorHardware;
import frc.robot.systems.intake.IntakeConstants.RackConstants;

public class IntakeRackIOKrakenX60 implements IntakeRackIO{
    private final TalonFX mIntakeRackMotor;

    private final VoltageOut mIntakeRackVoltageControl = new VoltageOut(0.0);
    private final TorqueCurrentFOC mIntakeRackAmpsControl = new TorqueCurrentFOC(0.0);
    private final MotionMagicTorqueCurrentFOC mIntakeRackRotationControl = new MotionMagicTorqueCurrentFOC(0.0);
    
    private final StatusSignal<Angle> mIntakeRackRotation;
    private final StatusSignal<AngularVelocity> mIntakeRackVelocityRPS;
    private final StatusSignal<AngularAcceleration> mIntakeRackAccelerationRPSS;

    private final StatusSignal<Voltage> mIntakeRackVoltage;
    private final StatusSignal<Current> mIntakeRackSupplyCurrent;
    private final StatusSignal<Current> mIntakeRackStatorCurrent;
    private final StatusSignal<Temperature> mIntakeRackTempCelsius;
    private final StatusSignal<Double> mIntakeRackReferencePosition;
    private final StatusSignal<Double> mIntakeRackReferencePositionSlope;
    
    public IntakeRackIOKrakenX60(BasicMotorHardware pConfig) {
        // Motor
        mIntakeRackMotor = new TalonFX(pConfig.motorID(), pConfig.canBus());
        var IntakeConfig = new TalonFXConfiguration();
        mIntakeRackMotor.getConfigurator().apply(IntakeConfig);

        IntakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        IntakeConfig.CurrentLimits.SupplyCurrentLimit = pConfig.currentLimit().supplyCurrentLimit();
        IntakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        IntakeConfig.CurrentLimits.StatorCurrentLimit = pConfig.currentLimit().statorCurrentLimit();

        IntakeConfig.MotorOutput.NeutralMode = pConfig.neutralMode();
        IntakeConfig.MotorOutput.Inverted = pConfig.direction();

        IntakeConfig.Feedback.SensorToMechanismRatio = pConfig.rotorToMechanismRatio();

        // Dont need a gravity type as work between F_G and elevator is 0//
        IntakeConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        IntakeConfig.Slot0.kP = RackConstants.kRackController.pdController().kP();
        IntakeConfig.Slot0.kD = RackConstants.kRackController.pdController().kD();
        IntakeConfig.Slot0.kS = RackConstants.kRackController.feedforward().getKs();
        IntakeConfig.Slot0.kG = RackConstants.kRackController.feedforward().getKg();
        IntakeConfig.Slot0.kV = RackConstants.kRackController.feedforward().getKv();
        IntakeConfig.Slot0.kA = RackConstants.kRackController.feedforward().getKa();
        IntakeConfig.MotionMagic.MotionMagicCruiseVelocity = RackConstants.kRackController.motionMagicConstants().maxVelocity();
        IntakeConfig.MotionMagic.MotionMagicAcceleration = RackConstants.kRackController.motionMagicConstants().maxAcceleration();
        IntakeConfig.MotionMagic.MotionMagicJerk = RackConstants.kRackController.motionMagicConstants().maxJerk();

        mIntakeRackMotor.getConfigurator().apply(IntakeConfig);

        mIntakeRackRotation = mIntakeRackMotor.getPosition();
        mIntakeRackVelocityRPS = mIntakeRackMotor.getVelocity();
        mIntakeRackAccelerationRPSS = mIntakeRackMotor.getAcceleration();
        mIntakeRackVoltage = mIntakeRackMotor.getMotorVoltage();
        mIntakeRackSupplyCurrent = mIntakeRackMotor.getSupplyCurrent();
        mIntakeRackStatorCurrent = mIntakeRackMotor.getStatorCurrent();
        mIntakeRackTempCelsius = mIntakeRackMotor.getDeviceTemp();
        mIntakeRackReferencePosition = mIntakeRackMotor.getClosedLoopReference();
        mIntakeRackReferencePositionSlope = mIntakeRackMotor.getClosedLoopReferenceSlope();

        mIntakeRackMotor.setPosition(0.0);

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0, 
            mIntakeRackRotation,
            mIntakeRackVelocityRPS,
            mIntakeRackAccelerationRPSS, 
            mIntakeRackVoltage,
            mIntakeRackSupplyCurrent,
            mIntakeRackStatorCurrent,
            mIntakeRackTempCelsius,
            mIntakeRackReferencePosition,
            mIntakeRackReferencePositionSlope
        );

        mIntakeRackMotor.optimizeBusUtilization(0.0);

        PhoenixUtil.registerSignals(
            CanivoreBus.OVERWORLD,
            mIntakeRackRotation,
            mIntakeRackVelocityRPS,
            mIntakeRackVoltage,
            mIntakeRackSupplyCurrent,
            mIntakeRackStatorCurrent,
            mIntakeRackTempCelsius,
            mIntakeRackReferencePosition,
            mIntakeRackReferencePositionSlope);
    }

    @Override
    public void updateInputs(IntakeRackInputs pInputs) {
        pInputs.iIsIntakeRackConnected = BaseStatusSignal.isAllGood(
            mIntakeRackRotation,
            mIntakeRackVelocityRPS,
            mIntakeRackAccelerationRPSS,
            mIntakeRackVoltage,
            mIntakeRackSupplyCurrent,
            mIntakeRackStatorCurrent,
            mIntakeRackTempCelsius,
            mIntakeRackReferencePosition,
            mIntakeRackReferencePositionSlope
        );
        pInputs.iIntakeRackPositionM = mIntakeRackRotation.getValueAsDouble();
        pInputs.iIntakeRackVelocityMS = mIntakeRackVelocityRPS.getValueAsDouble();
        pInputs.iIntakeRackMotorVolts = mIntakeRackVoltage.getValueAsDouble();
        pInputs.iIntakeRackSupplyCurrentAmps = mIntakeRackSupplyCurrent.getValueAsDouble();
        pInputs.iIntakeRackStatorCurrentAmps = mIntakeRackStatorCurrent.getValueAsDouble();
        pInputs.iIntakeRackTempCelsius = mIntakeRackTempCelsius.getValueAsDouble();
        pInputs.iIntakeClosedLoopReference = mIntakeRackReferencePosition.getValueAsDouble();
        pInputs.iIntakeClosedLoopReferenceSlope = mIntakeRackReferencePositionSlope.getValueAsDouble();
    }

    @Override 
    public void setPDConstants(double pKP, double pKD) {
        Slot0Configs slotConfig = new Slot0Configs();
        slotConfig.kP = pKP;
        slotConfig.kD = pKD;
        mIntakeRackMotor.getConfigurator().apply(slotConfig);
    }

    @Override 
    public void setMotionMagicConstants(double pCruiseVel, double pMaxAccel, double pMaxJerk) {
        MotionMagicConfigs motionMagicConfig = new MotionMagicConfigs();
        motionMagicConfig.MotionMagicCruiseVelocity = pCruiseVel;
        motionMagicConfig.MotionMagicAcceleration = pMaxAccel;
        motionMagicConfig.MotionMagicJerk = pMaxJerk;
        mIntakeRackMotor.getConfigurator().apply(motionMagicConfig);
    }

    @Override
    public void setMotorVolts(double pVolts) {
        mIntakeRackMotor.setControl(mIntakeRackVoltageControl.withOutput(pVolts));
    }

    @Override
    public void setMotorAmps(double pAmps){
        mIntakeRackMotor.setControl(mIntakeRackAmpsControl.withOutput(pAmps));
    }

    @Override
    public void resetPPID() {
        /* Does nothing at this point in time */
    }

    @Override
    public void setMotorPosition(double pPositionM, double feedforward) {
        mIntakeRackMotor.setControl(mIntakeRackRotationControl.withPosition(pPositionM).withFeedForward(feedforward));
    }

    @Override
    public void stopMotor() {
        mIntakeRackMotor.stopMotor();
    }

}