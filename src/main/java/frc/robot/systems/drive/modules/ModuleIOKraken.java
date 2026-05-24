// REBELLION 10014

package frc.robot.systems.drive.modules;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.systems.drive.DriveConstants.*;

import java.util.Queue;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.systems.drive.DriveConstants.ModuleHardwareConfig;
import frc.lib.PhoenixUtil;
import frc.lib.PhoenixUtil.CanivoreBus;
import frc.robot.systems.drive.PhoenixOdometryThread;

public class ModuleIOKraken implements ModuleIO {
    private final TalonFX mDriveMotor;
    private final VelocityTorqueCurrentFOC mDriveControl = 
        new VelocityTorqueCurrentFOC(0.0)
            .withUpdateFreqHz(0.0);
    private final VoltageOut mDriveVoltageControl = new VoltageOut(0.0);
    private final TorqueCurrentFOC mDriveAmpControl = new TorqueCurrentFOC(0.0);

    private final StatusSignal<Angle> mDrivePositionM;
    private final StatusSignal<AngularVelocity> mDriveVelocityMPS;
    private final StatusSignal<Voltage> mDriveVoltage;
    private final StatusSignal<Current> mDriveSupplyCurrent;
    private final StatusSignal<Current> mDriveStatorCurrent;
    // private final StatusSignal<Current> mDriveTorqueCurrent;
    private final StatusSignal<Temperature> mDriveTempCelsius;
    private final StatusSignal<AngularAcceleration> mDriveAccelerationMPSS;

    private final TalonFX mAzimuthMotor;
    // private final PositionDutyCycle mAzimuthPositionControl = new PositionDutyCycle(0.0)
    //     .withUpdateFreqHz(0.0);
    private final PositionTorqueCurrentFOC mAzimuthPositionControl = new PositionTorqueCurrentFOC(0.0);
    private final VoltageOut mAzimuthVoltageControl = new VoltageOut(0.0);
    private final TorqueCurrentFOC mAzimuthAmpControl = new TorqueCurrentFOC(0.0);

    private final StatusSignal<Angle> mAzimuthPosition;
    private final StatusSignal<AngularVelocity> mAzimuthVelocity;
    private final StatusSignal<Voltage> mAzimuthVoltage;
    private final StatusSignal<Current> mAzimuthStatorCurrent;
    private final StatusSignal<Current> mAzimuthSupplyCurrent;
    // private final StatusSignal<Current> mAzimuthTorqueCurrent;
    private final StatusSignal<Temperature> mAzimuthTemp;

    private final CANcoder mAbsoluteEncoder;
    private final StatusSignal<Angle> mAbsolutePositionSignal;

    private final Queue<Double> timestampQueue;
    private final Queue<Double> drivePositionQueue;
    private final Queue<Double> turnPositionQueue;

    public ModuleIOKraken(ModuleHardwareConfig pConfig) {
        mDriveMotor = new TalonFX(pConfig.driveID(), kCANBus);
        var driveConfig = new TalonFXConfiguration();

        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfig.CurrentLimits.StatorCurrentLimit = kDriveStatorAmpLimit;
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveConfig.CurrentLimits.SupplyCurrentLimit = kDriveSupplyAmpLimit;
        driveConfig.CurrentLimits.SupplyCurrentLowerTime = kDriveSupplyAmpLowerLimitTime;
        driveConfig.CurrentLimits.SupplyCurrentLowerLimit = kDriveSupplyAmpLowerLimit;

        driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = kDriveFOCAmpLimit;
        driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -kDriveFOCAmpLimit;
        driveConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;

        driveConfig.Voltage.PeakForwardVoltage = kPeakVoltage;
        driveConfig.Voltage.PeakReverseVoltage = -kPeakVoltage;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        driveConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        driveConfig.Feedback.SensorToMechanismRatio = kDriveMotorGearing / kWheelCircumferenceMeters;

        driveConfig.Slot0.kP = kModuleControllerConfigs.driveController().getP();
        driveConfig.Slot0.kI = kModuleControllerConfigs.driveController().getI();
        driveConfig.Slot0.kD = kModuleControllerConfigs.driveController().getD();

        driveConfig.Slot1.kP = kDriveAggressiveP;
        driveConfig.Slot1.kI = kModuleControllerConfigs.driveController().getI();
        driveConfig.Slot1.kD = kModuleControllerConfigs.driveController().getD();

        mDrivePositionM = mDriveMotor.getPosition();
        mDriveVelocityMPS = mDriveMotor.getVelocity();
        mDriveVoltage = mDriveMotor.getMotorVoltage();
        mDriveSupplyCurrent = mDriveMotor.getSupplyCurrent();
        mDriveStatorCurrent = mDriveMotor.getStatorCurrent();
        // mDriveTorqueCurrent = mDriveMotor.getTorqueCurrent();
        mDriveTempCelsius = mDriveMotor.getDeviceTemp();
        mDriveAccelerationMPSS = mDriveMotor.getAcceleration();

        mDriveMotor.getConfigurator().apply(driveConfig);
        
        mAbsoluteEncoder = new CANcoder(pConfig.encoderID(), kCANBus);
        
        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(0.5));
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        encoderConfig.MagnetSensor.withMagnetOffset(Rotations.of(-pConfig.offset()));
        mAbsoluteEncoder.getConfigurator().apply(encoderConfig);
        
        mAbsolutePositionSignal = mAbsoluteEncoder.getAbsolutePosition();

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, mAbsolutePositionSignal);
        mAbsoluteEncoder.optimizeBusUtilization(0.0);

        /* AZIMUTH INSTANTIATION AND CONFIGURATION */
        mAzimuthMotor = new TalonFX(pConfig.azimuthID(), kCANBus);
        TalonFXConfiguration turnConfig = new TalonFXConfiguration();

        turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        turnConfig.CurrentLimits.StatorCurrentLimit = kAzimuthStatorAmpLimit;
        turnConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        turnConfig.CurrentLimits.SupplyCurrentLimit = kAzimuthSupplyAmpLimit;
        turnConfig.TorqueCurrent.PeakForwardTorqueCurrent = kAzimuthFOCAmpLimit;
        turnConfig.TorqueCurrent.PeakReverseTorqueCurrent = -kAzimuthFOCAmpLimit;
        turnConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;

        turnConfig.Voltage.PeakForwardVoltage = kPeakVoltage;
        turnConfig.Voltage.PeakReverseVoltage = -kPeakVoltage;
        turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        turnConfig.MotorOutput.Inverted = kTurnMotorInvert ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
        turnConfig.Feedback.FeedbackRemoteSensorID = mAbsoluteEncoder.getDeviceID();
        turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        turnConfig.Feedback.SensorToMechanismRatio = kCANCoderToMechanismRatio;
        turnConfig.Feedback.RotorToSensorRatio = kAzimuthMotorGearing;
        turnConfig.Slot0.kP = kModuleControllerConfigs.azimuthController().getP();
        turnConfig.Slot0.kD = kModuleControllerConfigs.azimuthController().getD();
        turnConfig.ClosedLoopGeneral.ContinuousWrap = true;

        turnConfig.TorqueCurrent.PeakForwardTorqueCurrent = kAzimuthFOCAmpLimit;
        turnConfig.TorqueCurrent.PeakReverseTorqueCurrent = -kAzimuthFOCAmpLimit;

        mAzimuthMotor.getConfigurator().apply(turnConfig);

        mAzimuthPosition = mAzimuthMotor.getPosition();
        mAzimuthVelocity = mAzimuthMotor.getVelocity();
        mAzimuthVoltage = mAzimuthMotor.getMotorVoltage();
        mAzimuthStatorCurrent = mAzimuthMotor.getStatorCurrent();
        mAzimuthSupplyCurrent = mAzimuthMotor.getSupplyCurrent();
        // mAzimuthTorqueCurrent = mAzimuthMotor.getTorqueCurrent();
        mAzimuthTemp = mAzimuthMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
            kOdometryFrequency, 
            mDrivePositionM, 
            mAzimuthPosition);

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0, 
            mDriveVelocityMPS,
            mDriveStatorCurrent,
            mDriveSupplyCurrent,
            mDriveTempCelsius,
            // mDriveTorqueCurrent,
            mDriveVoltage,
            mDriveAccelerationMPSS,
            mAzimuthVelocity,
            mAzimuthStatorCurrent,
            mAzimuthSupplyCurrent,
            mAzimuthVoltage,
            mAzimuthTemp,
            mAbsolutePositionSignal);

        mDriveMotor.optimizeBusUtilization(0.0);
        mAzimuthMotor.optimizeBusUtilization(0.0);
        mAbsoluteEncoder.optimizeBusUtilization(0.0);

        timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
        turnPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(mAzimuthPosition.clone());
        drivePositionQueue = PhoenixOdometryThread.getInstance().registerSignal(mDrivePositionM.clone());

        PhoenixUtil.registerSignals(
            CanivoreBus.UNDERWORLD, 
            mDrivePositionM,
            mDriveVelocityMPS,
            mDriveStatorCurrent,
            mDriveSupplyCurrent,
            mDriveTempCelsius,
            // mDriveTorqueCurrent,
            mDriveVoltage,
            mDriveAccelerationMPSS,
            mAzimuthPosition,
            mAzimuthVelocity,
            mAzimuthStatorCurrent,
            mAzimuthSupplyCurrent,
            mAzimuthVoltage,
            mAzimuthTemp,
            mAbsolutePositionSignal);
    }

    @Override
    public void updateInputs(ModuleInputs pInputs) {
        pInputs.iIsDriveConnected = BaseStatusSignal.isAllGood(
            mDriveVelocityMPS,
            mDrivePositionM,
            mDriveVoltage,
            mDriveSupplyCurrent,
            mDriveStatorCurrent,
            // mDriveTorqueCurrent,
            mDriveTempCelsius,
            mDriveAccelerationMPSS);
        pInputs.iDrivePositionM = (mDrivePositionM.getValueAsDouble());
        pInputs.iDriveVelocityMPS = (mDriveVelocityMPS.getValueAsDouble());
        pInputs.iDriveMotorVolts = mDriveVoltage.getValueAsDouble();
        pInputs.iDriveSupplyCurrentAmps = mDriveSupplyCurrent.getValueAsDouble();
        pInputs.iDriveStatorCurrentAmps = mDriveStatorCurrent.getValueAsDouble();
        // pInputs.iDriveTorqueCurrentAmps = mDriveTorqueCurrent.getValueAsDouble();
        pInputs.iDriveTemperatureCelsius = mDriveTempCelsius.getValueAsDouble();

        pInputs.iIsAzimuthConnected = BaseStatusSignal.isAllGood(
            mAzimuthVelocity,
            mAzimuthVoltage,
            mAzimuthStatorCurrent,
            mAzimuthSupplyCurrent,
            mAzimuthTemp,
            mAzimuthPosition
            // mAzimuthTorqueCurrent
        );
        pInputs.iAzimuthPosition = Rotation2d.fromRotations(mAzimuthPosition.getValueAsDouble());
        pInputs.iAzimuthVelocity = Rotation2d.fromRotations(mAzimuthVelocity.getValueAsDouble());
        pInputs.iAzimuthMotorVolts = mAzimuthVoltage.getValueAsDouble();
        pInputs.iAzimuthStatorCurrentAmps = mAzimuthStatorCurrent.getValueAsDouble();
        pInputs.iAzimuthSupplyCurrentAmps = mAzimuthSupplyCurrent.getValueAsDouble();
        // pInputs.iAzimuthTorqueCurrentAmps = mAzimuthTorqueCurrent.getValueAsDouble();
        pInputs.iAzimuthTemperatureCelsius = mAzimuthTemp.getValueAsDouble();

        pInputs.iIsCancoderConnected = BaseStatusSignal.isAllGood(mAbsolutePositionSignal);
        pInputs.iAzimuthAbsolutePosition = Rotation2d.fromRotations(mAbsolutePositionSignal.getValueAsDouble());

        pInputs.odometryTimestamps =
            timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        pInputs.odometryDrivePositionsM =
            drivePositionQueue.stream()
                .mapToDouble((Double value) -> value)
                .toArray();
        pInputs.odometryTurnPositions =
            turnPositionQueue.stream()
                .map((Double value) -> Rotation2d.fromRotations(value))
                .toArray(Rotation2d[]::new);
        timestampQueue.clear();
        drivePositionQueue.clear();
        turnPositionQueue.clear();
    }

    /////////// DRIVE MOTOR METHODS \\\\\\\\\\\
    @Override
    public void setDriveVolts(double pVolts) {
        mDriveMotor.setControl(mDriveVoltageControl.withOutput(pVolts));
    }

    @Override
    public void setDriveAmperage(double pAmps) {
        mDriveMotor.setControl(mDriveAmpControl.withOutput(pAmps));
    }

    @Override
    public void setDriveVelocity(double pVelocityMPS, double pFeedforward, int slot) {
        mDriveMotor.setControl(
            mDriveControl
                .withVelocity(pVelocityMPS)
                .withFeedForward(pFeedforward)
                .withSlot(slot));
    }

    @Override
    public void setDrivePID(double pKP, double pKI, double pKD, int slot) {
        switch (slot) {
            case 0:
                var slotConfig0 = new Slot0Configs();
                slotConfig0.kP = pKP;
                slotConfig0.kI = pKI;
                slotConfig0.kD = pKD;
                mDriveMotor.getConfigurator().apply(slotConfig0);

                break;

            case 1:
                var slotConfig1 = new Slot1Configs();

                slotConfig1.kP = pKP;
                slotConfig1.kI = pKI;
                slotConfig1.kD = pKD;
                mDriveMotor.getConfigurator().apply(slotConfig1);

                break;

            case 2:
                var slotConfig2 = new Slot2Configs();

                slotConfig2.kP = pKP;
                slotConfig2.kI = pKI;
                slotConfig2.kD = pKD;
                mDriveMotor.getConfigurator().apply(slotConfig2);

                break;
        
            default:
                return;
        }
    }

    /////////// CANCODER METHODS \\\\\\\\\\\
    @Override
    public void resetAzimuthEncoder() {
        return;
    }

    /////////// AZIMUTH MOTOR METHODS \\\\\\\\\\\
    @Override
    public void setAzimuthVolts(double pVolts) {
        /* Sets azimuth voltage inbetween kPeakVoltage and -kPeakVoltage */
        mAzimuthMotor.setControl(mAzimuthVoltageControl.withOutput(pVolts));
    }

    @Override
    public void setAzimuthPosition(Rotation2d pRotation, double pFeedforward) {
        /* Uses voltage PID with a arbitrary FF on the with Slot 0 gains */
        mAzimuthMotor.setControl(mAzimuthPositionControl
                .withPosition(pRotation.getRotations())
                .withFeedForward(pFeedforward));
    }

    @Override
    public void setAzimuthAmps(double amps) {
        mAzimuthMotor.setControl(mAzimuthAmpControl.withOutput(amps));
    }

    /* Sets azimuth PID values on slot 0, used for tunable numbers */
    @Override
    public void setAzimuthPID(double pKP, double pKI, double pKD) {
        var slotConfig = new Slot0Configs();
        slotConfig.kP = pKP;
        slotConfig.kI = pKI;
        slotConfig.kD = pKD;
        mAzimuthMotor.getConfigurator().apply(slotConfig);
    }
}
