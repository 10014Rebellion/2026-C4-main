package frc.robot.systems.shooter.flywheels.encoder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.lib.PhoenixUtil;
import frc.lib.PhoenixUtil.CanivoreBus;
import frc.lib.hardware.HardwareRecords.RelativeCANCoderHardware;
import frc.robot.RobotConstants;

public class EncoderIOCANCoder implements EncoderIO{
    private final CANcoder mCANCoder;
    private final StatusSignal<Angle> mPosition;
    private final StatusSignal<AngularVelocity> mVelocity;
    private final StatusSignal<MagnetHealthValue> mMagnetHealth;

    public EncoderIOCANCoder(RelativeCANCoderHardware pCANCoderConfig) {
        this.mCANCoder = new CANcoder(pCANCoderConfig.cancoderID(), RobotConstants.kSubsystemsCANBus);

        CANcoderConfiguration mCTRConfigCANCoder = new CANcoderConfiguration();
        mCTRConfigCANCoder.MagnetSensor.SensorDirection = pCANCoderConfig.direction();
        this.mCANCoder.getConfigurator().apply(mCTRConfigCANCoder);

        mCANCoder.setPosition(0);
        mPosition = mCANCoder.getPosition();
        mVelocity = mCANCoder.getVelocity();
        mMagnetHealth = mCANCoder.getMagnetHealth();        

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            mPosition,
            mVelocity,
            mMagnetHealth
        );

        mCANCoder.optimizeBusUtilization(0.0);

        PhoenixUtil.registerSignals(
            CanivoreBus.OVERWORLD, 
            mPosition,
            mVelocity,
            mMagnetHealth);
    }

    @Override
    public void updateInputs(EncoderInputs pInputs) {
        pInputs.iIsEncoderConnected = BaseStatusSignal.isAllGood(
            mPosition,
            mVelocity,
            mMagnetHealth
        );
        pInputs.iEncoderPositionRot = Rotation2d.fromRotations(mPosition.getValueAsDouble());
        pInputs.iEncoderVelocityRPS = Rotation2d.fromRotations(mVelocity.getValueAsDouble());
        pInputs.iEncoderMagnetHealth = mMagnetHealth.getValue().toString();
    }

    @Override
    public void setPosition(Rotation2d pNewRot) {
        mCANCoder.setPosition(pNewRot.getRotations());
    }
}