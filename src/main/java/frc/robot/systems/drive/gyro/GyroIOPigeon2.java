// REBELLION 10014

package frc.robot.systems.drive.gyro;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import java.util.Queue;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import frc.lib.PhoenixUtil;
import frc.lib.PhoenixUtil.CanivoreBus;
import frc.robot.systems.drive.DriveConstants;
import frc.robot.systems.drive.PhoenixOdometryThread;

public class GyroIOPigeon2 implements GyroIO {
    private final Pigeon2 mGyro = new Pigeon2(DriveConstants.kPigeonCANID, DriveConstants.kCANBus);
    private final StatusSignal<Angle> mYaw = mGyro.getYaw();
    private final StatusSignal<Angle> mPitchPosition = mGyro.getPitch();
    private final StatusSignal<Angle> mRollPosition = mGyro.getRoll();
    private final StatusSignal<AngularVelocity> mYawVelocity = mGyro.getAngularVelocityXWorld();
    private final StatusSignal<LinearAcceleration> mYawAccelerationX = mGyro.getAccelerationX();
    private final StatusSignal<LinearAcceleration> mYawAccelerationY = mGyro.getAccelerationY();
    private final StatusSignal<LinearAcceleration> mYawAccelerationZ = mGyro.getAccelerationZ();

    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> yawTimestampQueue;

    public GyroIOPigeon2() {
        mGyro.getConfigurator().apply(new Pigeon2Configuration());
        mGyro.getConfigurator().setYaw(0.0);

        BaseStatusSignal.setUpdateFrequencyForAll(DriveConstants.kOdometryFrequency, mYaw);
        BaseStatusSignal.setUpdateFrequencyForAll(50, 
            mPitchPosition, 
            mRollPosition, 
            mYawVelocity, 
            mYawAccelerationX, 
            mYawAccelerationY, 
            mYawAccelerationZ);

        yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
        yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(mYaw.clone());

        mGyro.optimizeBusUtilization(0.0);

        PhoenixUtil.registerSignals(
            CanivoreBus.UNDERWORLD, 
            mYaw,
            mPitchPosition, 
            mRollPosition, 
            mYawVelocity, 
            mYawAccelerationX, 
            mYawAccelerationY, 
            mYawAccelerationZ);
    }

    @Override
    public void updateInputs(GyroInputs pInputs) {
        pInputs.iConnected = BaseStatusSignal.isAllGood(
            mYaw, 
            mYawVelocity,
            mYawAccelerationX,
            mYawAccelerationY,
            mYawAccelerationZ);
        pInputs.iYawPosition = Rotation2d.fromDegrees(mYaw.getValueAsDouble());
        pInputs.iPitchPosition = Rotation2d.fromDegrees(mPitchPosition.getValue().in(Degrees));
        pInputs.iRollPosition = Rotation2d.fromDegrees(mRollPosition.getValue().in(Degrees));
        pInputs.iYawVelocityPS = Rotation2d.fromDegrees(mYawVelocity.getValue().in(DegreesPerSecond));
        pInputs.iAccXG = mYawAccelerationX.getValueAsDouble();
        pInputs.iAccYG = mYawAccelerationY.getValueAsDouble();
        pInputs.iAccZG = mYawAccelerationZ.getValueAsDouble();

        pInputs.odometryYawTimestamps =
            yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        pInputs.odometryYawPositions =
            yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
        
        yawTimestampQueue.clear();
        yawPositionQueue.clear();
    }

    @Override
    public void resetGyro(Rotation2d pRotation) {
        mGyro.setYaw(pRotation.getDegrees());
    }
}
