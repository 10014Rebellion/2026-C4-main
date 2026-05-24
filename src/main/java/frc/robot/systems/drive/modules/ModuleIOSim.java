// REBELLION 10014

package frc.robot.systems.drive.modules;

import static frc.robot.systems.drive.DriveConstants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ModuleIOSim implements ModuleIO {
    private final DCMotorSim mDriveMotor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.004, kDriveMotorGearing),
        DCMotor.getKrakenX60Foc(1).withReduction(kDriveMotorGearing),
        0.0,
        0.0
    );

    private final DCMotorSim mAzimuthMotor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.025, kAzimuthMotorGearing),
        DCMotor.getKrakenX60Foc(1).withReduction(kAzimuthMotorGearing),
        0.0,
        0.0
    );

    private double mDriveAppliedVolts = 0.0;
    private double mAzimuthAppliedVolts = 0.0;

    private final PIDController mDrivePID = kModuleControllerConfigs.driveController();
    private final PIDController mAzimuthPID = kModuleControllerConfigs.azimuthController();

    public ModuleIOSim() {
        mAzimuthPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateInputs(ModuleInputs pInputs) {
        mDriveMotor.update(0.02);
        mAzimuthMotor.update(0.02);

        pInputs.iDrivePositionM = mDriveMotor.getAngularPositionRotations() * kWheelCircumferenceMeters;
        pInputs.iDriveVelocityMPS = (mDriveMotor.getAngularVelocityRPM() * kWheelCircumferenceMeters) / 60.0;
        pInputs.iDriveStatorCurrentAmps = Math.abs(mDriveMotor.getCurrentDrawAmps());
        pInputs.iDriveTemperatureCelsius = 0.0;
        pInputs.iDriveMotorVolts = mDriveAppliedVolts;

        pInputs.iAzimuthAbsolutePosition = new Rotation2d(mAzimuthMotor.getAngularPositionRad());
        pInputs.iAzimuthPosition = new Rotation2d(mAzimuthMotor.getAngularPositionRad());
        pInputs.iAzimuthVelocity = Rotation2d.fromRadians(mAzimuthMotor.getAngularVelocityRadPerSec());
        pInputs.iAzimuthStatorCurrentAmps = Math.abs(mAzimuthMotor.getCurrentDrawAmps());
        pInputs.iAzimuthTemperatureCelsius = 0.0;
        pInputs.iAzimuthMotorVolts = mAzimuthAppliedVolts;

        pInputs.odometryTimestamps = new double[] {((double) Logger.getTimestamp()) / 1.0e6};
        pInputs.odometryDrivePositionsM = new double[] {
            mDriveMotor.getAngularPositionRotations() * kWheelCircumferenceMeters
        };
        pInputs.odometryTurnPositions = new Rotation2d[] {
            new Rotation2d(mAzimuthMotor.getAngularPositionRad())
        };
    }

    /////////// DRIVE MOTOR METHODS \\\\\\\\\\\
    @Override
    public void setDriveVolts(double pVolts) {
        mDriveAppliedVolts = pVolts;
        mDriveMotor.setInputVoltage(mDriveAppliedVolts);
    }

    @Override
    public void setDriveVelocity(double pVelocityMPS, double pFeedforward, int slot) {
        setDriveVolts(mDrivePID.calculate(mDriveMotor.getAngularVelocityRPM() * kWheelCircumferenceMeters / 60.0, pVelocityMPS) + pFeedforward);
    }

    @Override
    public void setDrivePID(double pKP, double pKI, double pKD, int slot) {
        mDrivePID.setPID(pKP, pKI, pKD);
    }

    @Override
    public void resetAzimuthEncoder() {
        return;
    }

    /////////// AZIMUTH MOTOR METHODS \\\\\\\\\\\
    @Override
    public void setAzimuthVolts(double pVolts) {
        mAzimuthAppliedVolts = MathUtil.clamp(pVolts, -12.0, 12.0);
        mAzimuthMotor.setInputVoltage(mAzimuthAppliedVolts);
    }

    @Override
    public void setAzimuthPosition(Rotation2d pPosition, double pFeedforward) {
        setAzimuthVolts(mAzimuthPID.calculate(mAzimuthMotor.getAngularPositionRad(), pPosition.getRadians()) + pFeedforward);
    }

    @Override
    public void setAzimuthPID(double pKP, double pKI, double pKD) {
        mAzimuthPID.setPID(pKP, pKI, pKD);
    }
}
