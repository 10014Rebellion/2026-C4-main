package frc.robot.systems.shooter.hood;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.telemetry.Telemetry;
import frc.lib.tuning.LoggedTunableNumber;
import frc.robot.systems.efi.sensors.CANRangeSS;
import frc.robot.systems.shooter.ShotMap;

public class HoodSS extends SubsystemBase {
    public static enum HoodStates {
        STOPPED,
        TUNING_VOLTAGE,
        TUNING_AMPS,
        INCREMENTING,
        DECREMENTING,
        HOLD_POSITION,
        SHOTMAP_POSITION,
        STEP_INCREMENT,
        STEP_DECREMENT,
        OPPONENT_FEED_ANGLE,

        // CONSTANT SETPOINTS
        TUNING_SETPOINT,
        MAX,
        MID,
        MIN,
        TOWER_ANGLE,
        TRENCH_ANGLE,
        BUMP_ANGLE,
        CORNER_ANGLE
    }

    private final HoodIO mHoodIO;
    private final ArmFeedforward mHoodFF;
    private final HoodInputsAutoLogged mHoodInputs = new HoodInputsAutoLogged();

    private final LoggedTunableNumber tHoodKP = new LoggedTunableNumber("Shooter/Hood/Control/PID/kP",
            HoodConstants.kHoodControlConfig.pdController().kP());
    private final LoggedTunableNumber tHoodKD = new LoggedTunableNumber("Shooter/Hood/Control/PID/kD",
            HoodConstants.kHoodControlConfig.pdController().kD());
    private final LoggedTunableNumber tHoodKS = new LoggedTunableNumber("Shooter/Hood/Control/FF/kS",
            HoodConstants.kHoodControlConfig.feedforward().getKs());
    private final LoggedTunableNumber tHoodKG = new LoggedTunableNumber("Shooter/Hood/Control/FF/kG",
            HoodConstants.kHoodControlConfig.feedforward().getKg());
    private final LoggedTunableNumber tHoodKV = new LoggedTunableNumber("Shooter/Hood/Control/FF/kV",
            HoodConstants.kHoodControlConfig.feedforward().getKv());
    private final LoggedTunableNumber tHoodKA = new LoggedTunableNumber("Shooter/Hood/Control/FF/kA",
            HoodConstants.kHoodControlConfig.feedforward().getKa());
    private final LoggedTunableNumber tHoodCruiseVelDegS = new LoggedTunableNumber(
            "Shooter/Hood/Control/Profile/CruiseVelDegS",
            HoodConstants.kHoodControlConfig.motionMagicConstants().maxVelocity() * 360);
    private final LoggedTunableNumber tHoodMaxAccelDegS2 = new LoggedTunableNumber(
            "Shooter/Hood/Control/Profile/MaxAccelerationDegs2",
            HoodConstants.kHoodControlConfig.motionMagicConstants().maxAcceleration() * 360);
    private final LoggedTunableNumber tHoodMaxJerkDegS3 = new LoggedTunableNumber(
            "Shooter/Hood/Control/Profile/MaxJerks3",
            HoodConstants.kHoodControlConfig.motionMagicConstants().maxJerk() * 360);
    private final LoggedTunableNumber tHoodToleranceDegrees = new LoggedTunableNumber("Shooter/Hood/Control/Tolerance",
            HoodConstants.kTolerance.getDegrees());

    @AutoLogOutput(key = "Shooter/Hood/States/CurrentState")
    private HoodStates mCurrentHoodState = HoodStates.STOPPED;

    @AutoLogOutput(key = "Shooter/Hood/LatestClosedLoopGoalRot")
    private Rotation2d mLatestClosedLoopGoalRot = Rotation2d.kZero;
    private int mDesiredDirection = 0;

    @AutoLogOutput(key = "Shooter/Hood/LimitsEnforced")
    private boolean mLimitEnforced = false;
    private boolean mShouldUseCanRanges = false;
    private final CANRangeSS mCANRanges;

    public HoodSS(HoodIO pHoodIO, CANRangeSS pCANRange) {
        this.mHoodIO = pHoodIO;
        this.mCANRanges = pCANRange;
        this.mHoodFF = HoodConstants.kHoodControlConfig.feedforward();
    }

    @Override
    public void periodic() {
        mHoodIO.updateInputs(mHoodInputs);

        refreshTuneables();
        executeState();

        Logger.processInputs("Hood", mHoodInputs);
    }

    public void setCANRangeUsage(boolean pShouldUseCANRange) {
        mShouldUseCanRanges = pShouldUseCANRange;
    }

    /*
     * Performs variable updates or parameter intializations when a state is set,
     * SHOULD NOT CHANGE THE STATE THROUGH HERE.
     */
    @SuppressWarnings("incomplete-switch")
    private void initializeState(HoodStates pStateToInit) {
        switch (pStateToInit) {
            case HOLD_POSITION -> {
                setHoodPosition(mHoodInputs.iHoodAngle);
            }
            case STEP_INCREMENT -> {
                setHoodPosition(mHoodInputs.iHoodAngle.plus(HoodConstants.kAdjustStepAmount));
            }
            case STEP_DECREMENT -> {
                setHoodPosition(mHoodInputs.iHoodAngle.minus(HoodConstants.kAdjustStepAmount));
            }
        }
    }

    /*
     * Runs actions periodically to execute state, SHOULD NOT CHANGE THE STATE
     * THROUGH HERE.
     */
    @SuppressWarnings("incomplete-switch")
    private void executeState() {
        if (HoodConstants.kStateToSetpointMapHood.containsKey(mCurrentHoodState)) {
            setHoodPosition(HoodConstants.kStateToSetpointMapHood.get(mCurrentHoodState).get());
        } else {
            switch (mCurrentHoodState) {
                case STOPPED -> {
                    mHoodIO.stopMotor();
                }
                case TUNING_VOLTAGE -> {
                    setHoodVoltage(HoodConstants.tTuningVoltage.get());
                }
                case TUNING_AMPS -> {
                    setHoodAmps(HoodConstants.tTuningAmp.get());
                }
                case INCREMENTING -> {
                    setHoodPosition(mHoodInputs.iHoodAngle.plus(Rotation2d.fromDegrees(1)));
                }
                case DECREMENTING -> {
                    setHoodPosition(mHoodInputs.iHoodAngle.minus(Rotation2d.fromDegrees(1)));
                }
                case HOLD_POSITION -> {
                    setHoodPosition(mLatestClosedLoopGoalRot);
                }
                case SHOTMAP_POSITION -> {
                    setHoodPosition(Rotation2d.fromDegrees(
                        ShotMap.getInstance().getHoodAngle().getDegrees() * 
                        (mShouldUseCanRanges && mCANRanges.allHasFuel() ? 1.0 : 1.0)
                    ));
                }
            }
        }
    }

    public Command setStateCmd(HoodStates pNewState) {
        return setStateCmd(pNewState, true);
    }

    public Command setStateCmd(HoodStates pNewState, boolean holdRequirementContinuously) {
        return new FunctionalCommand(
                () -> setState(pNewState),
                () -> {
                }, (interrupted) -> {
                },
                () -> !holdRequirementContinuously,
                this);
    }

    private void setState(HoodStates pNewState) {
        mCurrentHoodState = pNewState;
        initializeState(pNewState);
    }

    private void setHoodPosition(Rotation2d pRot) {
        Telemetry.log("Shooter/Hood/Setpoint/NonLimited", pRot);

        pRot = clampRotToSoftLimits(pRot);

        Telemetry.log("Shooter/Hood/Setpoint/Limited", pRot);

        mLatestClosedLoopGoalRot = pRot;

        double ffOutput = mHoodFF.calculate(
                mHoodInputs.iHoodAngle.getRadians(),
                mHoodInputs.iHoodReferenceValueSlope.getRadians());

        double cos = mHoodInputs.iHoodReferenceValue.getCos();

        Telemetry.log("Shooter/Hood/ffOutput", ffOutput);
        Telemetry.log("Shooter/Hood/ffCos", cos);

        mHoodIO.setMotorPosition(pRot, ffOutput);

        mDesiredDirection = toDirection(getErrorRot());
        enforceSoftLimits();
    }

    private void setHoodVoltage(double pVolts) {
        mDesiredDirection = toDirection(pVolts);
        mHoodIO.setMotorVolts(pVolts);
        enforceSoftLimits();
    }

    private void setHoodAmps(double pAmps) {
        mDesiredDirection = toDirection(pAmps);
        mHoodIO.setMotorAmps(pAmps);
        enforceSoftLimits();
    }

    private void enforceSoftLimits() {
        if ((mHoodInputs.iHoodAngle.getRotations() > HoodConstants.kHoodLimits.forwardLimit().getRotations()
                && mDesiredDirection == 1)
                ||
                (mHoodInputs.iHoodAngle.getRotations() < HoodConstants.kHoodLimits.backwardLimit().getRotations()
                        && mDesiredDirection == -1)) {
            mLimitEnforced = true;
            mHoodIO.stopMotor();
        } else {
            mLimitEnforced = false;
        }
    }

    public int toDirection(double val) {
        if (val > 0)
            return 1;
        if (val < 0)
            return -1;
        else
            return 0;
    }

    private void setFF(double kS, double kG, double kV, double kA) {
        mHoodFF.setKs(kS);
        mHoodFF.setKg(kG);
        mHoodFF.setKv(kV);
        mHoodFF.setKa(kA);
    }

    public HoodStates getHoodState() {
        return mCurrentHoodState;
    }

    @AutoLogOutput(key = "Shooter/Hood/Feedback/ErrorRotation")
    public double getErrorRot() {
        return getCurrentGoal().getRotations() - mHoodInputs.iHoodAngle.getRotations();
    }

    @AutoLogOutput(key = "Shooter/Hood/Feedback/CurrentGoal")
    public Rotation2d getCurrentGoal() {
        return mLatestClosedLoopGoalRot;
    }

    @AutoLogOutput(key = "Shooter/Hood/Feedback/AtGoal")
    public boolean atGoal() {
        return Math.abs(getErrorRot()) <= (tHoodToleranceDegrees.get() / 360.0);
    }

    private Rotation2d clampRotToSoftLimits(Rotation2d pRotToClamp) {
        return Rotation2d.fromRotations(
                MathUtil.clamp(
                        pRotToClamp.getRotations(),
                        HoodConstants.kHoodLimits.backwardLimit().getRotations(),
                        HoodConstants.kHoodLimits.forwardLimit().getRotations()));
    }

    private void refreshTuneables() {
        LoggedTunableNumber.ifChanged(hashCode(),
                () -> mHoodIO.setPDConstants(tHoodKP.get(), tHoodKD.get()),
                tHoodKP, tHoodKD);

        LoggedTunableNumber.ifChanged(hashCode(),
                () -> setFF(tHoodKS.get(), tHoodKG.get(), tHoodKV.get(), tHoodKA.get()),
                tHoodKS, tHoodKG, tHoodKV, tHoodKA);

        // Default unit is rotations but the actual values are in degrees. This is to
        // preserve units //
        LoggedTunableNumber.ifChanged(hashCode(),
                () -> mHoodIO.setMotionMagicConstants(
                        tHoodCruiseVelDegS.get() / 360.0,
                        tHoodMaxAccelDegS2.get() / 360.0,
                        tHoodMaxJerkDegS3.get() / 360.0),
                tHoodCruiseVelDegS, tHoodMaxAccelDegS2, tHoodMaxJerkDegS3);
    }
}
