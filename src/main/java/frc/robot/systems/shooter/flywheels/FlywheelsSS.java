package frc.robot.systems.shooter.flywheels;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.telemetry.Telemetry;
import frc.lib.tuning.LoggedTunableNumber;
import frc.robot.logging.InvalidValueErrors.UnaccountedEnum;
import frc.robot.systems.efi.sensors.CANRangeSS;
import frc.robot.systems.shooter.ShotMap;
import frc.robot.systems.shooter.flywheels.encoder.EncoderIO;
import frc.robot.systems.shooter.flywheels.encoder.EncoderInputsAutoLogged;

import static frc.robot.systems.shooter.flywheels.FlywheelConstants.kFlywheelControlConfig;
import static frc.robot.systems.shooter.flywheels.FlywheelConstants.tTuningAmperage;

public class FlywheelsSS extends SubsystemBase {
  public static enum FlywheelStates {
    STOPPED,
    STANDBY_VOLTAGE,
    REV_VOLTAGE,
    TUNING_VOLTAGE,
    MAX_VOLTAGE,
    TUNING_VELOCITY,
    FEED_VELOCITY,
    OPPONENT_FEED_VELOCITY,
    HAILSTORM_VOLTAGE,
    MAX_VELOCITY,
    SHOTMAP_VELOCITY,
    STANDBY_VELOCITY,
    BOOST_SHOTMAP_VELOCITY,
    TOWER_VELOCITY,
    BUMP_VELOCITY,
    CORNER_VELOCITY,
    TRENCH_VELOCITY,
    TUNING_AMPERAGE
  }

  private final FlywheelIO mLeaderFlywheelIO;
  private final FlywheelIO mFollowerFlywheelIO;
  private final EncoderIO mFlywheelEncoder;

  private final FlywheelInputsAutoLogged mLeaderFlywheelInputs = new FlywheelInputsAutoLogged();
  private final FlywheelInputsAutoLogged mFollowerFlywheelInputs = new FlywheelInputsAutoLogged();
  private final EncoderInputsAutoLogged mEncoderInputs = new EncoderInputsAutoLogged();

  private final LoggedTunableNumber tFlywheelKP = new LoggedTunableNumber("Shooter/Flywheel/Control/PID/kP",
      kFlywheelControlConfig.pdController().kP());
  private final LoggedTunableNumber tFlywheelKD = new LoggedTunableNumber("Shooter/Flywheel/Control/PID/kD",
      kFlywheelControlConfig.pdController().kD());
  private final LoggedTunableNumber tFlywheelKS = new LoggedTunableNumber("Shooter/Flywheel/Control/FF/kS",
      kFlywheelControlConfig.feedforward().getKs());
  private final LoggedTunableNumber tFlywheelKV = new LoggedTunableNumber("Shooter/Flywheel/Control/FF/kV",
      kFlywheelControlConfig.feedforward().getKv());
  private final LoggedTunableNumber tFlywheelKA = new LoggedTunableNumber("Shooter/Flywheel/Control/FF/kA",
      kFlywheelControlConfig.feedforward().getKa());
  private final LoggedTunableNumber tFlywheelMaxVel = new LoggedTunableNumber("Shooter/Flywheel/Control/Profile/MaxVel",
      kFlywheelControlConfig.motionMagicConstants().maxVelocity());
  private final LoggedTunableNumber tFlywheelMaxAccel = new LoggedTunableNumber(
      "Shooter/Flywheel/Control/Profile/MaxAccel", kFlywheelControlConfig.motionMagicConstants().maxAcceleration());
  private final LoggedTunableNumber tFlywheelMaxJerk = new LoggedTunableNumber(
      "Shooter/Flywheel/Control/Profile/MaxJerk", kFlywheelControlConfig.motionMagicConstants().maxJerk());
  private final LoggedTunableNumber tFlywheelTolerance = new LoggedTunableNumber("Shooter/Flywheel/Control/Tolerance",
      FlywheelConstants.kToleranceRPS);
  private final LoggedTunableNumber tBoostFactor = new LoggedTunableNumber("Shooter/Flywheel/BoostFactor", 1.055);

  private Rotation2d mLastestClosedLoopGoalRPS = Rotation2d.kZero;

  @AutoLogOutput(key = "Shooter/Flywheel/States/CurrentState")
  private FlywheelStates mCurrentFlywheelState = FlywheelStates.STOPPED;
  private CANRangeSS mCanRangeSS;
  private boolean mShouldUseCanRanges = false;
  private final LoggedTunableNumber tInitialBoostFactor = new LoggedTunableNumber("Shooter/Flywheel/Boost", 1.0);

  public FlywheelsSS(FlywheelIO pLeaderFlywheelIO, FlywheelIO pFollowerFlywheelIO, CANRangeSS pCANRanges, EncoderIO pFlywheelEncoder) {
    this.mLeaderFlywheelIO = pLeaderFlywheelIO;
    this.mFollowerFlywheelIO = pFollowerFlywheelIO;
    this.mCanRangeSS = pCANRanges;
    this.mFlywheelEncoder = pFlywheelEncoder;
  }

  @Override
  public void periodic() {
    mLeaderFlywheelIO.updateInputs(mLeaderFlywheelInputs);
    mFollowerFlywheelIO.updateInputs(mFollowerFlywheelInputs);
    mFlywheelEncoder.updateInputs(mEncoderInputs);

    refreshTuneables();
    executeState();

    Logger.processInputs("Shooter/Flywheel/Leader", mLeaderFlywheelInputs);
    Logger.processInputs("Shooter/Flywheel/Follower", mFollowerFlywheelInputs);
    Logger.processInputs("Shooter/Flywheel/Encoder", mEncoderInputs);
  }

  public void setCANRangeUsage(boolean pShouldUseCANRange) {
    mShouldUseCanRanges = pShouldUseCANRange;
  }

  private void executeState() {
    if (FlywheelConstants.kFlywheelSetpointToVelocity.containsKey(mCurrentFlywheelState)) {
      setFlywheelVelocity(FlywheelConstants.kFlywheelSetpointToVelocity.get(mCurrentFlywheelState).get());
    } else if (FlywheelConstants.kFlywheelSetpointToVoltageTuneable.containsKey(mCurrentFlywheelState)) {
      setFlywheelVoltage(FlywheelConstants.kFlywheelSetpointToVoltageTuneable.get(mCurrentFlywheelState).get());
    } else {
      switch (mCurrentFlywheelState) {
        case STOPPED -> {
          stopFlywheels();
        }
        case SHOTMAP_VELOCITY -> {
          setFlywheelVelocity(Rotation2d.fromRotations(
            ShotMap.getInstance().getFlywheelVel().getRotations() * (mShouldUseCanRanges && mCanRangeSS.allHasFuel() ? tInitialBoostFactor.get() : 1.0))
          );
        }
        case BOOST_SHOTMAP_VELOCITY -> {
          setFlywheelVelocity(Rotation2d.fromRotations(ShotMap.getInstance().getFlywheelVel().getRotations() * tBoostFactor.get()));
        }
        case TUNING_AMPERAGE -> {
          setFlywheelAmperage(tTuningAmperage.get());
        }
        default -> {
          Telemetry.reportIssue(new UnaccountedEnum(mCurrentFlywheelState.toString()));
        }
      }
    }
  }

  @AutoLogOutput(key="Flywheels/IsHailStormReady")
  public boolean isHailstormReady(){
    return getFlywheelRPS().getRotations() >= FlywheelConstants.tLowestHailstormRPS.getAsDouble();
  }

  private void stopFlywheels() {
    mLeaderFlywheelIO.stopMotor();
    mFollowerFlywheelIO.enforceFollower();
  }

  private void setFlywheelVoltage(double pVoltage) {
    mLeaderFlywheelIO.setMotorVolts(pVoltage);
    mFollowerFlywheelIO.enforceFollower();
  }

  private void setFlywheelAmperage(double pVoltage) {
    mLeaderFlywheelIO.setMotorAmperage(pVoltage);
    mFollowerFlywheelIO.enforceFollower();
  }

  private void setFlywheelVelocity(Rotation2d pRotsPerS) {
    mLastestClosedLoopGoalRPS = pRotsPerS;
    Logger.recordOutput("Shooter/Flywheel/Control/FunctionSetpoint", mLastestClosedLoopGoalRPS);
    mLeaderFlywheelIO.setMotorVelAndAccel(
        pRotsPerS.getRotations(),
        0.0,
        kFlywheelControlConfig.feedforward().calculate(
          mLeaderFlywheelInputs.iFlywheelClosedLoopReference.getRotations(),
          mLeaderFlywheelInputs.iFlywheelClosedLoopReferenceSlope.getRotations()));
    mFollowerFlywheelIO.enforceFollower();
  }

  public Command setStateCmd(FlywheelStates pNewState) {
    return setStateCmd(pNewState, true);
  }

  public Command setStateCmd(FlywheelStates pNewState, boolean holdRequirementContinuously) {
    return new FunctionalCommand(
        () -> setState(pNewState),
        () -> {
        }, (interrupted) -> {
        },
        () -> !holdRequirementContinuously,
        this);
  }

  private void setState(FlywheelStates pNewState) {
    mCurrentFlywheelState = pNewState;
  }

  public Rotation2d getFlywheelRPS() {
    return mLeaderFlywheelInputs.iFlywheelRotorVelocityRPS;
  }

  private void setBothPDConstants(double pKP, double pKD) {
    mLeaderFlywheelIO.setPDConstants(pKP, pKD);
    mFollowerFlywheelIO.setPDConstants(pKP, pKD);
  }

  private void setFF(double pKS, double pKV, double pKA) {
    kFlywheelControlConfig.feedforward().setKv(pKV);
    kFlywheelControlConfig.feedforward().setKs(pKS);
    kFlywheelControlConfig.feedforward().setKa(pKA);
  }

  public FlywheelStates getFlywheelState() {
    return mCurrentFlywheelState;
  }

  @AutoLogOutput(key = "Shooter/Flywheel/Feedback/ErrorRPS")
  public double getErrorRPS() {
    return mLastestClosedLoopGoalRPS.minus(mLeaderFlywheelInputs.iFlywheelRotorVelocityRPS).getRotations();
  }

  @AutoLogOutput(key = "Shooter/Flywheel/Feedback/LatestClosedLoopGoalRPS")
  public Rotation2d getLatestClosedLoopGoal() {
    return mLastestClosedLoopGoalRPS;
  }

  @AutoLogOutput(key = "Shooter/Flywheel/Feedback/AtLatestClosedLoopGoal")
  public boolean atLatestClosedLoopGoal() {
    return Math.abs(mLeaderFlywheelInputs.iFlywheelRotorVelocityRPS.getRotations()
        - mLastestClosedLoopGoalRPS.getRotations()) <= tFlywheelTolerance.get();
  }

  private void refreshTuneables() {
    LoggedTunableNumber.ifChanged(hashCode(),
        () -> setBothPDConstants(tFlywheelKP.get(), tFlywheelKD.get()),
        tFlywheelKP, tFlywheelKD);

    LoggedTunableNumber.ifChanged(hashCode(),
        () -> mLeaderFlywheelIO.setMotionMagicConstants(tFlywheelMaxVel.get(), tFlywheelMaxAccel.get(),
            tFlywheelMaxJerk.get()),
        tFlywheelMaxVel, tFlywheelMaxAccel, tFlywheelMaxJerk);

    LoggedTunableNumber.ifChanged(hashCode(),
        () -> setFF(tFlywheelKS.get(), tFlywheelKV.get(), tFlywheelKA.get()),
        tFlywheelKS, tFlywheelKV, tFlywheelKA);
  }
}
