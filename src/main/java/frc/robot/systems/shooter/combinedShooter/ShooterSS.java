package frc.robot.systems.shooter.combinedShooter;

import static frc.robot.systems.shooter.combinedShooter.ShooterConstants.kFlywheelControlConfig;
import static frc.robot.systems.shooter.combinedShooter.ShooterConstants.tTuningAmperage;

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
import frc.robot.systems.shooter.encoder.EncoderIO;
import frc.robot.systems.shooter.encoder.EncoderInputsAutoLogged;
import frc.robot.systems.shooter.shotMap.FeedMap;
import frc.robot.systems.shooter.shotMap.ShotMap;

public class ShooterSS extends SubsystemBase {
  public static enum ShooterStates {
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
    TUNING_AMPERAGE,
    INTAKE_VOLT,
    INTAKE_VELOCITY,
    OUTTAKE_VOLT,
    KICKBACK_VOLT,
    SLIGHT_OUTTAKE_VOLT
  }

  private final ShooterIO mLeaderShooterIO;
  private final ShooterIO mFollower1ShooterIO;
  private final ShooterIO mFollower2ShooterIO;
  private final ShooterIO mFollower3ShooterIO;

  private final EncoderIO mShooterEncoder;

  private final ShooterInputsAutoLogged mLeaderShooterInputs = new ShooterInputsAutoLogged();
  private final ShooterInputsAutoLogged mFollowerShooterInputs = new ShooterInputsAutoLogged();
  private final EncoderInputsAutoLogged mEncoderInputs = new EncoderInputsAutoLogged();

  private final LoggedTunableNumber tShooterKP = new LoggedTunableNumber("Shooter/Shooter/Control/PID/kP",
      kFlywheelControlConfig.pdController().kP());
  private final LoggedTunableNumber tShooterKD = new LoggedTunableNumber("Shooter/Shooter/Control/PID/kD",
      kFlywheelControlConfig.pdController().kD());
  private final LoggedTunableNumber tShooterKS = new LoggedTunableNumber("Shooter/Shooter/Control/FF/kS",
      kFlywheelControlConfig.feedforward().getKs());
  private final LoggedTunableNumber tShooterKV = new LoggedTunableNumber("Shooter/Shooter/Control/FF/kV",
      kFlywheelControlConfig.feedforward().getKv());
  private final LoggedTunableNumber tShooterKA = new LoggedTunableNumber("Shooter/Shooter/Control/FF/kA",
      kFlywheelControlConfig.feedforward().getKa());
  private final LoggedTunableNumber tShooterMaxVel = new LoggedTunableNumber("Shooter/Shooter/Control/Profile/MaxVel",
      kFlywheelControlConfig.motionMagicConstants().maxVelocity());
  private final LoggedTunableNumber tShooterMaxAccel = new LoggedTunableNumber(
      "Shooter/Shooter/Control/Profile/MaxAccel", kFlywheelControlConfig.motionMagicConstants().maxAcceleration());
  private final LoggedTunableNumber tShooterMaxJerk = new LoggedTunableNumber(
      "Shooter/Shooter/Control/Profile/MaxJerk", kFlywheelControlConfig.motionMagicConstants().maxJerk());
  private final LoggedTunableNumber tShooterTolerance = new LoggedTunableNumber("Shooter/Shooter/Control/Tolerance",
      ShooterConstants.kToleranceRPS);
  private final LoggedTunableNumber tBoostFactor = new LoggedTunableNumber("Shooter/Shooter/BoostFactor", 1.055);

  private Rotation2d mLastestClosedLoopGoalRPS = Rotation2d.kZero;

  @AutoLogOutput(key = "Shooter/Shooter/States/CurrentState")
  private ShooterStates mCurrentShooterState = ShooterStates.STOPPED;
  private CANRangeSS mCanRangeSS;
  private boolean mShouldUseCanRanges = false;
  private final LoggedTunableNumber tInitialBoostFactor = new LoggedTunableNumber("Shooter/Shooter/Boost", 1.0);

  public ShooterSS(ShooterIO pLeaderShooterIO, ShooterIO p1FollowerShooterIO, ShooterIO p2FollowerShooterIO, ShooterIO p3FollowerShooterIO, CANRangeSS pCANRanges, EncoderIO pShooterEncoder) {
    this.mLeaderShooterIO = pLeaderShooterIO;
    this.mFollower1ShooterIO = p1FollowerShooterIO;
    this.mFollower2ShooterIO = p2FollowerShooterIO;
    this.mFollower3ShooterIO = p3FollowerShooterIO;
    this.mCanRangeSS = pCANRanges;
    this.mShooterEncoder = pShooterEncoder;
  }

  @Override
  public void periodic() {
    mLeaderShooterIO.updateInputs(mLeaderShooterInputs);
    mFollower1ShooterIO.updateInputs(mFollowerShooterInputs);
    mFollower2ShooterIO.updateInputs(mFollowerShooterInputs);
    mFollower3ShooterIO.updateInputs(mFollowerShooterInputs);

    mShooterEncoder.updateInputs(mEncoderInputs);

    refreshTuneables();
    executeState();

    Logger.processInputs("Shooter/Shooter/Leader", mLeaderShooterInputs);
    Logger.processInputs("Shooter/Shooter/Follower1", mFollowerShooterInputs);
    Logger.processInputs("Shooter/Shooter/Follower2", mFollowerShooterInputs);
    Logger.processInputs("Shooter/Shooter/Follower3", mFollowerShooterInputs);

    Logger.processInputs("Shooter/Shooter/Encoder", mEncoderInputs);
  }

  public void setCANRangeUsage(boolean pShouldUseCANRange) {
    mShouldUseCanRanges = pShouldUseCANRange;
  }

  private void executeState() {
    if (ShooterConstants.kShooterSetpointToVelocity.containsKey(mCurrentShooterState)) {
      setShooterVelocity(ShooterConstants.kShooterSetpointToVelocity.get(mCurrentShooterState).get());
    } 
    else if (ShooterConstants.kShooterSetpointToVoltageTuneable.containsKey(mCurrentShooterState)) {
      setShooterVoltage(ShooterConstants.kShooterSetpointToVoltageTuneable.get(mCurrentShooterState).get());
    } else {
      switch (mCurrentShooterState) {
        case STOPPED -> {
          stopShooter();
        }
        case SHOTMAP_VELOCITY -> {
          setShooterVelocity(Rotation2d.fromRotations(
            ShotMap.getInstance().getFlywheelVel().getRotations() * (mShouldUseCanRanges && mCanRangeSS.allHasFuel() ? tInitialBoostFactor.get() : 1.0))
          );
        }
        case FEED_VELOCITY -> {
          setShooterVelocity(Rotation2d.fromRotations(
            FeedMap.getInstance().getFlywheelVel().getRotations() * (mShouldUseCanRanges && mCanRangeSS.allHasFuel() ? tInitialBoostFactor.get() : 1.0))
          );
        }
        case BOOST_SHOTMAP_VELOCITY -> {
          setShooterVelocity(Rotation2d.fromRotations(ShotMap.getInstance().getFlywheelVel().getRotations() * tBoostFactor.get()));
        }
        case TUNING_AMPERAGE -> {
          setShooterAmperage(tTuningAmperage.get());
        }
        case TUNING_VOLTAGE -> {
          setShooterVoltage(ShooterConstants.tTuningVoltage.get());
        }
        default -> {
          Telemetry.reportIssue(new UnaccountedEnum(mCurrentShooterState.toString()));
        }
      }
    }

  }

  @AutoLogOutput(key="Shooters/IsHailStormReady")
  public boolean isHailstormReady(){
    return getShooterRPS().getRotations() >= ShooterConstants.tLowestHailstormRPS.getAsDouble();
  }

  private void stopShooter() {
    mLeaderShooterIO.stopMotor();
    mFollower1ShooterIO.enforceFollower();
    mFollower2ShooterIO.enforceFollower();
    mFollower3ShooterIO.enforceFollower();

  }

  private void setShooterVoltage(double pVoltage) {
    mLeaderShooterIO.setMotorVolts(pVoltage);
    mFollower1ShooterIO.enforceFollower();
    mFollower2ShooterIO.enforceFollower();
    mFollower3ShooterIO.enforceFollower();  
  }

  private void setShooterAmperage(double pVoltage) {
    mLeaderShooterIO.setMotorAmperage(pVoltage);
    mFollower1ShooterIO.enforceFollower();
    mFollower2ShooterIO.enforceFollower();
    mFollower3ShooterIO.enforceFollower();  
  }

  private void setShooterVelocity(Rotation2d pRotsPerS) {
    mLastestClosedLoopGoalRPS = pRotsPerS;
    Logger.recordOutput("Shooter/Control/FunctionSetpoint", mLastestClosedLoopGoalRPS);
    mLeaderShooterIO.setMotorVelAndAccel(
        pRotsPerS.getRotations(),
        0.0,
        kFlywheelControlConfig.feedforward().calculate(
          mLeaderShooterInputs.iShooterClosedLoopReference.getRotations(),
          mLeaderShooterInputs.iShooterClosedLoopReferenceSlope.getRotations()));
    mFollower1ShooterIO.enforceFollower();
    mFollower2ShooterIO.enforceFollower();
    mFollower3ShooterIO.enforceFollower();  
  }

  public Command setStateCmd(ShooterStates pNewState) {
    return setStateCmd(pNewState, true);
  }

  public Command setStateCmd(ShooterStates pNewState, boolean holdRequirementContinuously) {
    return new FunctionalCommand(
        () -> setState(pNewState),
        () -> {
        }, (interrupted) -> {
        },
        () -> !holdRequirementContinuously,
        this);
  }

  private void setState(ShooterStates pNewState) {
    mCurrentShooterState = pNewState;
  }

  public Rotation2d getShooterRPS() {
    return mLeaderShooterInputs.iShooterRotorVelocityRPS;
  }

  private void setBothPDConstants(double pKP, double pKD) {
    mLeaderShooterIO.setPDConstants(pKP, pKD);
    mFollower1ShooterIO.setPDConstants(pKP, pKD);
    mFollower2ShooterIO.setPDConstants(pKP, pKD);
    mFollower3ShooterIO.setPDConstants(pKP, pKD);

  }

  private void setFF(double pKS, double pKV, double pKA) {
    ShooterConstants.kFlywheelControlConfig.feedforward().setKv(pKV);
    ShooterConstants.kFlywheelControlConfig.feedforward().setKs(pKS);
    ShooterConstants.kFlywheelControlConfig.feedforward().setKa(pKA);
  }

  public ShooterStates getShooterState() {
    return mCurrentShooterState;
  }

  @AutoLogOutput(key = "Shooter/Shooter/Feedback/ErrorRPS")
  public double getErrorRPS() {
    return mLastestClosedLoopGoalRPS.minus(mLeaderShooterInputs.iShooterRotorVelocityRPS).getRotations();
  }

  @AutoLogOutput(key = "Shooter/Shooter/Feedback/LatestClosedLoopGoalRPS")
  public Rotation2d getLatestClosedLoopGoal() {
    return mLastestClosedLoopGoalRPS;
  }

  @AutoLogOutput(key = "Shooter/Shooter/Feedback/AtLatestClosedLoopGoal")
  public boolean atLatestClosedLoopGoal() {
    return Math.abs(mLeaderShooterInputs.iShooterRotorVelocityRPS.getRotations()
        - mLastestClosedLoopGoalRPS.getRotations()) <= tShooterTolerance.get();
  }

  private void refreshTuneables() {
    LoggedTunableNumber.ifChanged(hashCode(),
        () -> setBothPDConstants(tShooterKP.get(), tShooterKD.get()),
        tShooterKP, tShooterKD);

    LoggedTunableNumber.ifChanged(hashCode(),
        () -> mLeaderShooterIO.setMotionMagicConstants(tShooterMaxVel.get(), tShooterMaxAccel.get(),
            tShooterMaxJerk.get()),
        tShooterMaxVel, tShooterMaxAccel, tShooterMaxJerk);

    LoggedTunableNumber.ifChanged(hashCode(),
        () -> setFF(tShooterKS.get(), tShooterKV.get(), tShooterKA.get()),
        tShooterKS, tShooterKV, tShooterKA);
  }

  //for fuel pump
  public boolean atGoal() {
    if(ShooterConstants.kShooterSetpointToVelocity.containsKey(mCurrentShooterState)) {
      return getRPSErrorForState() < ShooterConstants.kToleranceRPS;
    } else{
        return getAvgShooterRPS() >= ShooterConstants.kRPSForShooting.getRotations();
    }
  }

  public double getRPSErrorForState() {
    if(ShooterConstants.kShooterSetpointToVelocity.containsKey(mCurrentShooterState)) {
      return Math.abs(getAvgShooterRPS() - ShooterConstants.kShooterSetpointToVelocity.get(mCurrentShooterState).get().getRotations());
    } else {
      return Double.MAX_VALUE;
    }
  }
  
  public double getAvgShooterRPS() {
    return (
      mLeaderShooterInputs.iShooterRotorVelocityRPS.getRotations() 
        + 
      mFollowerShooterInputs.iShooterRotorVelocityRPS.getRotations()) / 2.0;
  }

}
