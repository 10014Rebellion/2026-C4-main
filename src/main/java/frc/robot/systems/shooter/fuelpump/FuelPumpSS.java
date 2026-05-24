package frc.robot.systems.shooter.fuelpump;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.telemetry.Telemetry;
import frc.lib.tuning.LoggedTunableNumber;
import frc.robot.logging.InvalidValueErrors.UnaccountedEnum;

import static frc.robot.systems.shooter.fuelpump.FuelPumpConstants.kFuelPumpControlConfig;

public class FuelPumpSS extends SubsystemBase {
  public static enum FuelPumpState {
    STOPPED,
    TUNING_VOLT,
    INTAKE_VOLT,
    INTAKE_VELOCITY,
    OUTTAKE_VOLT,
    KICKBACK_VOLT,
    SLIGHT_OUTTAKE_VOLT
  }

  @AutoLogOutput(key = "Shooter/FuelPump/States/CurrentState")
  private FuelPumpState mCurrentFuelPumpState = FuelPumpState.STOPPED;

  private final FuelPumpIO mLeaderFuelPumpIO;
  private final FuelPumpIO mFollowerFuelPumpIO;

  private final FuelPumpInputsAutoLogged mLeaderFuelPumpInputs = new FuelPumpInputsAutoLogged();
  private final FuelPumpInputsAutoLogged mFollowerFuelPumpInputs = new FuelPumpInputsAutoLogged();

  private final LoggedTunableNumber tFuelPumpKP = new LoggedTunableNumber("Shooter/FuelPump/Control/PID/kP", kFuelPumpControlConfig.pdController().kP());
  private final LoggedTunableNumber tFuelPumpKD = new LoggedTunableNumber("Shooter/FuelPump/Control/PID/kD", kFuelPumpControlConfig.pdController().kD());
  private final LoggedTunableNumber tFuelPumpKS = new LoggedTunableNumber("Shooter/FuelPump/Control/FF/kS", kFuelPumpControlConfig.feedforward().getKs());
  private final LoggedTunableNumber tFuelPumpKV = new LoggedTunableNumber("Shooter/FuelPump/Control/FF/kV", kFuelPumpControlConfig.feedforward().getKv());
  private final LoggedTunableNumber tFuelPumpKA = new LoggedTunableNumber("Shooter/FuelPump/Control/FF/kA", kFuelPumpControlConfig.feedforward().getKa());
  // private final LoggedTunableNumber tFuelPumpTolerance = new LoggedTunableNumber("Shooter/FuelPump/Control/Tolerance", FuelPumpConstants.kToleranceRPS);

  private SimpleMotorFeedforward mFuelPumpFeedforward = new SimpleMotorFeedforward(tFuelPumpKS.get(), tFuelPumpKV.get(), tFuelPumpKA.get());

  public FuelPumpSS(FuelPumpIO pLeaderFuelPumpIO, FuelPumpIO pFollowerFuelPumpIO) {
    this.mLeaderFuelPumpIO = pLeaderFuelPumpIO;
    this.mFollowerFuelPumpIO = pFollowerFuelPumpIO;
  }

  @Override
  public void periodic() {
    mLeaderFuelPumpIO.updateInputs(mLeaderFuelPumpInputs);
    mFollowerFuelPumpIO.updateInputs(mFollowerFuelPumpInputs);

    refreshTuneables();
    executeState();
    mFollowerFuelPumpIO.enforceFollower();

    Telemetry.log("Shooter/FuelPump/IsFuelPumpAtGoal", atGoal());
    Logger.processInputs("Shooter/FuelPump/Leader", mLeaderFuelPumpInputs);
    Logger.processInputs("Shooter/FuelPump/Follower", mFollowerFuelPumpInputs);
  }

  public boolean atGoal() {
    if(FuelPumpConstants.kStateToTuneableFuelPumpVelocity.containsKey(mCurrentFuelPumpState)) {
      return getRPSErrorForState() < FuelPumpConstants.kToleranceRPS;
    } else{
        return getAvgFuelPumpRPS() >= FuelPumpConstants.kRPSForShooting.getRotations();
    }
  }

  public double getRPSErrorForState() {
    if(FuelPumpConstants.kStateToTuneableFuelPumpVelocity.containsKey(mCurrentFuelPumpState)) {
      return Math.abs(getAvgFuelPumpRPS() - FuelPumpConstants.kStateToTuneableFuelPumpVelocity.get(mCurrentFuelPumpState).get().getRotations());
    } else {
      return Double.MAX_VALUE;
    }
  }

  private void executeState() {
    if(FuelPumpConstants.kStateToTuneableFuelPumpVelocity.containsKey(mCurrentFuelPumpState)) {
      setFuelPumpVelocity(FuelPumpConstants.kStateToTuneableFuelPumpVelocity.get(mCurrentFuelPumpState).get());
    } else if(FuelPumpConstants.kStateToTuneableFuelPumpVolts.containsKey(mCurrentFuelPumpState)) {
      setFuelPumpVolts(FuelPumpConstants.kStateToTuneableFuelPumpVolts.get(mCurrentFuelPumpState).get());
    } else {
      switch (mCurrentFuelPumpState) {
        case STOPPED -> {
          stopMotors();
        }
        case TUNING_VOLT -> {
          setFuelPumpVolts(FuelPumpConstants.tTuningVoltage.get());
        }
        default -> {
          Telemetry.reportIssue(new UnaccountedEnum(mCurrentFuelPumpState.toString()));
        }
      }
    }
  }

  private void setFuelPumpVelocity(Rotation2d pRPS) {
    mLeaderFuelPumpIO.setMotorVelocity(
      pRPS,
      mFuelPumpFeedforward.calculate(pRPS.getRotations())
  );
    mFollowerFuelPumpIO.enforceFollower();
  }

  private void setFuelPumpVolts(double pVoltage) {
    mLeaderFuelPumpIO.setMotorVolts(pVoltage);
    mFollowerFuelPumpIO.enforceFollower();
  }

  private void stopMotors() {
    mLeaderFuelPumpIO.stopMotor();
    mFollowerFuelPumpIO.enforceFollower();
  }

  public Command setStateCmd(FuelPumpState pNewState) {
    return setStateCmd(pNewState, true);
  }

  public Command setStateCmd(FuelPumpState pNewState, boolean holdRequirementContinuously) {
    return new FunctionalCommand(
      () -> setState(pNewState), () -> {}, (interrupted) -> {},
      () -> !holdRequirementContinuously, this);
  }

  private void setState(FuelPumpState pNewState) {
    mCurrentFuelPumpState = pNewState;
  }

  public double getAvgFuelPumpRPS() {
    return (
      mLeaderFuelPumpInputs.iFuelPumpVelocityRPS.getRotations() 
        + 
      mFollowerFuelPumpInputs.iFuelPumpVelocityRPS.getRotations()) / 2.0;
  }

  public void setBothPDConstants(double KP, double KD){
    mFollowerFuelPumpIO.setPDConstants(0, KP, KD);
    mLeaderFuelPumpIO.setPDConstants(0, KP, KD);
  }

  public void setFF(double pKS, double pKV, double pKA){
    mFuelPumpFeedforward.setKs(pKS);
    mFuelPumpFeedforward.setKv(pKV);
    mFuelPumpFeedforward.setKa(pKA);
  }

  private void refreshTuneables() {
    LoggedTunableNumber.ifChanged( hashCode(), 
      () -> setBothPDConstants(tFuelPumpKP.get(), tFuelPumpKD.get()), 
      tFuelPumpKP, tFuelPumpKD
    );

    LoggedTunableNumber.ifChanged( hashCode(), 
      () -> setFF(tFuelPumpKS.get(), tFuelPumpKV.get(), tFuelPumpKA.get()),
      tFuelPumpKS, tFuelPumpKV, tFuelPumpKA
    );
  }
}
