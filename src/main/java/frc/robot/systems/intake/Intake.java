// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.systems.intake.rack.IntakeRackSS;
import frc.robot.systems.intake.rack.IntakeRackSS.IntakeRackState;
import frc.robot.systems.intake.roller.IntakeRollerSS;
import frc.robot.systems.intake.roller.IntakeRollerSS.IntakeRollerState;

public class Intake {
  public static final double tIntakeCompactTime = 0.3;

  private final IntakeRackSS mIntakeRackSS;
  private final IntakeRollerSS mIntakeRollerSS;

  public Intake(IntakeRackSS mIntakeRackSS, IntakeRollerSS pIntakeRollerSS) {
    this.mIntakeRackSS = mIntakeRackSS;
    this.mIntakeRollerSS = pIntakeRollerSS;
  }

  // ROLLER COMMANDS //
  public Command setRollerStateCmd(IntakeRollerState rollerState) {
    return mIntakeRollerSS.setStateCmd(rollerState);
  }

  public Command setRollerStateCmd(IntakeRollerState rollerState, boolean holdReqs) {
    return mIntakeRollerSS.setStateCmd(rollerState, holdReqs);
  }

  public Command stopRollerCmd() {
    return mIntakeRollerSS.setStateCmd(IntakeRollerState.IDLE);
  }

  public void disableRackSoftLimits() {
    mIntakeRackSS.shouldDisableSoftLimits(true);
  }

  public void enableRackSoftLimits() {
    mIntakeRackSS.shouldDisableSoftLimits(false);
  }

  // PIVOT COMMANDS //
  public Command setRackStateCmd(IntakeRackState intake) {
    return mIntakeRackSS.setStateCmd(intake);
  }

  public Command setRackStateCmd(IntakeRackState intake, boolean holdReqs) {
    return mIntakeRackSS.setStateCmd(intake, holdReqs);
  }

  public Command trashCompactRepeat() {
    return new RepeatCommand(new SequentialCommandGroup(
        mIntakeRackSS.setStateCmd(IntakeRackState.COMPACT_HIGH).withTimeout(tIntakeCompactTime),
        mIntakeRackSS.setStateCmd(IntakeRackState.COMPACT_LOW).withTimeout(tIntakeCompactTime)));
  }

  public Command trashCompact() {
    return mIntakeRackSS.setStateCmd(IntakeRackState.COMPACT);
  }

  public Command anshulCompact() {
    return mIntakeRackSS.setStateCmd(IntakeRackState.ANSHUL_COMPACT);
  }

  public boolean safeToRunRollers() {
    return mIntakeRackSS.isSafeToRunintakeRollers();
  }
}
