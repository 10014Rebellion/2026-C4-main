// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems.intake.roller;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.systems.intake.IntakeConstants;
import frc.robot.systems.shooter.combinedShooter.ShooterIO;
import frc.robot.systems.shooter.flywheels.FlywheelInputsAutoLogged;

public class IntakeRollerSS extends SubsystemBase {
    public static enum IntakeRollerState {
        IDLE,
        INTAKE,
        OUTTAKE,
        TUNING,
        INVALID;
    }

    private final IntakeRollerIO mIntakeLeaderRollerIO;
    private final IntakeRollerIO mIntakeFollowerRollerIO;
    private final IntakeRollerInputsAutoLogged mLeaderIntakeRollerInputs = new IntakeRollerInputsAutoLogged();
    private final IntakeRollerInputsAutoLogged mFollowerIntakeRollerInputs = new IntakeRollerInputsAutoLogged();     

    @AutoLogOutput(key="IntakeRoller/State")
    private IntakeRollerState mIntakeRollerState = IntakeRollerState.IDLE;

    public IntakeRollerSS(IntakeRollerIO pLeaderIntakeRollerIO, IntakeRollerIO pFollowerIntakeRollerIO) {
        this.mIntakeLeaderRollerIO = pLeaderIntakeRollerIO;
        this.mIntakeFollowerRollerIO = pFollowerIntakeRollerIO;
    }
  
    @Override
    public void periodic() {
        mIntakeLeaderRollerIO.updateInputs(mLeaderIntakeRollerInputs);
        mIntakeFollowerRollerIO.updateInputs(mFollowerIntakeRollerInputs);
        Logger.processInputs("Intake/IntakeRoller/Leader", mLeaderIntakeRollerInputs);
        Logger.processInputs("Intake/IntakeRoller/Follower", mFollowerIntakeRollerInputs);
        executeState();
    }

    public void executeState() {
        switch (mIntakeRollerState) {
            case IDLE, INTAKE, OUTTAKE, TUNING -> {
                setRollerVolts(
                    IntakeConstants.RollerConstants.kStateToIntakeVoltage.get(mIntakeRollerState).get());
            } 
            case INVALID -> {}
            default -> {}
        }
    }

    private void setRollerVolts(double pVolts) {
        mIntakeLeaderRollerIO.setMotorVolts(pVolts);
        mIntakeFollowerRollerIO.enforceFollower();
    }

    public Command setStateCmd(IntakeRollerState pIntakeRollerState) {
        return setStateCmd(pIntakeRollerState, true);
    }
  
    public Command setStateCmd(IntakeRollerState pIntakeRollerState, boolean holdReqs) {
        return new FunctionalCommand(
            () -> mIntakeRollerState = pIntakeRollerState, 
            () -> {}, 
            (interrupted) -> {}, 
            () -> !holdReqs, 
            this);
    }
}