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

public class IntakeRollerSS extends SubsystemBase {
    public static enum IntakeRollerState {
        IDLE,
        INTAKE,
        OUTTAKE,
        TUNING,
        INVALID;
    }

    private final IntakeRollerIO mIntakeRollerIO;
    private final IntakeRollerInputsAutoLogged mIntakeRollerInputs = new IntakeRollerInputsAutoLogged();

    @AutoLogOutput(key="IntakeRoller/State")
    private IntakeRollerState mIntakeRollerState = IntakeRollerState.IDLE;

    public IntakeRollerSS(IntakeRollerIO pIntakeRollerIO) {
        this.mIntakeRollerIO = pIntakeRollerIO;
    }
  
    @Override
    public void periodic() {
        mIntakeRollerIO.updateInputs(mIntakeRollerInputs);
        Logger.processInputs("Intake/Roller", mIntakeRollerInputs);

        executeState();
    }

    public void executeState() {
        switch (mIntakeRollerState) {
            case IDLE, INTAKE, OUTTAKE, TUNING -> {
                mIntakeRollerIO.setMotorVolts(
                    IntakeConstants.RollerConstants.kStateToIntakeVoltage.get(mIntakeRollerState).get());
            } 
            case INVALID -> {}
            default -> {}
        }
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