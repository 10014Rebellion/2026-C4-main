// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems.climb;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSS extends SubsystemBase {
    public enum ClimbState {
        UP,
        DOWN,
        STAY,
        STAY_ROBOT,
        IDLE,
        STOP,
        INVALID;
    }
  
    private final ClimbIO mClimbIO;
    private final ClimbInputsAutoLogged mClimbInputs = new ClimbInputsAutoLogged();

    private ClimbState mClimbState = ClimbState.IDLE;
    private int mDesiredDirection = 0;
    private boolean mLimitEnforced = false;

    public ClimbSS(ClimbIO pClimbIO) {
        mClimbIO = pClimbIO;
    }

    @Override
    public void periodic() {
        mClimbIO.updateInputs(mClimbInputs);
        Logger.processInputs("Climb", mClimbInputs);
        Logger.recordOutput("Climb/LimitsEnforced", mLimitEnforced);
        Logger.recordOutput("Climb/State", mClimbState);
        executeState();
    }

    public void initiateState(ClimbState pClimbState) {
        mClimbState = pClimbState;
    }

    public void executeState() {
        switch (mClimbState) {
            case STOP -> {
                mClimbIO.stopMotor();
            }
            case UP, DOWN, STAY, STAY_ROBOT -> {
                setClimbVolts(ClimbConstants.kStateToVoltage.get(mClimbState).get());
            }
            case IDLE -> {
                setClimbVolts(ClimbConstants.kStateToVoltage.get(mClimbState).get());
            } case INVALID -> {}
            default  -> {}
        }
    }

    public Command setStateCmd(ClimbState pState) {
        return new FunctionalCommand(
            () -> initiateState(pState), 
            () -> {}, 
            (interrupted) -> {}, 
            () -> false, 
            this);
    }


    public Command goUpTillClimbHeightThenStay() {
        return setStateCmd(ClimbState.UP)
            .until(() -> mClimbInputs.iClimbPositionMeters > ClimbConstants.kClimbHeight)
            .andThen(setStateCmd(ClimbState.STAY).withTimeout(0.02));
    }

    public boolean readyToPreClimb() {
        return mClimbInputs.iClimbPositionMeters > ClimbConstants.kClimbHeight;
    }

    public Command goDownTillClimbedThenStayClimbed() {
        return setStateCmd(ClimbState.DOWN)
            .until(() -> mClimbInputs.iClimbPositionMeters < ClimbConstants.kClimbedHeight)
            .andThen(setStateCmd(ClimbState.STAY_ROBOT).withTimeout(0.02));
    }

    public boolean hasClimbed() {
        return mClimbInputs.iClimbPositionMeters < ClimbConstants.kClimbedHeight;
    }

    public void setClimbVolts(double pVolts) {
        mDesiredDirection = toDirection(pVolts);
        mClimbIO.setMotorVolts(pVolts);
        enforceSoftLimits();
    }

    public void changeClimbNeutralMode(NeutralModeValue pNeutralMode){
        mClimbIO.changeClimbNeutralMode(pNeutralMode);
    }

    public boolean atGoal(double pGoal) {
        return Math.abs(pGoal - mClimbInputs.iClimbPositionMeters) < 0.01;
    }

    public void enforceSoftLimits() {
        if(
        (mClimbInputs.iClimbPositionMeters > ClimbConstants.kSoftLimits.forwardLimitM()
            && mDesiredDirection == 1 ) 
            || 
        (mClimbInputs.iClimbPositionMeters < ClimbConstants.kSoftLimits.backwardLimitM() 
            && mDesiredDirection == -1)) {
                mLimitEnforced = true;
                mClimbIO.stopMotor();
        } else {
            mLimitEnforced = false;
        }
    }

    public int toDirection(double val) {
        if(val > 0) return 1;
        if(val < 0) return -1;
        else return 0;
    }
}
