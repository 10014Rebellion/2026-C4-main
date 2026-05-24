// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.telemetry.Telemetry;

public class AutoEvent extends Command {
    private final EventLoop mAutoEventLoop;
    private boolean mIsRunning = false;
    private final Trigger mIsRunningTrigger;
    private String mAutoName;
  
    public AutoEvent(String pAutoName, Subsystem pSubsystem) {
        mAutoName = pAutoName;

        mAutoEventLoop = new EventLoop();

        mIsRunningTrigger = loggedCondition("IsRunning", () -> mIsRunning, false);

        addRequirements(pSubsystem);
    }

    @Override
    public void initialize() {
        mIsRunning = true;
        mAutoEventLoop.poll();
    }

    @Override
    public void execute() {
        mAutoEventLoop.poll();   
    }

    @Override
    public void end(boolean interrupted) {
        mIsRunning = false;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public boolean getIsRunning() {
        return mIsRunning;
    }

    public Trigger getIsRunningTrigger() {
        return mIsRunningTrigger;
    }

    public Trigger condition(BooleanSupplier condition) {
        return new Trigger(mAutoEventLoop, condition);
    }

    public EventLoop getLoop() {
        return mAutoEventLoop;
    }

    public Trigger loggedCondition(String key, BooleanSupplier condition, boolean useTuneableOverride) {
        if(useTuneableOverride) {
            LoggedNetworkBoolean override = new LoggedNetworkBoolean(mAutoName+"/"+key, false);
            return new Trigger(mAutoEventLoop, () -> {
                boolean cond = condition.getAsBoolean();
                Telemetry.log("Auton/"+mAutoName+"/"+key, cond);
                return override.get() || cond;
            });
        } else {
            return new Trigger(mAutoEventLoop, () -> {
                boolean cond = condition.getAsBoolean();
                Telemetry.log("Auton/"+mAutoName+"/"+key, cond);
                return cond;
            });
        }
    }

    public Trigger loggedCondition(String key, BooleanSupplier condition, boolean useTuneableOverride, EventLoop loop) {
        if(useTuneableOverride) {
            LoggedNetworkBoolean override = new LoggedNetworkBoolean(mAutoName+"/"+key, false);
            return new Trigger(loop, () -> {
                boolean cond = condition.getAsBoolean();
                Telemetry.log("Auton/"+mAutoName+"/"+key, cond);
                return override.get() || cond;
            });
        } else {
            return new Trigger(loop, () -> {
                boolean cond = condition.getAsBoolean();
                Telemetry.log("Auton/"+mAutoName+"/"+key, cond);
                return cond;
            });
        }
    }
}
