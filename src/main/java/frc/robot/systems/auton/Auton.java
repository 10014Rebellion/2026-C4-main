package frc.robot.systems.auton;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import frc.robot.commands.FollowPathCommand;

import frc.robot.commands.AutoEvent;
import frc.robot.systems.climb.ClimbSS;
import frc.robot.systems.drive.Drive;
import frc.robot.systems.efi.FuelInjectorSS;
import frc.robot.systems.intake.Intake;
import frc.robot.systems.shooter.flywheels.FlywheelsSS;
import frc.robot.systems.shooter.fuelpump.FuelPumpSS;
import frc.robot.systems.shooter.hood.HoodSS;

public class Auton {
    protected final AutonCommands mAutos;
    protected final Drive mDriveSS;
    protected final FlywheelsSS mFlywheelsSS;
    protected final HoodSS mHoodSS;
    protected final FuelPumpSS mFuelPumpSS;
    protected final Intake mIntakeSS;
    protected final ClimbSS mClimbSS;
    protected final FuelInjectorSS mInjectorSS;

    public Auton(AutonCommands pAutos) {
        mAutos = pAutos;

        this.mDriveSS = pAutos.getDriveSubsystem();
        this.mFlywheelsSS = pAutos.getFlywheelSubsystem();
        this.mHoodSS = pAutos.getHoodSubsystem();
        this.mFuelPumpSS = pAutos.getFuelPumpSubsystem();
        this.mIntakeSS = pAutos.getIntakeSubsystem();
        this.mClimbSS = pAutos.getClimbSubsystem();
        this.mInjectorSS = pAutos.getFuelInjector();
    }

    protected FollowPathCommand followChoreoPath(
            String pPathName, boolean pIsFirst, AutoEvent pAutoEvent, boolean isMirrored) {
        return mAutos.followChoreoPath(pPathName, pIsFirst, pAutoEvent, isMirrored);
    }

    protected FollowPathCommand followChoreoPath(String pPathName, PPHolonomicDriveController pPID, boolean pIsFirst, AutoEvent pAutoEvent, boolean isMirrored) {
        return mAutos.followChoreoPath(pPathName, pPID, pIsFirst, pAutoEvent, isMirrored);
    }

    protected AutoEvent getAuton() {
        return new AutoEvent("EmptyAuto", mAutos);
    }
}
