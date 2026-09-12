package frc.robot.systems.auton.routines;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import com.pathplanner.lib.events.EventScheduler;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.SequentialEndingCommandGroup;
import frc.robot.systems.climb.ClimbSS;
import frc.robot.systems.drive.Drive;
import frc.robot.systems.drive.controllers.HolonomicController.ConstraintType;
import frc.robot.systems.efi.FuelInjectorSS;
import frc.robot.systems.intake.Intake;
import frc.robot.systems.intake.roller.IntakeRollerSS.IntakeRollerState;
import frc.robot.systems.shooter.combinedShooter.ShooterSS;
import frc.robot.systems.shooter.hood.HoodSS;
public class AutoRoutines {
    private final Drive mDriveSS;    
    private final HoodSS mHoodSS;
    private final ShooterSS mShooterSS;
    private final Intake mIntakeSS;
    private final FuelInjectorSS mFuelInjectorSS;
    private final ClimbSS mClimbSS;
    private final AutoFactory autoFactory;
    private final AutonCommands mAutonCommands;
    private final double kShotTime1Seconds = 3.5;
    private final double kShotTime2Seconds = 6.5;

    @AutoLogOutput(key = "Drive/Auton/DebugValueAutoCommands")
    private String dbg = "";

    public AutoRoutines(Drive pDriveSS, HoodSS pHoodSS, ShooterSS pShooterSS,
            Intake pIntake, FuelInjectorSS pInjectorSS, ClimbSS pClimbSS) {
        this.mDriveSS = pDriveSS;
        this.mHoodSS = pHoodSS;
        this.mShooterSS = pShooterSS;
        this.mIntakeSS = pIntake;
        this.mFuelInjectorSS = pInjectorSS;
        this.mClimbSS = pClimbSS;
        this.mAutonCommands = new AutonCommands(mDriveSS, mIntakeSS, mHoodSS, mShooterSS, mClimbSS, mFuelInjectorSS);

        autoFactory = new AutoFactory(
            mDriveSS::getPose, // A function that returns the current robot pose
            mDriveSS::setPose, // A function that resets the current robot pose to the provided Pose2d
            mDriveSS::followTrajectory, // The drive subsystem trajectory follower 
            true, // If alliance flipping should be enabled 
            mDriveSS // The drive subsystem
        );
    }
    public AutoRoutine TRIValorDoubleSwipeLeft() {
        AutoRoutine routine = autoFactory.newRoutine("TRIValorDoubleSwipeLeft");
        AutoTrajectory firstSwipe = routine.trajectory("TRIValorDoubleSwipeLeft1");
        AutoTrajectory secondSwipe = routine.trajectory("TRIValorDoubleSwipeLeft2");

        Command firstSwipeCmd = firstSwipe.cmd();
        Command secondSwipeCmd = secondSwipe.cmd();

        // When the routine begins, reset odometry and start the first trajectory 
        routine.active().onTrue(
            Commands.sequence(
                firstSwipe.resetOdometry(),
                firstSwipeCmd
            )
        );
        
        Supplier<Pose2d> poseSupplier1 = () -> (firstSwipe.getFinalPose()).orElse(Pose2d.kZero);
        firstSwipe.atTime("intake").onTrue(mAutonCommands.traversePathWithIntakeOutOnly(0.0, "TRIValorDoubleSwipeLeft1"));
        firstSwipe.atTime("stopIntake").onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE));
        firstSwipe.atTime("alignToShoot").onTrue(mAutonCommands.followPathToAutoAlignShoot( 
                    mDriveSS.setToGenericAutoAlignWithGeneratorReset(
                        poseSupplier1,
                        ConstraintType.LINEAR)));
        
        firstSwipe.done().onTrue(mAutonCommands.shootFuelToHub(kShotTime1Seconds));
        firstSwipe.doneFor(kShotTime1Seconds + 1).onTrue(secondSwipeCmd);

        Supplier<Pose2d> poseSupplier2 = () -> (secondSwipe.getFinalPose()).orElse(Pose2d.kZero);
        secondSwipe.atTime("intake").onTrue(mAutonCommands.traversePathWithIntakeOutOnly(0.0, "TRIValorDoubleSwipeLeft2").alongWith(new InstantCommand(() -> dbg = "intake")));
        secondSwipe.atTime("stopIntake").onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE).alongWith(new InstantCommand(() -> dbg = "stopIntake")));
        secondSwipe.atTime("alignToShoot").onTrue(mAutonCommands.followPathToAutoAlignShoot(
             new SequentialEndingCommandGroup(
                    mDriveSS.setToGenericAutoAlignWithGeneratorReset(
                        poseSupplier2,
                        ConstraintType.LINEAR)).alongWith(new InstantCommand(() -> dbg = "alignToShoot"))
            ));
        secondSwipe.done().onTrue(mAutonCommands.shootFuelToHub(kShotTime2Seconds).andThen(new InstantCommand(() -> dbg = "Finished")));
        
        return routine; 
    }


}
