package frc.robot.systems.auton;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.math.AllianceFlipUtil;
import frc.robot.commands.AutoEvent;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.SequentialEndingCommandGroup;
import frc.robot.game.GameGoalPoseChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.FollowPathCommand;
import frc.robot.systems.intake.rack.IntakeRackSS.IntakeRackState;
import frc.robot.systems.intake.roller.IntakeRollerSS.IntakeRollerState;
import frc.robot.systems.drive.controllers.HolonomicController.ConstraintType;
import frc.robot.systems.shooter.flywheels.FlywheelsSS.FlywheelStates;
import frc.robot.systems.shooter.hood.HoodSS.HoodStates;
import frc.robot.systems.shooter.fuelpump.FuelPumpSS.FuelPumpState;

public class SingleSwipeClimb extends Auton {
    private boolean mWantToShoot = false;
    private final String mAutoName;
    private final String mFirstSwipePathName;
    private final double mFirstSwipeAlignTime;

    private final Supplier<Pose2d> mTrenchApproachPose;
    private final Supplier<Pose2d> mTrenchExitPose;

    private final double kShotTimeSeconds = 6.5;
    private final double kShotEndTimeSeconds = 0.02; 

    private final Pose2d mClimbPose;

    private final boolean mIsMirrored;

    public SingleSwipeClimb(
        AutonCommands pAutos, 
        String pAutoName, 
        String pFirstSwipePathName, 
        double pFirstSwipeAlignTime,
        Supplier<Pose2d> pTrenchApproachPose, Supplier<Pose2d> pTrenchExitPose,
        Pose2d pClimbPose,
        boolean pIsMirrored) {
        super(pAutos);
        mAutoName = pAutoName;
        mFirstSwipePathName = pFirstSwipePathName;
        mFirstSwipeAlignTime = pFirstSwipeAlignTime;
        mTrenchApproachPose = pTrenchApproachPose;
        mTrenchExitPose = pTrenchExitPose;
        mClimbPose = pClimbPose;
        mIsMirrored = pIsMirrored;
    }

    @Override
    protected AutoEvent getAuton() {
        AutoEvent auto = new AutoEvent(mAutoName, mAutos);

        Trigger autoActivted = auto.getIsRunningTrigger();

        Trigger intakingRange = mAutos.inIntakeRange(auto);
        Trigger shootingRange = auto.loggedCondition(
            auto.getName()+"/WantToShoot", 
            () -> mWantToShoot, 
            true);

        Trigger inShootingTolerance = auto.loggedCondition(
            auto.getName()+"/ShootingTolerance", 
            () -> 
                mFlywheelsSS.atLatestClosedLoopGoal() && 
                mHoodSS.atGoal() &&
                mDriveSS.getDriveManager().waitUntilAutoAlignFinishes().getAsBoolean(), 
            true);

        Trigger inShootingToleranceDebounced = inShootingTolerance
            .debounce(0.1, DebounceType.kRising)
            .debounce(5.0, DebounceType.kFalling);

        FollowPathCommand firstSwipePath = 
            followChoreoPath(mFirstSwipePathName, true, auto, mIsMirrored);

        Pose2d lastPoseOfFirstSwipe = mAutos.getTraj(mFirstSwipePathName).get().getPathPoses().get(
            mAutos.getTraj(mFirstSwipePathName).get().getPathPoses().size() - 1);

        SequentialEndingCommandGroup firstSwipeIntakeShot = 
            mAutos.timedIntakeShot(kShotTimeSeconds, kShotEndTimeSeconds);

        SequentialEndingCommandGroup firstSwipeInjectorShot = 
            mAutos.timedInjectorShot(kShotTimeSeconds, kShotEndTimeSeconds);

        Trigger hasFirstShotEnded = auto.loggedCondition(
            mFirstSwipePathName+"/FirstShotEnded", 
            () -> (firstSwipeIntakeShot.hasEnded() && firstSwipeInjectorShot.hasEnded()),
            true);

        SequentialEndingCommandGroup goToPreClimbPose = new SequentialEndingCommandGroup(
            mDriveSS.getDriveManager().setToGenericAutoAlign(
                () -> getClimbEndPose(mClimbPose.transformBy(new Transform2d(0.0, -0.05, Rotation2d.kZero))), 
                ConstraintType.LINEAR));

        Trigger atPreClimbPose = auto.loggedCondition(
            "ClimbEnd/AtPreClimbPose", 
            () -> 
                goToPreClimbPose.isRunning()
                    &&
                mDriveSS.getDriveManager().waitUntilAutoAlignFinishes().getAsBoolean(), 
            true);

        SequentialEndingCommandGroup goToClimbPose = new SequentialEndingCommandGroup(
            mDriveSS.getDriveManager().setToGenericAutoAlign(
                () -> getClimbEndPose(mClimbPose), 
                ConstraintType.LINEAR));

        Trigger atClimbPose = auto.loggedCondition(
            "ClimbEnd/ReadyToPreClimb", 
            () -> 
                goToPreClimbPose.isRunning()
                    &&
                mDriveSS.getDriveManager().waitUntilAutoAlignFinishes().getAsBoolean(), 
            true);

        SequentialEndingCommandGroup prepareForClimb = 
            new SequentialEndingCommandGroup(mClimbSS.goUpTillClimbHeightThenStay());

        Trigger preparedForClimb = auto.loggedCondition(
            "ClimbEnd/PreparedForClimb", 
            () -> prepareForClimb.hasEnded(), 
            true);

        SequentialEndingCommandGroup climb = 
            new SequentialEndingCommandGroup(mClimbSS.goUpTillClimbHeightThenStay());

        Trigger hasClimbed = auto.loggedCondition(
            "ClimbEnd/HasClimbed", 
            () -> climb.hasEnded(), 
            true);

        intakingRange
            .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.INTAKE))
            .onTrue(mIntakeSS.setRackStateCmd(IntakeRackState.INTAKE))
            .onFalse(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE));

        shootingRange
            .onTrue(mFlywheelsSS.setStateCmd(FlywheelStates.SHOTMAP_VELOCITY))
            .onTrue(mHoodSS.setStateCmd(HoodStates.SHOTMAP_POSITION))
            .onTrue(mFuelPumpSS.setStateCmd(FuelPumpState.INTAKE_VOLT))
            .onFalse(mFlywheelsSS.setStateCmd(FlywheelStates.STANDBY_VELOCITY))
            .onFalse(mHoodSS.setStateCmd(HoodStates.MIN))
            .onFalse(mFuelPumpSS.setStateCmd(FuelPumpState.STOPPED));

        /* FIRST PATHHH */
        autoActivted
            .onTrue(Commands.waitSeconds(0.5).andThen(firstSwipePath))
            .onTrue(mFlywheelsSS.setStateCmd(FlywheelStates.STANDBY_VELOCITY))
            .onTrue(mIntakeSS.setRackStateCmd(IntakeRackState.INTAKE))
            .onTrue(Commands.runOnce(() -> mWantToShoot = false));

        firstSwipePath.atTime(mFirstSwipeAlignTime)
            .onTrue(Commands.runOnce(() -> mWantToShoot = true))
            .onTrue((new SequentialEndingCommandGroup(
                    transitionPose(mTrenchApproachPose),
                    transitionPose(mTrenchExitPose),
                    mDriveSS.getDriveManager().setToGenericAutoAlignWithGeneratorReset(
                        () -> getSwipeEndPose(lastPoseOfFirstSwipe),
                        ConstraintType.LINEAR))));

        firstSwipePath.hasEnded().and(() -> mWantToShoot).and(hasFirstShotEnded.negate()).and(inShootingToleranceDebounced)
            .onTrue(firstSwipeIntakeShot)
            .onTrue(firstSwipeInjectorShot)
            .onTrue(mIntakeSS.trashCompact());

        firstSwipePath.hasEnded().and(hasFirstShotEnded)
            .onTrue(Commands.runOnce(() -> mWantToShoot = false))
            .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE))
            .onTrue(mIntakeSS.setRackStateCmd(IntakeRackState.INTAKE))
            .onTrue(mFuelPumpSS.setStateCmd(FuelPumpState.STOPPED))
            .onTrue(goToPreClimbPose)
            .onTrue(prepareForClimb);

        /* GO CLIMBBB */
        atPreClimbPose.and(preparedForClimb)
            .onTrue(goToClimbPose);

        atClimbPose.and(preparedForClimb)
            .onTrue(climb);

        hasClimbed
            .onTrue(mAutos.endAuto(auto));

        return auto;
    }

    private Pose2d getSwipeEndPose(Pose2d pose) {
        return AllianceFlipUtil.apply(
            new Pose2d(
                pose.getX(), pose.getY(),
                GameGoalPoseChooser.turnFromHub(AllianceFlipUtil.apply(pose))
                    .plus((AllianceFlipUtil.shouldFlip()) 
                        ? Rotation2d.k180deg 
                        : Rotation2d.kZero)
        ));
    }

    private Pose2d getClimbEndPose(Pose2d pose) {
        return AllianceFlipUtil.apply(pose);
    }

    public Command transitionPose(Supplier<Pose2d> poseSup) {
        return mDriveSS.getDriveManager().setToGenericAutoAlignWithGeneratorReset(
            poseSup,
            () -> new ChassisSpeeds(AllianceFlipUtil.shouldFlip() ? -0.5 : 0.5, 0.0, 0.0),
            ConstraintType.LINEAR)
                .onlyIf(
                    () -> (
                        !AllianceFlipUtil.shouldFlip()
                            &&
                        poseSup.get().getX() < mDriveSS.getPoseEstimate().getX()
                    ) || (
                        AllianceFlipUtil.shouldFlip()
                            &&
                        poseSup.get().getX() > mDriveSS.getPoseEstimate().getX()
                    )
                )
                .onlyWhile(() -> (mDriveSS.getDriveManager().getAutoAlignController().inTolerance(new Transform2d(0.1, 0.1, Rotation2d.fromDegrees(5.0)), mDriveSS.getPoseEstimate())));
    }
}
