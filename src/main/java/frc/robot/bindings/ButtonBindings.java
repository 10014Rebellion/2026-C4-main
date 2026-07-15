package frc.robot.bindings;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.controllers.FlydigiApex4;
import frc.lib.controllers.RebelButtonBoardRebuilt;
import frc.lib.math.AllianceFlipUtil;
import frc.robot.commands.DriveCharacterizationCommands;
import frc.robot.game.GameGoalPoseChooser;
import frc.robot.systems.climb.ClimbSS;
import frc.robot.systems.climb.ClimbSS.ClimbState;
import frc.robot.systems.drive.Drive;
import frc.robot.systems.drive.DriveManager.DriveState;
import frc.robot.systems.efi.FuelInjectorSS;
import frc.robot.systems.efi.FuelInjectorSS.FuelInjectorState;
import frc.robot.systems.efi.sensors.CANRangeSS;
import frc.robot.systems.intake.Intake;
import frc.robot.systems.intake.rack.IntakeRackSS.IntakeRackState;
import frc.robot.systems.intake.roller.IntakeRollerSS.IntakeRollerState;
import frc.robot.systems.shooter.combinedShooter.ShooterSS;
import frc.robot.systems.shooter.combinedShooter.ShooterSS.ShooterStates;
import frc.robot.systems.shooter.hood.HoodSS;
import frc.robot.systems.shooter.hood.HoodSS.HoodStates;;

public class ButtonBindings {
    public static enum HeadingTraversalState {
        ALLIANCE, CENTER, NONE
    }

    private final boolean closedLoopFuelPump = true;
    private final Drive mDriveSS;
    private final HoodSS mHoodSS;
    private final ShooterSS mShooterSS;
    private final Intake mIntakeSS;
    private final FuelInjectorSS mFuelInjectorSS;
    private final ClimbSS mClimbSS;
    private final CANRangeSS mCANRanges;

    private final FlydigiApex4 mPilotController = new FlydigiApex4(BindingsConstants.kPilotControllerPort);
    private final FlydigiApex4 mGunnerController = new FlydigiApex4(1);
    private final RebelButtonBoardRebuilt mGunnerButtonboard = new RebelButtonBoardRebuilt(1, 2);

    private final LoggedNetworkBoolean kUsingPilotGunner = new LoggedNetworkBoolean("DriverOperator/UsePilotGunner",
            true);

    HoodStates prevHoodState = HoodStates.STOPPED;
    DriveState prevDriveState = DriveState.TELEOP;
    HeadingTraversalState mHeadingTraversalState = HeadingTraversalState.NONE;

    private boolean inCenterFlag = false;

    public ButtonBindings(Drive pDriveSS, HoodSS pHoodSS, ShooterSS pShooterSS,
            Intake pIntake, FuelInjectorSS pInjectorSS, ClimbSS pClimbSS, CANRangeSS pCANRanges) {
        this.mDriveSS = pDriveSS;
        this.mHoodSS = pHoodSS;
        this.mCANRanges = pCANRanges;
        this.mShooterSS = pShooterSS;
        this.mIntakeSS = pIntake;
        this.mFuelInjectorSS = pInjectorSS;
        this.mClimbSS = pClimbSS;
        this.mDriveSS.setDefaultCommand(mDriveSS.getDriveManager().setToTeleop());
    }

    public void initBindings() {
        initTriggers();

        mDriveSS.getDriveManager().acceptJoystickInputs(
                () -> -mPilotController.getLeftY(),
                () -> -mPilotController.getLeftX(),
                () -> -mPilotController.getRightX(),
                () -> mPilotController.getPOVAngle());

        initCompBindings();
        testBindings();
    }

    private Command rumbleForShift(double pRumbleTimeSec) {
                return rumbleForShift(pRumbleTimeSec, 0.7);
    }

    private Command rumbleForShift(double pRumbleTimeSec, double pRumbleValue) {
                return new SequentialCommandGroup(
                        new InstantCommand(() -> mPilotController.setRumble(RumbleType.kBothRumble, pRumbleValue)),
                        new WaitCommand(pRumbleTimeSec),
                        new InstantCommand(() -> mPilotController.setRumble(RumbleType.kBothRumble, 0))
                );
    }

    private Trigger constructPreshotPos(Trigger pBtn, ShooterStates pFlywheelState, HoodStates pHoodState) {
        return pBtn
                .onTrue(mShooterSS.setStateCmd(pFlywheelState))
                .onTrue(mHoodSS.setStateCmd(pHoodState))
                .onFalse(mShooterSS.setStateCmd(ShooterStates.STANDBY_VOLTAGE))
                .onFalse(mHoodSS.setStateCmd(HoodStates.MIN));
    }

    public void initCompBindings() {
        boolean useAnshulCompact = true;

        // PILOT CONTROLS
        Trigger wantToAutoAlignToHubBtn = mPilotController.a().and(kUsingPilotGunner);
        Trigger wantToEnableC4LED = mPilotController.rightTrigger().and(kUsingPilotGunner);
        // Trigger wantToSlowStowBtn = mPilotController.leftBumper().and(kUsingPilotGunner);
        Trigger wantToSafeStowBtn = mPilotController.leftBumper().and(kUsingPilotGunner); //TODO: make sure to uncomment this
        Trigger wantToLineAlignToBumpBtn = mPilotController.b().and(kUsingPilotGunner);
        Trigger wantToLineAlignToClimbBtn = mPilotController.a().and(kUsingPilotGunner);
        Trigger wantToYawToBump =
        mPilotController.rightTrigger().and(kUsingPilotGunner);
        Trigger wantToLineAlignToTrenchBtn = mPilotController.y().and(kUsingPilotGunner);
        Trigger wantsToHeadingXLockBtn = mPilotController.x().and(kUsingPilotGunner);
        Trigger wantToIntakeBtn = mPilotController.rightBumper().and(kUsingPilotGunner);

        // GUNNER CONTROLS
        Trigger wantToDynamicShootBtn =   mGunnerButtonboard.blueSquareRight().and(kUsingPilotGunner);
        Trigger wantToDeployClimbBtn =     mGunnerButtonboard.whiteUpwardTriangleLeft().and(kUsingPilotGunner);
        Trigger wantToClimbAscendBtn =     mGunnerButtonboard.whiteDownwardTriangleLeft().and(kUsingPilotGunner);
        Trigger wantToSlowStowBtn =        mGunnerButtonboard.greenDiamondLeft().and(kUsingPilotGunner);
        Trigger wantToStowIntakeBtn =      mGunnerButtonboard.yellowTriangleLeft().and(kUsingPilotGunner);
        Trigger wantToIntakeOutBtn =       mGunnerButtonboard.redTriangleLeft().and(kUsingPilotGunner);
        Trigger wantToOuttakeBtn =         mGunnerButtonboard.redCircleBottom().and(kUsingPilotGunner);
        Trigger wantToIntakeRollerBtn =    mGunnerButtonboard.blueCircleBottom().and(kUsingPilotGunner);
        Trigger wantToRevFlywheelsBtn =    mGunnerButtonboard.yellowTriangleRight().and(kUsingPilotGunner);
        Trigger wantToStopFlywheelsBtn =   mGunnerButtonboard.redTriangleRight().and(kUsingPilotGunner);
        Trigger wantToBumpShotBtn =        mGunnerButtonboard.redSquareCenter().and(kUsingPilotGunner);
        Trigger wantToTowerShotBtn =       mGunnerButtonboard.blueSquareCenter().and(kUsingPilotGunner);
        Trigger wantToTrenchShotBtn =      mGunnerButtonboard.greenSquareCenter().and(kUsingPilotGunner);
        Trigger wantToCornerShotBtn =      mGunnerButtonboard.yellowSquareCenter().and(kUsingPilotGunner);
        Trigger wantToHailstormBtn =       mGunnerButtonboard.whiteSquareRight().and(kUsingPilotGunner);
        Trigger wantToSnowPlowBtn =        mGunnerButtonboard.yellowRectangleRight().and(kUsingPilotGunner);
        Trigger wantToDisableCamsBtn =     mGunnerButtonboard.orangePilotTop().and(kUsingPilotGunner);
        Trigger wantToDisableCANRangeBtn = mGunnerButtonboard.purplePilotTop().and(kUsingPilotGunner);
        Trigger wantToDisableSoftLimits =  mGunnerButtonboard.bluePilotTop().and(kUsingPilotGunner);

        // OTHER CONDITIONAL TRIGGERS
        Trigger autonomousWorking = new Trigger(() -> true);
        Trigger inCenter = new Trigger(() -> GameGoalPoseChooser.inCenter(mDriveSS.getPoseEstimate()));
        Trigger isRobotMoving = new Trigger(() -> !mDriveSS.isRobotStationary());
        Trigger driveIsHeadingXLocked = new Trigger(
                () -> mDriveSS.getDriveManager().getDriveState().equals(DriveState.HEADING_X_LOCK));

        Trigger inNoHoodZone = new Trigger(() -> (GameGoalPoseChooser.inLeftTrenchYRange(mDriveSS.getPoseEstimate()) ||
                GameGoalPoseChooser.inEitherTrenchXRange(mDriveSS.getPoseEstimate()))
                ||
                (GameGoalPoseChooser.inRightTrenchYRange(mDriveSS.getPoseEstimate()) ||
                        GameGoalPoseChooser.inEitherTrenchXRange(mDriveSS.getPoseEstimate())));
        Trigger inSuperNoHoodZone = new Trigger(
                () -> (GameGoalPoseChooser.inLeftTrenchYRange(mDriveSS.getPoseEstimate()) ||
                        GameGoalPoseChooser.inEitherSuperTrenchXRange(mDriveSS.getPoseEstimate()))
                        ||
                        (GameGoalPoseChooser.inRightTrenchYRange(mDriveSS.getPoseEstimate()) ||
                                GameGoalPoseChooser.inEitherSuperTrenchXRange(mDriveSS.getPoseEstimate())));
        prevHoodState = mHoodSS.getHoodState();
        Trigger flywheelAtGoal = new Trigger(() -> mShooterSS.atLatestClosedLoopGoal());
        Trigger hoodAtGoal = new Trigger(() -> mHoodSS.atGoal());
        Trigger headingAlignAtGoal = new Trigger(mDriveSS.getDriveManager().waitUntilHeadingAlignFinishes());
        Trigger shooterAtGoal = hoodAtGoal.and(flywheelAtGoal);
        Trigger fuelPumpAtGoal = new Trigger(() -> mShooterSS.atGoal());
        // Trigger atPositionGoal = new
        // Trigger(mDriveSS.getDriveManager().waitUntilAutoAlignFinishes());
        Trigger atHeadingGoal = (headingAlignAtGoal.or(driveIsHeadingXLocked));
        Trigger atLineGoal = new Trigger(() -> mDriveSS.getDriveManager().getLineAlignController().atGoal());

        Trigger anyCANRangesTriggered = new Trigger(() -> mCANRanges.anyHasFuel());
        Trigger allCANRangesTriggered = new Trigger(() -> mCANRanges.allHasFuel());

        Trigger wantToShoot = new Trigger(() -> {
            return wantToSnowPlowBtn.getAsBoolean()
                    ||
                    wantToHailstormBtn.getAsBoolean()
                    ||
                    wantToDynamicShootBtn.getAsBoolean();
        });

        final double kShootingReadyDebounceSeconds = 0.35;
        final double kKickbackTime = 0.5;

        Trigger dynamicShootReady = shooterAtGoal.and(headingAlignAtGoal.or(atHeadingGoal)).and(fuelPumpAtGoal)
                .debounce(kShootingReadyDebounceSeconds, DebounceType.kBoth);
        double dynamicShootTimeout = 1.5;

        // Trigger staticShootReady = shooterAtGoal.debounce(kShootingReadyDebounceSeconds, DebounceType.kBoth);

        Trigger staticShootReady = shooterAtGoal.and(fuelPumpAtGoal).and(headingAlignAtGoal.or(atHeadingGoal)).debounce(kShootingReadyDebounceSeconds, DebounceType.kBoth);
        double staticShootTimeout = 4.0;

        anyCANRangesTriggered.and(wantToShoot.negate())
                .onTrue(mFuelInjectorSS.setStateCmd(FuelInjectorState.KICKBACK))
                .onFalse(mFuelInjectorSS.setStateCmd(FuelInjectorState.IDLE));

        new Trigger(() -> mIntakeSS.safeToRunRollers())
                .onFalse(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE));

        constructPreshotPos(wantToTrenchShotBtn, ShooterStates.TRENCH_VELOCITY, HoodStates.TRENCH_ANGLE);
        constructPreshotPos(wantToBumpShotBtn, ShooterStates.BUMP_VELOCITY, HoodStates.BUMP_ANGLE);
        constructPreshotPos(wantToCornerShotBtn, ShooterStates.CORNER_VELOCITY, HoodStates.CORNER_ANGLE);
        constructPreshotPos(wantToTowerShotBtn, ShooterStates.TOWER_VELOCITY, HoodStates.TOWER_ANGLE);

        Trigger wantToStaticShoot = wantToTrenchShotBtn.or(wantToBumpShotBtn).or(wantToCornerShotBtn)
                .or(wantToTowerShotBtn);

        // wantToEnableC4LED       
        //         .onTrue(new InstantCommand(() -> mPD.setSwitchableChannel(true)))
        //         .onFalse(new InstantCommand(() -> mPD.setSwitchableChannel(false)));

        // If at goal, shoot it in
        wantToStaticShoot.and(staticShootReady)
                .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.INTAKE))
                .onTrue(mFuelInjectorSS.setStateCmd(FuelInjectorState.INTAKE));

        /* Debounce if not at goal */
        wantToStaticShoot.debounce(staticShootTimeout, DebounceType.kRising).and(staticShootReady.negate()).and(wantToDisableCANRangeBtn)
                .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.INTAKE))
                .onTrue(mFuelInjectorSS.setStateCmd(FuelInjectorState.INTAKE));

        wantToTrenchShotBtn
                .onTrue(mShooterSS.setStateCmd(ShooterStates.TRENCH_VELOCITY));

        // wantToSlowStow.and(isRackMoving)
        wantToSlowStowBtn
                // .onTrue(useAnshulCompact ? mIntakeSS.anshulCompact() : mIntakeSS.SlowStow())
                .onTrue(mIntakeSS.setSlowStow())
                .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.INTAKE))
                .onFalse(mIntakeSS.setRackStateCmd(IntakeRackState.INTAKE))
                .onFalse(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE));
        
        wantToSlowStowBtn
                .whileTrue(mIntakeSS.setSlowStow())
                .whileFalse(mIntakeSS.setRackStateCmd(IntakeRackState.STOPPED));

        wantToDisableCANRangeBtn
                .onTrue(new InstantCommand(()->mShooterSS.setCANRangeUsage(false)))
                .onFalse(new InstantCommand(()->mShooterSS.setCANRangeUsage(true)));

        // wantToSlowStow.and(isRackMoving.negate())
        // .onTrue(new
        // WaitCommand(kKickbackTime).andThen(mFuelPumpSS.setStateCmd(closedLoopFuelPump
        // ? FuelPumpState.INTAKE_VELOCITY : FuelPumpState.INTAKE_VOLT)));
        // .onFalse(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE));

        wantToDisableSoftLimits
                .onTrue(new InstantCommand(()->mIntakeSS.disableRackSoftLimits()))
                .onFalse(new InstantCommand(()->mIntakeSS.enableRackSoftLimits()));

        wantToStowIntakeBtn.and(wantToDisableSoftLimits.negate())
                .onTrue(mIntakeSS.setRackStateCmd(IntakeRackState.STOW))
                .onFalse(mIntakeSS.setRackStateCmd(IntakeRackState.STOPPED));

        wantToStowIntakeBtn.and(wantToDisableSoftLimits)
                .onTrue(mIntakeSS.setRackStateCmd(IntakeRackState.MANUAL_IN))
                .onFalse(mIntakeSS.setRackStateCmd(IntakeRackState.STOPPED));

        wantToIntakeOutBtn.and(wantToDisableSoftLimits.negate())
                .onTrue(mIntakeSS.setRackStateCmd(IntakeRackState.INTAKE))
                .onFalse(mIntakeSS.setRackStateCmd(IntakeRackState.STOPPED));

        wantToIntakeOutBtn.and(wantToDisableSoftLimits)
                .onTrue(mIntakeSS.setRackStateCmd(IntakeRackState.MANUAL_OUT))
                .onFalse(mIntakeSS.setRackStateCmd(IntakeRackState.STOPPED));

        wantToOuttakeBtn
                .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.OUTTAKE))
                .onTrue(mFuelInjectorSS.setStateCmd(FuelInjectorState.OUTTAKE))
                .onFalse(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE))
                .onFalse(mFuelInjectorSS.setStateCmd(FuelInjectorState.IDLE));

        wantToIntakeRollerBtn
                .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.INTAKE))
                .onFalse(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE));

        wantToRevFlywheelsBtn
                .onTrue(mShooterSS.setStateCmd(ShooterStates.REV_VOLTAGE));

        wantToStopFlywheelsBtn
                .onTrue(mShooterSS.setStateCmd(ShooterStates.STOPPED));

        mPilotController.startButton()
                .onTrue(Commands.runOnce(() -> mDriveSS.resetGyro()));

        wantsToHeadingXLockBtn
                .onTrue(mDriveSS.getDriveManager().setToHeadingXLock());

        wantToDeployClimbBtn
                .onTrue(mClimbSS.setStateCmd(ClimbState.UP))
                .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.OUTTAKE))
                .onTrue(mIntakeSS.setRackStateCmd(IntakeRackState.STOW))
                .onFalse(mClimbSS.setStateCmd(ClimbState.STOP));

        wantToClimbAscendBtn
                .onTrue(mClimbSS.setStateCmd(ClimbState.DOWN))
                .onFalse(mClimbSS.setStateCmd(ClimbState.STAY_ROBOT));

        // wantsToHeadingXLock
        // .onFalse(new ConditionalCommand(
        // new InstantCommand(),
        // mDriveSS.getDriveManager().setToTeleop(),
        // wantToShoot));

        wantsToHeadingXLockBtn.negate().and(wantToShoot.negate())
                .onTrue(mDriveSS.getDriveManager().setToTeleop());

        /* AUTO ALIGNS TO HUB AND SHOOTS */
        // wantToCloseShoot
        // .onTrue(mShooterSS.setStateCmd(ShooterStates.CLOSE_VELOCITY))
        // .onTrue(mHoodSS.setStateCmd(HoodStates.CLOSE_SHOT));

        // wantToAutoAlignToHubBtn.and(autonomousWorking)
        //         .onTrue(mDriveSS.getDriveManager().setToGenericAutoAlign(
        //                 () -> GameGoalPoseChooser.getCloseShotPose(),
        //                 ConstraintType.LINEAR))
        //         .onFalse(mDriveSS.getDriveManager().setToTeleop());

        wantToLineAlignToTrenchBtn.and(autonomousWorking)
                .onTrue(
                        mDriveSS.getDriveManager().setToGenericLineAlign(
                                () -> GameGoalPoseChooser.getClosestTrench(mDriveSS.getPoseEstimate()),
                                () -> Rotation2d.kZero,
                                () -> 1,
                                () -> AllianceFlipUtil.shouldFlip()))
                .onFalse(mDriveSS.getDriveManager().setToTeleop());

        wantToLineAlignToBumpBtn.and(autonomousWorking)
                .onTrue(Commands.runOnce(() -> inCenterFlag = inCenter.getAsBoolean()));

        wantToLineAlignToBumpBtn
                .onFalse(noneTraversalHeadingState().andThen(mDriveSS.getDriveManager().setToTeleop()));

        /* TRAVERSAL Logic */
        wantToLineAlignToBumpBtn.and(() -> inCenterFlag)
                .onTrue(centerTraversalHeadingState()
                        .andThen(
                                mDriveSS.getDriveManager().setToGenericLineAlign(
                                        () -> GameGoalPoseChooser.getClosestBump(mDriveSS.getPoseEstimate()),
                                        () -> Rotation2d.k180deg,
                                        () -> 1,
                                        () -> true)));

        wantToLineAlignToBumpBtn.and(() -> !inCenterFlag)
                .onTrue(allianceTraversalHeadingState().andThen(
                        mDriveSS.getDriveManager().setToGenericLineAlign(
                                () -> GameGoalPoseChooser.getClosestBump(mDriveSS.getPoseEstimate())
                                        .transformBy(new Transform2d(new Translation2d(), Rotation2d.k180deg)),
                                () -> Rotation2d.k180deg,
                                () -> 1,
                                () -> false)));

        wantToLineAlignToClimbBtn
                .onTrue(
                        mDriveSS.getDriveManager().setToGenericLineAlign(
                                () -> GameGoalPoseChooser.getClosestClimbPose(mDriveSS.getPoseEstimate()),
                                () -> Rotation2d.kZero,
                                () -> 0.4,
                                () -> true).onlyWhile(atLineGoal.negate()))

                                // .andThen(mDriveSS.getDriveManager()
                                //         .setToGenericAutoAlign(
                                //                 () -> GameGoalPoseChooser
                                //                         .getClosestClimbPose(mDriveSS.getPoseEstimate()),
                                //                 ConstraintType.LINEAR)))
                .onFalse(mDriveSS.getDriveManager().setToTeleop());

        // wantToCloseShoot.and(autonomousWorking).and(wantsToHeadingXLock.negate()).and(driveIsHeadingXLocked)
        // .onTrue(mDriveSS.getDriveManager().setToGenericAutoAlign(
        // () -> GameGoalPoseChooser.getCloseShotPose(),
        // ConstraintType.LINEAR));

        // wantToCloseShoot.and(shooterAtGoal.and(atPositionalGoal).debounce(kShootingReadyDebounceSeconds,
        // DebounceType.kBoth))
        // .onTrue(mFuelPumpSS.setStateCmd(FuelPumpState.INTAKE_VOLT))
        // .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.INTAKE))
        // .onTrue(mIntakeSS.SlowStowPivotRepeat());

        // wantToCloseShoot
        // .onFalse(mFuelPumpSS.setStateCmd(FuelPumpState.STOPPED))
        // .onFalse(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE))
        // .onFalse(mIntakeSS.setPivotStateCmd(IntakePivotStates.INTAKE))
        // .onFalse(mHoodSS.setStateCmd(HoodStates.MIN));

        /* SHOOTS FROM ANYWHERE IN OUR FLYWHEEL RANGE */
        wantToDynamicShootBtn
                .onTrue(mFuelInjectorSS.setStateCmd(FuelInjectorState.IDLE))
                .onTrue(mShooterSS.setStateCmd(ShooterStates.SHOTMAP_VELOCITY))
                .onTrue(mHoodSS.setStateCmd(HoodStates.SHOTMAP_POSITION));

        // TODO: ADD BACK DRIVE AUTOMATION
        wantToDynamicShootBtn.and(autonomousWorking)
                .onTrue(mDriveSS.getDriveManager().setToGenericHeadingAlign(
                        () -> GameGoalPoseChooser.turnFromHub(mDriveSS.getPoseEstimate()),
                        () -> GameGoalPoseChooser.getHub()))
                .onFalse(mDriveSS.getDriveManager().setToTeleop());

        wantToDynamicShootBtn.and(autonomousWorking).and(wantsToHeadingXLockBtn.negate()).and(driveIsHeadingXLocked)
                .onTrue(mDriveSS.getDriveManager().setToGenericHeadingAlign(
                        () -> GameGoalPoseChooser.turnFromHub(mDriveSS.getPoseEstimate()),
                        () -> GameGoalPoseChooser.getHub()));

        // If at goal, shoot it in
        wantToDynamicShootBtn.and(dynamicShootReady)
                .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.INTAKE))
                .onTrue(mShooterSS.setStateCmd(ShooterStates.SHOTMAP_VELOCITY))
                .onTrue(mFuelInjectorSS.setStateCmd(FuelInjectorState.INTAKE));

        /* Debounce if not at goal */
        wantToDynamicShootBtn.debounce(dynamicShootTimeout, DebounceType.kRising).and(dynamicShootReady.negate()).and(wantToDisableCANRangeBtn.negate())
                .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.INTAKE))
                .onTrue(mShooterSS.setStateCmd(ShooterStates.SHOTMAP_VELOCITY))
                .onTrue(mFuelInjectorSS.setStateCmd(FuelInjectorState.INTAKE));

        wantToDynamicShootBtn
                .onFalse(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE))
                .onFalse(mIntakeSS.setRackStateCmd(IntakeRackState.INTAKE))
                .onFalse(mFuelInjectorSS.setStateCmd(FuelInjectorState.IDLE))
                .onFalse(mHoodSS.setStateCmd(HoodStates.MIN));

        /* Feeding Logic */
        wantToSnowPlowBtn
                .onTrue(mShooterSS.setStateCmd(ShooterStates.FEED_VELOCITY))
                .onTrue(mHoodSS.setStateCmd(HoodStates.MAX));

        wantToHailstormBtn
                .onTrue(mShooterSS.setStateCmd(ShooterStates.OPPONENT_FEED_VELOCITY))
                .onTrue(mHoodSS.setStateCmd(HoodStates.OPPONENT_FEED_ANGLE));

        // wantToHailstormBtn.and(() -> mShooterSS.isHailstormReady())
        //         .onTrue(mFuelInjectorSS.setStateCmd(FuelInjectorState.INTAKE))
        //         .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.INTAKE));

        wantToSnowPlowBtn.or(wantToHailstormBtn).and(fuelPumpAtGoal)
                .and((shooterAtGoal.and(atHeadingGoal).debounce(kShootingReadyDebounceSeconds, DebounceType.kBoth)))
                .onTrue(mFuelInjectorSS.setStateCmd(FuelInjectorState.INTAKE))
                .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.INTAKE));

        wantToSnowPlowBtn.debounce(0.75, DebounceType.kRising)
                .or(wantToHailstormBtn.debounce(0.75, DebounceType.kRising))
                .and(((shooterAtGoal.and(atHeadingGoal).debounce(kShootingReadyDebounceSeconds, DebounceType.kBoth)))
                        .negate())
                .onTrue(mFuelInjectorSS.setStateCmd(FuelInjectorState.INTAKE))
                .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.INTAKE));

        wantToHailstormBtn.or(wantToSnowPlowBtn)
                .onFalse(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE))
                .onFalse(mIntakeSS.setRackStateCmd(IntakeRackState.INTAKE))
                .onFalse(mFuelInjectorSS.setStateCmd(FuelInjectorState.IDLE))
                .onFalse(mHoodSS.setStateCmd(HoodStates.MIN));

        // wantToHailstormBtn
        //         .onTrue(mShooterSS.setStateCmd(ShooterStates.HAILSTORM_VOLTAGE))
        //         .onTrue(mFuelPumpSS.setStateCmd(FuelPumpState.INTAKE_VOLT))
        //         .onTrue(mHoodSS.setStateCmd(HoodStates.OPPONENT_FEED_ANGLE));

        // wantToHailstormBtn.and(() -> mShooterSS.isHailstormReady())
        //         .onTrue(mFuelInjectorSS.setStateCmd(FuelInjectorState.INTAKE))
        //         .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.INTAKE));

        // wantToSnowPlowBtn.and(fuelPumpAtGoal)
        //         .and((shooterAtGoal.and(atHeadingGoal).debounce(kShootingReadyDebounceSeconds, DebounceType.kBoth)))
        //         .onTrue(mFuelInjectorSS.setStateCmd(FuelInjectorState.INTAKE))
        //         .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.INTAKE));

        // wantToSnowPlowBtn.debounce(0.75, DebounceType.kRising)
        //         .or(wantToHailstormBtn.debounce(0.75, DebounceType.kRising))
        //         .and(((shooterAtGoal.and(atHeadingGoal).debounce(kShootingReadyDebounceSeconds, DebounceType.kBoth)))
        //                 .negate())
        //         .onTrue(mFuelInjectorSS.setStateCmd(FuelInjectorState.INTAKE))
        //         .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.INTAKE));

        // wantToHailstormBtn.or(wantToSnowPlowBtn)
        //         .onFalse(mFuelPumpSS.setStateCmd(FuelPumpState.STOPPED))
        //         .onFalse(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE))
        //         .onFalse(mIntakeSS.setRackStateCmd(IntakeRackState.INTAKE))
        //         .onFalse(mFuelInjectorSS.setStateCmd(FuelInjectorState.IDLE))
        //         .onFalse(mHoodSS.setStateCmd(HoodStates.MIN));

        /* Makes flywheel stand by */
        wantToShoot.negate()
                .onTrue(mShooterSS.setStateCmd(ShooterStates.STANDBY_VELOCITY))
                .onTrue(mHoodSS.setStateCmd(HoodStates.MIN));

        /* HOOD PROTECTION LOGIC */
        // inSuperNoHoodZone.or(inNoHoodZone.and(isRobotMoving))
        //         .onTrue(Commands.runOnce(() -> prevHoodState = mHoodSS.getHoodState())
        //                 .andThen(mHoodSS.setStateCmd(HoodStates.MIN)))
        //         .onFalse(mHoodSS.setStateCmd(prevHoodState));

        /* INTAKE LOGIC */
        wantToIntakeBtn
                .onTrue(mIntakeSS.setRackStateCmd(IntakeRackState.INTAKE))
                .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.INTAKE))
                // .onTrue(mDriveSS.getDriveManager().setToTeleopSniper())
                .onFalse(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE));

        
        // wantToSlowStowBtn
        //         .onTrue(mIntakeSS.setRackStateCmd(IntakeRackState.SLOW_STOW))
        //         .onFalse(mIntakeSS.setRackStateCmd(IntakeRackState.STOPPED));
        // .onFalse(mDriveSS.getDriveManager().setToTeleop());

        // wantToSafeStowBtn
        //         .onTrue(mIntakeSS.setRackStateCmd(IntakeRackState.SAFESTOW));

        // wantToInitiateClimb
        // .onTrue(mClimbSS.goUpTillClimbHeightThenStay())
        // .onFalse(mClimbSS.setStateCmd(ClimbState.STAY));

        // wantToEndClimb
        // .onTrue(mClimbSS.goDownTillClimbedThenStayClimbed())
        // .onFalse(mClimbSS.setStateCmd(ClimbState.STAY));
    }

    public void testBindings() {
        mPilotController.startButton().and(isTesting())
            .onTrue(Commands.runOnce(() -> mDriveSS.resetGyro()));

        mPilotController.a().and(isTesting())
            .onTrue(mShooterSS.setStateCmd(ShooterStates.TUNING_AMPERAGE))
            .onFalse(mShooterSS.setStateCmd(ShooterStates.STOPPED));

        mPilotController.b().and(isTesting())
            .onTrue(mShooterSS.setStateCmd(ShooterStates.TUNING_VELOCITY))
            .onFalse(mShooterSS.setStateCmd(ShooterStates.STOPPED));

        mPilotController.x().and(isTesting())
            .onTrue(mIntakeSS.setSlowStow())
            .onFalse(mIntakeSS.setRackStateCmd(IntakeRackState.STOPPED));

        mPilotController.y().and(isTesting())
            .onTrue(mIntakeSS.setRackStateCmd(IntakeRackState.TUNING_SETPOINT))
            .onFalse(mIntakeSS.setRackStateCmd(IntakeRackState.STOPPED));

        mPilotController.leftBumper().and(isTesting())
                .whileTrue(mIntakeSS.setSlowStow())
                .whileFalse(mIntakeSS.setRackStateCmd(IntakeRackState.STOPPED));
                
        // mPilotController.b().and(isTesting())
        //     .onTrue(mHoodSS.setStateCmd(HoodStates.TUNING_VOLTAGE))
        //     .onFalse(mHoodSS.setStateCmd(HoodStates.STOPPED));

        // mPilotController.y().and(isTesting())
        //     .onTrue(mHoodSS.setStateCmd(HoodStates.TUNING_SETPOINT))
        //     .onFalse(mHoodSS.setStateCmd(HoodStates.STOPPED));

        // mPilotController.a()
        //     .onTrue(mFuelInjectorSS.setStateCmd(FuelInjectorState.INTAKE))
        //     .onFalse(mFuelInjectorSS.setStateCmd(FuelInjectorState.IDLE));

        // mPilotController.b().and(isTesting())
        //     .onTrue(mFuelPumpSS.setStateCmd(FuelPumpState.TUNING_VOLT))
        //     .onFalse(mFuelPumpSS.setStateCmd(FuelPumpState.STOPPED));

        // mPilotController.b().and(isTesting())
        //     .onTrue(mFuelPumpSS.setStateCmd(FuelPumpState.INTAKE_VELOCITY))
        //     .onFalse(mFuelPumpSS.setStateCmd(FuelPumpState.STOPPED));

        // mPilotController.x().and(isTesting())
        //     .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.INTAKE))
        //     .onFalse(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE));
    }

    public void initTriggers() {
        // new Trigger(() -> HubShift.getShiftedShiftInfo().remainingTime() <= 5 && HubShift.getShiftedShiftInfo().remainingTime() > 3)
        //         .onTrue(rumbleForShift(1.0, 0.4));

        final double kSecLeftToRumble = 4;
        final double kRumbleTime = 0.5;
        final double kRumbleWait = 0.2;
        final double kRumbleStrength = 0.8;

        // new Trigger(() -> HubShift.getShiftedShiftInfo().remainingTime() <= kSecLeftToRumble)
        //         .onTrue(
        //                 rumbleForShift(kRumbleTime, kRumbleStrength)
        //                 .andThen(new WaitCommand(kRumbleWait))
        //                 .andThen(rumbleForShift(kRumbleTime, kRumbleStrength))
        //         );

        new Trigger(() -> DriverStation.isTeleopEnabled())
                .onTrue(mDriveSS.getDriveManager().setToTeleop())
                .onTrue(mIntakeSS.setRollerStateCmd(IntakeRollerState.IDLE))
                .onTrue(mShooterSS.setStateCmd(ShooterStates.STANDBY_VELOCITY));
                
        new Trigger(() -> DriverStation.isFMSAttached()).and(() -> DriverStation.isTeleopEnabled())
                .onTrue(mHoodSS.setStateCmd(HoodStates.MIN))
                .onTrue(mIntakeSS.setRackStateCmd(IntakeRackState.INTAKE));

        // Trigger climbButton = new Trigger(() ->
        // mHBSS.getClimbButtonUpdateInputs().iPressed);
        // climbButton.and(() -> DriverStation.isDisabled() &&
        // !DriverStation.isFMSAttached())
        // .onFalse(new InstantCommand(() ->
        // mClimbSS.changeClimbNeutralMode(NeutralModeValue.Coast)).ignoringDisable(true))
        // .onTrue(new InstantCommand(() ->
        // mClimbSS.changeClimbNeutralMode(NeutralModeValue.Brake)).ignoringDisable(true));
    }

    public Command rumbleDriverController() {
        return Commands.startEnd(
                () -> mPilotController.setRumble(RumbleType.kBothRumble, 1.0),
                () -> mPilotController.setRumble(RumbleType.kBothRumble, 0.5));
    }

    public BooleanSupplier isTesting() {
        return () -> !kUsingPilotGunner.get();
    }

    public Command centerTraversalHeadingState() {
        return Commands.runOnce(() -> mHeadingTraversalState = HeadingTraversalState.CENTER);
    }

    public Command allianceTraversalHeadingState() {
        return Commands.runOnce(() -> mHeadingTraversalState = HeadingTraversalState.ALLIANCE);
    }

    public Command noneTraversalHeadingState() {
        return Commands.runOnce(() -> mHeadingTraversalState = HeadingTraversalState.ALLIANCE);
    }
}
