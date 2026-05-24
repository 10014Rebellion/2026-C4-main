package frc.robot.systems.auton;

import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import frc.lib.telemetry.Telemetry;

import frc.robot.commands.FollowPathCommand;
import frc.robot.commands.SequentialEndingCommandGroup;
import frc.robot.game.FieldConstants;
import frc.robot.game.GameGoalPoseChooser;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoEvent;
import frc.robot.systems.auton.routines.DoubleSwipe;
import frc.robot.systems.auton.routines.ShootPreload;
import frc.robot.systems.auton.routines.SingleSwipe;
import frc.robot.systems.auton.routines.SingleSwipeClimb;
import frc.robot.systems.climb.ClimbSS;
import frc.robot.systems.climb.ClimbSS.ClimbState;
import frc.robot.systems.drive.Drive;
import frc.robot.systems.drive.DriveManager.DriveState;
import frc.robot.systems.drive.controllers.HolonomicController.ConstraintType;
import frc.robot.systems.efi.FuelInjectorSS;
import frc.robot.systems.efi.FuelInjectorSS.FuelInjectorState;
import frc.robot.systems.intake.Intake;
import frc.robot.systems.intake.rack.IntakeRackSS.IntakeRackState;
import frc.robot.systems.intake.roller.IntakeRollerSS.IntakeRollerState;
import frc.robot.systems.shooter.flywheels.FlywheelsSS;
import frc.robot.systems.shooter.flywheels.FlywheelsSS.FlywheelStates;
import frc.robot.systems.shooter.fuelpump.FuelPumpSS;
import frc.robot.systems.shooter.fuelpump.FuelPumpSS.FuelPumpState;
import frc.robot.systems.shooter.hood.HoodSS;
import frc.robot.systems.shooter.hood.HoodSS.HoodStates;
import frc.lib.math.AllianceFlipUtil;

public class AutonCommands extends SubsystemBase {
    public static final Pose2d kTopLeftBump = new Pose2d();
    public static final Pose2d kTopRightBump = new Pose2d();
    public static final Pose2d kBottomLeftBump = new Pose2d();
    public static final Pose2d kBottomRightBump = new Pose2d();

    private final Drive mRobotDrive;
    private final Intake mIntake;
    private final HoodSS mHoodSS;
    private final FuelPumpSS mFuelPumpSS;
    private final FlywheelsSS mFlywheelsSS;
    private final ClimbSS mClimbSS;
    private final FuelInjectorSS mFuelInjectorSS;

    private final SendableChooser<Supplier<Command>> mAutoChooser;
    private final LoggedDashboardChooser<Supplier<Command>> mAutoChooserLogged;

    private final AutoFactory mAutoFactory;

    private String[] usedPathNames = new String[] {
        "TL_Round_CS",
        "H_D",
        "BL_Catch",
        "BR_Catch",
        "TR_Round_CS",
        "L2_TL_TL_Single",
        "BSL_TL_Half",
        "BSR_TR_Half",
        "TLSub_Corall_Catch",
        "TL_BL_ BSL",
        "TL_TL_BSL",
        "TRSub_Corall_Catch",
        "TR_BR_BSR",
        "TR_TR_BSR",
        "L_IB_IC_ST",
        "L_ST_BUMP"
    };

    public AutonCommands(Drive pRobotDrive, Intake pIntake, FuelPumpSS pFuelPumpSS, HoodSS pHoodSS, FlywheelsSS pFlywheelsSS, ClimbSS pClimbSS, FuelInjectorSS pInjectorSS) {
        this.mRobotDrive = pRobotDrive;
        this.mIntake = pIntake;
        this.mHoodSS = pHoodSS;
        this.mFlywheelsSS = pFlywheelsSS;
        this.mFuelPumpSS = pFuelPumpSS;
        this.mClimbSS = pClimbSS;
        this.mFuelInjectorSS = pInjectorSS;

        mAutoFactory = new AutoFactory(
            mRobotDrive::getPoseEstimate, 
            mRobotDrive::setPose, 
            (SwerveSample s) -> {}, 
            true, 
            mRobotDrive,
            (sample, isStart) -> {
                Logger.recordOutput("Drive/Choreo/Trajectory", sample.getPoses());
                Logger.recordOutput("Drive/Choreo/IsTrajectoryRunning", isStart);
            });

        loadCacheForAllPaths();

        mAutoChooser = new SendableChooser<>();
        mAutoChooser.setDefaultOption("StationaryDefault", () -> backUpAuton());

        DoubleSwipe mLeftBumpDoubleSwipe = new DoubleSwipe(
            this,
            "LeftTrenchBumpDoubleSwipe",
            "L_IT_IC_ST",
            5.8,
            "L_ST_IB_ST_BUMP",
            3.7,
            0.0,
            false
        );

        tryToAddPathToChooser("LeftTrenchBumpDoubleSwipe",() -> mLeftBumpDoubleSwipe.getAuton());

        DoubleSwipe mRightTrenchBumpDoubleSwipe = new DoubleSwipe(
            this,
            "RightTrenchBumpDoubleSwipe",
            "R_IT_IC_ST",
            5.2,
            "R_ST_IB_ST_BUMP",
            4.41,
            0.0,
            false
        );

        tryToAddPathToChooser("RightTrenchBumpDoubleSwipe",() -> mRightTrenchBumpDoubleSwipe.getAuton());

        DoubleSwipe mLeftTrenchDoubleSwipe = new DoubleSwipe(
            this,
            "LeftTrenchDoubleSwipe",
            "L_IT_IC_ST",
            5.4,
            "L_ST_IB_ST",
            5.3,
            0.0,
            false
        );

        DoubleSwipe mLeftBumpDoubleSwipe2 = new DoubleSwipe(
            this,
            "WorldsLeftDoubleSwipe",
            "L_IB_IC_ST",
            6.4,
            "L_ST_BUMP",
            4.0,
            3.0,
            false
        );

        DoubleSwipe mRightBumpDoubleSwipe2 = new DoubleSwipe(
            this,
            "DNU_WorldsRightDoubleSwipe",
            "L_IB_IC_ST",
            6.4,
            "L_ST_BUMP",
            4.0,
            0.0,
            true
        );
        
        // new DoubleSwipe(
        //     this,
        //     "DNU_WorldsRightDoubleSwipe",
        //     "R_IB_IC_ST",
        //     7.2,
        //     "R_ST_BUMP",
        //     4.0,
        //     0.0,
        //     false
        // );

        DoubleSwipe mRightTrenchDoubleSwipe = new DoubleSwipe(
            this,
            "RightTrenchDoubleSwipe",
            "R_IT_IC_ST",
            4.9,
            "R_ST_IB_ST",
            5.0,
            0.0,
            false
        );

        DoubleSwipe q97DoubleSwipe = new DoubleSwipe(
            this,
            "RightTrenchDoubleSwipe",
            "R_IT_IC_ST",
            4.9,
            "R_ST_IB_ST",
            5.0,
            8.0,
            false
        );
        tryToAddPathToChooser("Q97", () -> q97DoubleSwipe.getAuton());

        SingleSwipe AroundTheWorldLeft = new SingleSwipe(
            this,
            "AroundTheWorldLeft", 
            "L_AroundTheWorld", 
            8.0, 
            5.06, 
            false);

        DoubleSwipe AroundTheWorldLeftDouble = new DoubleSwipe(
                this,
                "DoubleAroundTheWorldLeft",
                "L_AroundTheWorld",
                8.0,
                "L_AroundTheWorld2",
                5.0,
                3.0,
                false
            );

        tryToAddPathToChooser("DoubleAroundTheWorldLeft", () -> AroundTheWorldLeftDouble.getAuton());

        DoubleSwipe AroundTheWorldRightDouble = new DoubleSwipe(
                this,
                "DoubleAroundTheWorldRight",
                "L_AroundTheWorld",
                8.0,
                "L_AroundTheWorld2",
                5.0,
                3.0,
                true
            );

        tryToAddPathToChooser("DoubleAroundTheWorldRight", () -> AroundTheWorldRightDouble.getAuton());

        SingleSwipeClimb AroundTheWorldClimbLeft = new SingleSwipeClimb(
            this,
            "DNU_AroundTheWorldLeftBumpClimb", 
            "L_AroundTheWorldBump", 
            5.1, 
            3.0, 
            kBottomLeftBump, 
            false);

        SingleSwipe AroundTheWorldRight = new SingleSwipe(
            this, 
            "DNU_AroundTheWorldRight", 
            "L_AroundTheWorld", 
            8.0,
            5.0, 
            true);

        SingleSwipeClimb AroundTheWorldClimbRight = new SingleSwipeClimb(
            this,
            "DNU_AroundTheWorldClimbRight", 
            "L_AroundTheWorld", 
            8.0, 
            5.0, 
            kBottomLeftBump, 
            true);

            SingleSwipe AroundTheWorldLeftBump = new SingleSwipe(
                this,
            "EOU_AroundTheWorldLeftBump", 
                "L_AroundTheWorldBump", 
                5.1, 
                5.0, 
                false);
    
            SingleSwipeClimb AroundTheWorldClimbLeftBump = new SingleSwipeClimb(
                this,
                "DNU_AroundTheWorldLeftClimbBump", 
                "L_AroundTheWorldBump", 
                5.1, 
                5.0, 
                kBottomLeftBump, 
                false);
    
            SingleSwipe AroundTheWorldRightBump = new SingleSwipe(
                this, 
                "AroundTheWorldRightBump", 
                "L_AroundTheWorldBump", 
                5.1,
                5.0, 
                true);
    
            SingleSwipeClimb AroundTheWorldClimbRightBump = new SingleSwipeClimb(
                this,
                "DNU_AroundTheWorldClimbRightBump", 
                "L_AroundTheWorldBump", 
                5.1, 
                5.0, 
                kBottomLeftBump, 
                true);

        ShootPreload mPreload = new ShootPreload(
            this,
            "Preload", 
            () -> GameGoalPoseChooser.getHub().plus(new Transform2d(AllianceFlipUtil.shouldFlip() ? -2.0 : 2.0, 0.0, Rotation2d.kZero))
            );

        tryToAddPathToChooser(
            "DNU_Preload", 
            () -> mPreload.getAuton()
        );

        tryToAddPathToChooser(
            "LeftBumpDoubleSwipe", 
            () -> mLeftBumpDoubleSwipe.getAuton()
        );

        tryToAddPathToChooser(
            "LefttTrenchDoubleSwipe", 
            () -> mLeftTrenchDoubleSwipe.getAuton()
        );

        tryToAddPathToChooser(
            "AroundTheWorldLeft", 
            () -> AroundTheWorldLeft.getAuton()
        );

        tryToAddPathToChooser(
            "DNU_AroundTheWorldLeftClimb", 
            () -> AroundTheWorldClimbLeft.getAuton()
        );

        tryToAddPathToChooser(
            "WorldsLeftBumpDoubleSwipe", 
            () -> mLeftBumpDoubleSwipe2.getAuton()
        );

        tryToAddPathToChooser(
            "DNU_WorldsRightBumpDoubleSwipe", 
            () -> mRightBumpDoubleSwipe2.getAuton()
        );

        tryToAddPathToChooser(
            "RightTrenchDoubleSwipe", 
            () -> mRightTrenchDoubleSwipe.getAuton()
        );

        tryToAddPathToChooser(
            "DNU_AroundTheWorldRight", 
            () -> AroundTheWorldRight.getAuton()
        );

        tryToAddPathToChooser(
            "DNU_AroundTheWorldRightClimb", 
            () -> AroundTheWorldClimbRight.getAuton()
        );

        tryToAddPathToChooser(
            "EUO_AroundTheWorldLeftBump", 
            () -> AroundTheWorldLeftBump.getAuton()
        );

        tryToAddPathToChooser(
            "DNU_AroundTheWorldLeftClimbBump", 
            () -> AroundTheWorldClimbLeftBump.getAuton()
        );

        tryToAddPathToChooser(
            "DNU_AroundTheWorldRightBump", 
            () -> AroundTheWorldRightBump.getAuton()
        );

        tryToAddPathToChooser(
            "DNU_AroundTheWorldRightClimbBump", 
            () -> AroundTheWorldClimbRightBump.getAuton()
        );

        mAutoChooserLogged = new LoggedDashboardChooser<>("Autos", mAutoChooser);
    }

    ///////////////// PATH CHAINING LOGIC \\\\\\\\\\\\\\\\\\\\\\
    public Command backUpAuton() {
        return new InstantCommand();
    }

    ///////////////// SUPERSTRUCTURE COMMANDS AND DATA \\\\\\\\\\\\\\\\\\\\\
    public Command endAuto(AutoEvent auto) {
        return Commands.runOnce(() -> auto.cancel());
    }

    public Trigger inIntakeRange(AutoEvent auto) {
        return auto.loggedCondition(auto.getName()+"/InIntakeRange", 
        () -> {
            if(DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue)) {
                return mRobotDrive.getPoseEstimate().getX() > 5.0;
            } else {
                return mRobotDrive.getPoseEstimate().getX() < AllianceFlipUtil.applyX(5.0);
            }
        }, 
        true);
    }

    public Trigger inScoringRange(AutoEvent auto) {
        return auto.loggedCondition(
            "InScoringRange", 
            () -> {
                if(DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue)) {
                    return mRobotDrive.getPoseEstimate().getX() < 4.0;
                } else {
                    return mRobotDrive.getPoseEstimate().getX() > AllianceFlipUtil.applyX(4.0);
                }
            }, 
            true);
    }


    /* ACTION TRIGGERS */
    public Trigger traversePathWithIntakeOutOnly(double delaySeconds, Command pathCommand, Trigger condition, String pathName, AutoEvent routine) {
        SequentialEndingCommandGroup pathCommandEnding = new SequentialEndingCommandGroup(pathCommand);

        condition
            .onTrue(Commands.waitSeconds(delaySeconds).andThen(pathCommandEnding))
            .onTrue(mFlywheelsSS.setStateCmd(FlywheelStates.STANDBY_VELOCITY))
            .onTrue(mHoodSS.setStateCmd(HoodStates.MIN))
            .onTrue(mFuelPumpSS.setStateCmd(FuelPumpState.STOPPED))
            .onTrue(mIntake.setRollerStateCmd(IntakeRollerState.IDLE))
            .onTrue(mIntake.setRackStateCmd(IntakeRackState.INTAKE))
            .onTrue(mFuelInjectorSS.setStateCmd(FuelInjectorState.IDLE));

        return routine.loggedCondition(
            pathName+"/HasEnded", 
            () -> pathCommandEnding.hasEnded(), 
            true);
    }

    public Trigger traversePathWithIntakeOutOnly(Command pathCommand, Trigger condition, String pathName, AutoEvent routine) {
        return traversePathWithIntakeOutOnly(0.0, pathCommand, condition, pathName, routine);
    }

    /**
     * Want to run with intake out ONLY, sepereate method to actually run the intake 
     * @param delaySeconds
     * @param pathCommand
     * @param condition
     * @param pathName
     * @param routine
     * @return
     */
    public Trigger traversePathWithIntakeOutOnly(double delaySeconds, FollowPathCommand pathCommand, Trigger condition, String pathName, AutoEvent routine) {
        condition
            .onTrue(Commands.waitSeconds(delaySeconds).andThen(pathCommand))
            .onTrue(mFlywheelsSS.setStateCmd(FlywheelStates.STANDBY_VELOCITY))
            .onTrue(mHoodSS.setStateCmd(HoodStates.MIN))
            .onTrue(mFuelPumpSS.setStateCmd(FuelPumpState.STOPPED))
            .onTrue(mIntake.setRollerStateCmd(IntakeRollerState.IDLE))
            .onTrue(mIntake.setRackStateCmd(IntakeRackState.INTAKE))
            .onTrue(mFuelInjectorSS.setStateCmd(FuelInjectorState.IDLE));

        return routine.loggedCondition(
            pathName+"/HasEnded", 
            pathCommand.hasEnded(), 
            true);
    }

    public Trigger traversePathWithIntakeInOnly(double delaySeconds, FollowPathCommand pathCommand, Trigger condition, String pathName, AutoEvent routine) {
        condition
            .onTrue(Commands.waitSeconds(delaySeconds).andThen(pathCommand))
            .onTrue(mFlywheelsSS.setStateCmd(FlywheelStates.STANDBY_VELOCITY))
            .onTrue(mHoodSS.setStateCmd(HoodStates.MIN))
            .onTrue(mFuelPumpSS.setStateCmd(FuelPumpState.STOPPED))
            .onTrue(mIntake.setRollerStateCmd(IntakeRollerState.IDLE))
            .onTrue(mIntake.setRackStateCmd(IntakeRackState.STOW))
            .onTrue(mFuelInjectorSS.setStateCmd(FuelInjectorState.IDLE));

        return routine.loggedCondition(
            pathName+"/HasEnded", 
            pathCommand.hasEnded(), 
            true);
    }

    public Trigger traversePathWithIntakeOutOnly(FollowPathCommand pathCommand, Trigger condition, String pathName, AutoEvent routine) {
        return traversePathWithIntakeOutOnly(0.0, pathCommand, condition, pathName, routine);
    }

    public Trigger followPathToAutoAlignShoot(Command autoAlignCommand, Trigger condition, String pathName, AutoEvent routine) {
        SequentialEndingCommandGroup autoAlignEndingCommand = new SequentialEndingCommandGroup(autoAlignCommand);

        condition
            .onTrue(autoAlignEndingCommand)
            .onTrue(mFlywheelsSS.setStateCmd(FlywheelStates.SHOTMAP_VELOCITY))
            .onTrue(mHoodSS.setStateCmd(HoodStates.SHOTMAP_POSITION))
            .onTrue(mFuelPumpSS.setStateCmd(FuelPumpState.INTAKE_VELOCITY));

        return routine.loggedCondition(
            pathName+"/InShootingTolerance", 
            () -> 
                mRobotDrive.getDriveManager().waitUntilAutoAlignFinishes().getAsBoolean()
                    &&
                mHoodSS.atGoal()
                    &&
                mFlywheelsSS.atLatestClosedLoopGoal()
                    &&
                mFuelPumpSS.atGoal()
                    &&
                !GameGoalPoseChooser.inCenter(mRobotDrive.getPoseEstimate())
                    &&
                mHoodSS.getHoodState().equals(HoodStates.SHOTMAP_POSITION)
                    &&
                mFlywheelsSS.getFlywheelState().equals(FlywheelStates.SHOTMAP_VELOCITY)
                    &&
                mRobotDrive.getDriveManager().getDriveState().equals(DriveState.AUTO_ALIGN)
                    &&
                autoAlignEndingCommand.isRunning(),
            true)
                .debounce(0.25, DebounceType.kRising)
                .debounce(5.0, DebounceType.kFalling);
    }

    @SuppressWarnings("unlikely-arg-type")
    public Trigger setUpShooterFromStationary(Trigger condition, AutoEvent routine){
        condition  
            .onTrue(mFlywheelsSS.setStateCmd(FlywheelStates.SHOTMAP_VELOCITY))
            .onTrue(mHoodSS.setStateCmd(HoodStates.SHOTMAP_POSITION));

        return routine.loggedCondition(
        "Stationary/InShootingTolerance", 
            () -> 
                mHoodSS.atGoal()
                    &&
                mFlywheelsSS.atLatestClosedLoopGoal()
                    && 
                mHoodSS.getCurrentGoal().equals(HoodStates.SHOTMAP_POSITION)
                    &&
                mFlywheelsSS.getFlywheelState().equals(FlywheelStates.SHOTMAP_VELOCITY), 
            true);
    }

    public void resetAllStates(Trigger condition) {
        condition
            .onTrue(mRobotDrive.getDriveManager().setDriveStateCommand(DriveState.TELEOP))
            .onTrue(mFlywheelsSS.setStateCmd(FlywheelStates.STANDBY_VELOCITY))
            .onTrue(mHoodSS.setStateCmd(HoodStates.MIN))
            .onTrue(mFuelPumpSS.setStateCmd(FuelPumpState.STOPPED))
            .onTrue(mFuelInjectorSS.setStateCmd(FuelInjectorState.IDLE))
            .onTrue(mIntake.setRackStateCmd(IntakeRackState.INTAKE))
            .onTrue(mIntake.setRollerStateCmd(IntakeRollerState.IDLE))
            .onTrue(mClimbSS.setStateCmd(ClimbState.IDLE));
    }

    public void resetAndEndAutos(Trigger condition, AutoEvent routine) {
        resetAllStates(condition);
        condition
            .onTrue(endAuto(routine));
    }

    public Trigger shootFuelToHub(double shotTime, Trigger condition, String pathName, AutoEvent routine) {
        SequentialEndingCommandGroup injectorShot = timedInjectorShot(shotTime, 0.02);
        SequentialEndingCommandGroup intakeShot = timedIntakeShot(shotTime, 0.02);

        condition
            .onTrue(injectorShot)
            .onTrue(intakeShot)
            .onTrue(mIntake.anshulCompact());

        return routine.loggedCondition(
            pathName+"/FuelToHubHasEnded", 
            () -> injectorShot.hasEnded() && intakeShot.hasEnded(), 
            true);
    }

    public Trigger runIntake(Trigger condition, String pathName, AutoEvent routine) {
        SequentialEndingCommandGroup intakeRollerCommandEnding = new SequentialEndingCommandGroup(mIntake.setRollerStateCmd(IntakeRollerState.INTAKE));

        condition
            .onTrue(intakeRollerCommandEnding)
            .onFalse(mIntake.setRollerStateCmd(IntakeRollerState.IDLE));

        return routine.loggedCondition(
            pathName + "/IntakingHasEnded", 
            () -> intakeRollerCommandEnding.hasEnded(), 
            true);
    }

    public Trigger goToClimb(Trigger condition, Supplier<Pose2d> pClimbPose, String name, AutoEvent auto) {
        SequentialEndingCommandGroup goToPreClimbPose = new SequentialEndingCommandGroup(
            mRobotDrive.getDriveManager().setToGenericAutoAlign(
                () -> pClimbPose.get().transformBy(
                    new Transform2d(
                        Math.signum(
                            pClimbPose.get().getX() - FieldConstants.kFieldXM / 2.0) 
                        * 0.05, 
                        0.00, 
                        Rotation2d.kZero)), 
                ConstraintType.LINEAR));

        Trigger atPreClimbPose = auto.loggedCondition(
            name+"/AtPreClimbPose", 
            () -> 
                goToPreClimbPose.isRunning()
                    &&
                mRobotDrive.getDriveManager().waitUntilAutoAlignFinishes().getAsBoolean(), 
            true);

        SequentialEndingCommandGroup prepareForClimb = 
            new SequentialEndingCommandGroup(mClimbSS.goUpTillClimbHeightThenStay());

        Trigger preparedForClimb = auto.loggedCondition(
            name+"/PreparedForClimb", 
            () -> prepareForClimb.hasEnded(), 
            true);

        SequentialEndingCommandGroup goToClimbPose = new SequentialEndingCommandGroup(
            mRobotDrive.getDriveManager().setToGenericAutoAlign(
                pClimbPose, 
                ConstraintType.LINEAR));

        Trigger atClimbPose = auto.loggedCondition(
            name+"/AtClimbPose", 
            () -> 
                goToClimbPose.isRunning()
                    &&
                mRobotDrive.getDriveManager().waitUntilAutoAlignFinishes().getAsBoolean(), 
            true);

        SequentialEndingCommandGroup climb = 
            new SequentialEndingCommandGroup(mClimbSS.goDownTillClimbedThenStayClimbed());

        Trigger hasClimbed = auto.loggedCondition(
            name+"/HasClimbed", 
            () -> climb.hasEnded(), 
            true);

        condition
            .onTrue(goToPreClimbPose)
            .onTrue(prepareForClimb);

        atPreClimbPose.and(preparedForClimb)
            .onTrue(goToClimbPose);

        atClimbPose
            .onTrue(climb);

        return hasClimbed;
    }

    /* Super structure commands */
    public SequentialEndingCommandGroup timedIndexShot(double timeout, double endTimeout) {
        return new SequentialEndingCommandGroup(
                mFuelPumpSS.setStateCmd(FuelPumpState.INTAKE_VOLT).withTimeout(timeout),
                mFuelPumpSS.setStateCmd(FuelPumpState.STOPPED).withTimeout(endTimeout));
    }

    public SequentialEndingCommandGroup timedIntakeShot(double timeout, double endTimeout) {
        return new SequentialEndingCommandGroup(
            mIntake.setRollerStateCmd(IntakeRollerState.INTAKE).withTimeout(timeout),
            mIntake.setRollerStateCmd(IntakeRollerState.IDLE).withTimeout(endTimeout));
    }

    public SequentialEndingCommandGroup timedInjectorShot(double timeout, double endTimeout) {
        return new SequentialEndingCommandGroup(
            mFuelInjectorSS.setStateCmd(FuelInjectorState.INTAKE).withTimeout(timeout),
            mFuelInjectorSS.setStateCmd(FuelInjectorState.IDLE).withTimeout(endTimeout));
    }

    ///////////////// DRIVE COMMANDS AND DATA \\\\\\\\\\\\\\\\\\\\\\
    public FollowPathCommand followChoreoPath(
            String pPathName, boolean pIsFirst, AutoEvent pAuto, boolean isMirrored) {
        return mRobotDrive.getDriveManager().followPathCommand(
            (!isMirrored) ? getTraj(pPathName).get() : getTraj(pPathName).get().mirrorPath(), 
            pIsFirst,
            pAuto);
    }

    public FollowPathCommand followChoreoPath(
            String pPathName, PPHolonomicDriveController pPID, boolean pIsFirst, AutoEvent pAuto, boolean isMirrored) {
        return mRobotDrive.getDriveManager().followPathCommand(
            (!isMirrored) ? getTraj(pPathName).get() : getTraj(pPathName).get().mirrorPath(),
            pPID, 
            pIsFirst,
            pAuto);
    }

    public Optional<PathPlannerPath> getTraj(String pPathName) {
        try {
            return Optional.of(PathPlannerPath.fromChoreoTrajectory(pPathName));
        } catch(Exception e) {
            e.printStackTrace();
            return Optional.empty();
        }
    }

    public void setPoseFromTrajectory(PathPlannerTrajectory idealTraj) {
        mRobotDrive.setPose(AllianceFlipUtil.apply(idealTraj.sample(0.0).pose));
    }

    ///////////////// PATH CHOOSING LOGIC \\\\\\\\\\\\\\\\\\\\\\
    public Supplier<Command> getAuto() {
        return getAutoChooser().get();
    }

    public LoggedDashboardChooser<Supplier<Command>> getAutoChooser() {
        return mAutoChooserLogged;
    }

    public final void tryToAddPathToChooser(String pPathName, Supplier<Command> pAuto) {
        tryToAddPathToChooser(pPathName, new Runnable() {
            @Override
            public void run() {
                mAutoChooser.addOption(pPathName, pAuto);
            }
        });
    }  
    
    /* Stops magic auton errors from occuring due to FMS or some BS I cook up */
    public void tryToAddPathToChooser(String pPathName, Runnable pPathAdding) {
        try {
            pPathAdding.run();
        } catch(Exception e) {
            mAutoChooser.addOption("Failed: "+pPathName, () -> backUpAuton());
            Telemetry.reportException(e);
        }
    }

    /* Subsystem Getters */
    public Drive getDriveSubsystem() {
        return mRobotDrive;
    }

    public FlywheelsSS getFlywheelSubsystem() {
        return mFlywheelsSS;
    }

    public HoodSS getHoodSubsystem() {
        return mHoodSS;
    }

    public FuelPumpSS getFuelPumpSubsystem() {
        return mFuelPumpSS;
    }

    public Intake getIntakeSubsystem() {
        return mIntake;
    }

    public ClimbSS getClimbSubsystem() {
        return mClimbSS;
    }

    public FuelInjectorSS getFuelInjector() {
        return mFuelInjectorSS;
    }

    public AutoFactory getAutoFactory() {
        return mAutoFactory;
    }

    private void loadCacheForAllPaths() {
        for(int i = 0; i < usedPathNames.length; i++) {
            mAutoFactory.trajectoryCmd(usedPathNames[i]);
            getTraj(usedPathNames[i]);
        }
    }
}