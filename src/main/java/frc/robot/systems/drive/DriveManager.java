// package frc.robot.systems.drive;

// import java.util.ArrayList;
// import java.util.List;
// import java.util.Optional;
// import java.util.function.BooleanSupplier;
// import java.util.function.DoubleSupplier;
// import java.util.function.Supplier;

// import org.littletonrobotics.junction.AutoLogOutput;

// // import com.pathplanner.lib.commands.FollowPathCommand;
// import frc.robot.commands.AutoEvent;
// import frc.robot.commands.FollowPathCommand;
// import com.pathplanner.lib.controllers.PPHolonomicDriveController;
// import com.pathplanner.lib.path.PathPlannerPath;

// import edu.wpi.first.math.Pair;
// import edu.wpi.first.math.filter.Debouncer;
// import edu.wpi.first.math.filter.Debouncer.DebounceType;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Transform2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.FunctionalCommand;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.lib.controls.TurnPointFeedforward;
// import frc.lib.math.AllianceFlipUtil;
// import frc.lib.math.EqualsUtil;
// import frc.lib.telemetry.Telemetry;
// import frc.robot.systems.drive.controllers.HeadingController;
// import frc.robot.systems.drive.controllers.HolonomicController;
// import frc.robot.systems.drive.controllers.HolonomicController.ConstraintType;
// import frc.robot.systems.drive.controllers.LineController;
// import frc.robot.systems.drive.controllers.ManualTeleopController;
// import frc.robot.systems.drive.controllers.ManualTeleopController.DriverProfiles;
// import frc.robot.game.GameDriveManager;
// import frc.robot.game.GameDriveManager.GameDriveStates;
// import frc.robot.logging.DriveErrors;

// public class DriveManager {
//     public static enum DriveState {
//         // TELEOP AND AUTON CONTROLS
//         TELEOP,
//         TELEOP_SNIPER,
//         POV_SNIPER,
//         HEADING_ALIGN,
//         HEADING_X_LOCK,
//         AUTO_ALIGN,
//         LINE_ALIGN,
//         AUTON,
//         AUTON_HEADING_ALIGN,
//         STOP,

//         // TUNING
//         DRIFT_TEST,
//         LINEAR_TEST,
//         SYSID_CHARACTERIZATION,
//         WHEEL_CHARACTERIZATION
//     }

//     private Drive mDrive;

//     @AutoLogOutput(key = "Drive/State")
//     private DriveState mDriveState = DriveState.TELEOP;

//     private final ManualTeleopController mTeleopController = new ManualTeleopController();

//     private final HeadingController mHeadingController = new HeadingController(TurnPointFeedforward.zeroTurnPointFF());

//     private Supplier<Rotation2d> mGoalRotationSup = () -> new Rotation2d();

//     private final HolonomicController mAutoAlignController = new HolonomicController();
//     private final LineController mLineAlignController = new LineController(
//         () -> 0.0, 
//         () -> 1.0, 
//         () -> false);

//     // @AutoLogOutput(key="Drive/GoalPoseSup")
//     private Supplier<Pose2d> mGoalPoseSup = () -> new Pose2d();
//     private Supplier<ChassisSpeeds> mChassisSpeedSup = () -> new ChassisSpeeds();
//     private final Debouncer mAutoAlignTimeout = new Debouncer(0.1, DebounceType.kRising);

//     private ChassisSpeeds mPPDesiredSpeeds = new ChassisSpeeds();

//     private GameDriveManager mGameDriveManager;

//     public DriveManager(Drive pDrive) {
//         mDrive = pDrive;
//         mGameDriveManager = new GameDriveManager(mDrive);
//         mHeadingController.setHeadingGoal(mGoalRotationSup);
//     }

//     public Optional<ChassisSpeeds> computeDesiredSpeedsFromState() {
//         mHeadingController.updateController();
//         mAutoAlignController.updateControllers();
//         mLineAlignController.updateControllers();

//         ChassisSpeeds teleopSpeeds = mTeleopController.computeChassisSpeeds(
//             mDrive.getPoseEstimate().getRotation(), 
//             false, 
//             true);
//         Optional<ChassisSpeeds> desiredSpeeds = of(teleopSpeeds);
//         switch (mDriveState) {
//             case TELEOP:
//                 break;
//             case TELEOP_SNIPER:
//                 desiredSpeeds = of(mTeleopController.computeChassisSpeeds(
//                    mDrive.getPoseEstimate().getRotation(), 
//                    true, 
//                    true));
//                 break;
//             case POV_SNIPER:
//                 desiredSpeeds = of(mTeleopController.computeSniperPOVChassisSpeeds(
//                    mDrive.getPoseEstimate().getRotation(), 
//                    false));
//                 break;
//             case HEADING_ALIGN:
//                 desiredSpeeds = of(new ChassisSpeeds(
//                     teleopSpeeds.vxMetersPerSecond, 
//                     teleopSpeeds.vyMetersPerSecond,
//                     mHeadingController.getSnapOutputRadians(mDrive.getPoseEstimate().getRotation())));
//                 break;
//             case HEADING_X_LOCK:
//                 desiredSpeeds = Optional.empty();
//                 SwerveHelper.runXLock(
//                     kTrackWidthXMeters, 
//                     kTrackWidthYMeters, 
//                     mDrive.getModules());
//                 break;
//             case AUTO_ALIGN:
//                 desiredSpeeds = of(mAutoAlignController.calculate(
//                     mGoalPoseSup.get(), 
//                     mChassisSpeedSup.get(),
//                     mDrive.getPoseEstimate()));
//                 break;
//             case LINE_ALIGN:
//                 desiredSpeeds = of(mLineAlignController.calculate(
//                     teleopSpeeds, 
//                     mGoalPoseSup.get(), 
//                     mDrive.getPoseEstimate()));
//                 break;
//             case AUTON:
//                 desiredSpeeds = of(mPPDesiredSpeeds);
//                 break;
//             case AUTON_HEADING_ALIGN:
//                 desiredSpeeds = of(new ChassisSpeeds(
//                     mPPDesiredSpeeds.vxMetersPerSecond, 
//                     mPPDesiredSpeeds.vyMetersPerSecond,
//                     mHeadingController.getSnapOutputRadians(mDrive.getPoseEstimate().getRotation())));
//                 break;
//             case DRIFT_TEST:
//                 desiredSpeeds = of(ChassisSpeeds.fromFieldRelativeSpeeds(
//                     new ChassisSpeeds(
//                         Drive.tLinearTestSpeedMPS.get(), 
//                         0.0, 
//                         Math.toRadians(
//                             Drive.tRotationDriftTestSpeedDeg.get())),
//                     mDrive.getPoseEstimate().getRotation()));
//                 break;
//             case LINEAR_TEST:
//                 desiredSpeeds = of(ChassisSpeeds.fromFieldRelativeSpeeds(
//                     new ChassisSpeeds(
//                         Drive.tLinearTestSpeedMPS.get(),
//                         0.0,
//                         0.0), 
//                     mDrive.getPoseEstimate().getRotation()));
//                 break;
//                 /* Set by characterization commands in the CHARACTERIZATION header. Wheel characterization is currently unimplemented */
//             case SYSID_CHARACTERIZATION:
//             case WHEEL_CHARACTERIZATION:
//                 /* If null, then PID isn't set, so characterization can set motors w/o interruption */
//                 desiredSpeeds = Optional.empty();
//                 break;
//             case STOP:
//                 desiredSpeeds = of(new ChassisSpeeds());
//                 break;
//             default:
//                 /* Defaults to Teleop control if no other cases are run*/
//         }

//         return desiredSpeeds;
//     }

//     public Optional<ChassisSpeeds> of(ChassisSpeeds speeds) {
//         return Optional.of(speeds);
//     } 

//     ///////////////////////// STATE COMMANDS \\\\\\\\\\\\\\\\\\\\\\\\
//     /* Sets drive state  and handles FF model initial conditions */
//     public void setDriveState(DriveState state) {
//         mDriveState = state;
//         switch (mDriveState) {
//             case AUTON:
//                 mDrive.setFFModel(true, false);
//                 break;
//             case AUTO_ALIGN, STOP:
//                 mDrive.setFFModel(false, false);
//                 break;
//             default:
//                 mDrive.setFFModel(false, true);
//         }
//     } 

//     public Command setDriveStateCommand(DriveState state) {
//         return Commands.runOnce(() -> setDriveState(state), mDrive);
//     }

//     /* Set's state initially, and doesn't end till interruped by another drive command */
//     public Command setDriveStateCommandContinued(DriveState state) {
//         return new FunctionalCommand(
//             () -> setDriveState(state), 
//             () -> {}, (interrupted) -> {}, 
//             () -> false, 
//             mDrive
//         );
//     }
    
//     /*
//      * REGULAR DRIVER CONTROL 
//      */
//     public Command setToTeleop() {
//         return setDriveStateCommandContinued(DriveState.TELEOP);
//     }

//     /*
//      * SLOWED DRIVER CONTROL 
//      */
//     public Command setToTeleopSniper() {
//         return setDriveStateCommandContinued(DriveState.TELEOP_SNIPER);
//     }

//     /*
//      * DRIVER CONTROL USING XBOX POV BUTTONS
//      */
//     public Command setToPOVSniper() {
//         return setDriveStateCommandContinued(DriveState.POV_SNIPER);
//     }

//     /*
//      * STOPS DRIVE
//      */
//     public Command setToStop() {
//         return setDriveStateCommand(DriveState.STOP);
//     }

//     public Command setToHeadingXLock() {
//         return setDriveStateCommandContinued(DriveState.HEADING_X_LOCK);
//     }

//     /*
//      * TESTS ROTATION WHILE TRANSLATION
//      */
//     public Command setToDriftTest() {
//         return setDriveStateCommandContinued(DriveState.DRIFT_TEST);
//     }

//     /*
//      * TESTS 0 TO X SPEED DISTANCE
//      */
//     public Command setToLinearTest() {
//         return setDriveStateCommandContinued(DriveState.LINEAR_TEST);
//     }

//     /*
//      * SYS ID
//      */
//     public Command setToSysIDCharacterization() {
//         return setDriveStateCommandContinued(DriveState.SYSID_CHARACTERIZATION);
//     }

//     /*
//      * WHEEL ODOM CHARACTERIZATION
//      */
//     public Command setToWheelCharacterization() {
//         return setDriveStateCommandContinued(DriveState.WHEEL_CHARACTERIZATION);
//     }

//     /*
//      * FOLLOWS PATHPLANNER PATH WITH DEFAULT PID
//      */
//     public FollowPathCommand followPathCommand(PathPlannerPath path, boolean isFirst) {
//         return followPathCommand(
//             path, 
//             new PPHolonomicDriveController(
//                 kPPTranslationPID, 
//                 kPPRotationPID), 
//             isFirst);
//     }

//     /*
//      * FOLLOWS PATHPLANNER PATH WITH CUSTOM PID
//      */
//     public FollowPathCommand followPathCommand(
//         PathPlannerPath path, PPHolonomicDriveController drivePID, boolean isFirst) {
//         return new FollowPathCommand(
//             path, mDrive::getPoseEstimate, 
//             mDrive::getRobotChassisSpeeds,
//             (speeds, ff) -> {
//                 setDriveState(DriveState.AUTON);
//                 mPPDesiredSpeeds = speeds;
//                 mDrive.setDriveFeedforwards(ff);
//             }, 
//             drivePID, 
//             mDrive.getPPRobotConfig(), 
//             () -> AllianceFlipUtil.shouldFlip(), 
//             isFirst,
//             mDrive::setPose,
//             mDrive);
//     }

//     /*
//      * FOLLOWS PATHPLANNER PATH WITH DEFAULT PID
//      */
//     public FollowPathCommand followPathCommand(PathPlannerPath path, boolean isFirst, AutoEvent pAuto) {
//         return followPathCommand(
//             path, 
//             new PPHolonomicDriveController(
//                 kPPTranslationPID, 
//                 kPPRotationPID), 
//             isFirst,
//             pAuto);
//     }

//     /*
//      * FOLLOWS PATHPLANNER PATH WITH CUSTOM PID
//      */
//     public FollowPathCommand followPathCommand(
//         PathPlannerPath path, PPHolonomicDriveController drivePID, boolean isFirst, AutoEvent pAuto) {
//         return new FollowPathCommand(
//             path, mDrive::getPoseEstimate, 
//             mDrive::getRobotChassisSpeeds,
//             (speeds, ff) -> {
//                 setDriveState(DriveState.AUTON);
//                 mPPDesiredSpeeds = speeds;
//                 mDrive.setDriveFeedforwards(ff);
//             }, 
//             drivePID, 
//             mDrive.getPPRobotConfig(), 
//             () -> AllianceFlipUtil.shouldFlip(), 
//             isFirst,
//             mDrive::setPose,
//             pAuto,
//             mDrive);
//     }

//     public void setToAuton() {
//         setDriveState(DriveState.AUTON);
//     }

//     /*
//      * GAME SPECIFIC SETPOINTS FOR HEADING, LINE AND AUTO CONTROLLER
//      */
//     public Command getGameDriveCommand(GameDriveStates pGameDriveStates) {
//         return mGameDriveManager.getSetGameDriveStateCmd(pGameDriveStates);
//     }

//     /*
//      * Reference GameDriveManager to use game-specific implementation of mDrive command
//      * @param Goal strategy, based on where you're aligning
//      * @param Constraint type, linear or on an axis
//      */
//     public Command setToGenericAutoAlign(Supplier<Pose2d> pGoalPoseSup, ConstraintType pConstraintType) {
//         return new InstantCommand(() -> {
//             mGoalPoseSup = pGoalPoseSup;
//             mAutoAlignController.setConstraintType(pConstraintType);
//             mAutoAlignController.reset(
//                 mDrive.getPoseEstimate(), 
//                 ChassisSpeeds.fromRobotRelativeSpeeds(
//                     mDrive.getRobotChassisSpeeds(), 
//                     mDrive.getPoseEstimate().getRotation()),
//                 mGoalPoseSup.get());
//             }).andThen( setDriveStateCommandContinued( DriveState.AUTO_ALIGN ) );
//     }

//     /*
//      * Reference GameDriveManager to use game-specific implementation of mDrive command
//      * Resets
//      * @param Goal strategy, based on where you're aligning
//      * @param Constraint type, linear or on an axis
//      */
//     public Command setToGenericAutoAlignWithGeneratorReset(Supplier<Pose2d> pGoalPoseSup, ConstraintType pConstraintType) {
//         return new InstantCommand(() -> {
//             mGoalPoseSup = pGoalPoseSup;
//             mChassisSpeedSup = () -> new ChassisSpeeds();
//             mAutoAlignController.setConstraintType(pConstraintType);
//             mAutoAlignController.reset(
//                 mDrive.getPoseEstimate(), 
//                 ChassisSpeeds.fromRobotRelativeSpeeds(
//                     mDrive.getRobotChassisSpeeds(), 
//                     mDrive.getPoseEstimate().getRotation()),
//                 mGoalPoseSup.get());
//             mDrive.resetSetpointGenerator();
//             }).andThen( setDriveStateCommandContinued( DriveState.AUTO_ALIGN ) );
//     }

//     public Command setToGenericAutoAlignWithGeneratorReset(Supplier<Pose2d> pGoalPoseSup, Supplier<ChassisSpeeds> speedSup, ConstraintType pConstraintType) {
//         return new InstantCommand(() -> {
//             mGoalPoseSup = pGoalPoseSup;
//             mChassisSpeedSup = speedSup;
//             mAutoAlignController.setConstraintType(pConstraintType);
//             mAutoAlignController.reset(
//                 mDrive.getPoseEstimate(), 
//                 ChassisSpeeds.fromRobotRelativeSpeeds(
//                     mDrive.getRobotChassisSpeeds(), 
//                     mDrive.getPoseEstimate().getRotation()),
//                 mGoalPoseSup.get());
//             mDrive.resetSetpointGenerator();
//             }).andThen( setDriveStateCommandContinued( DriveState.AUTO_ALIGN ) );
//     }

//     public Command setToGenericLineAlign(Supplier<Pose2d> pGoalPoseSup, Supplier<Rotation2d> pLineAngle, DoubleSupplier pTelScal, BooleanSupplier pTeleopInvert) {
//         return new InstantCommand(() -> {
//             mGoalPoseSup = pGoalPoseSup;
//             mLineAlignController.setControllerGoalSettings(
//                 pTelScal, 
//                 () -> pLineAngle.get().getTan(), 
//                 pTeleopInvert);
//             mLineAlignController.reset(
//                 mDrive.getPoseEstimate(), 
//                 mGoalPoseSup.get());
//         }).andThen( setDriveStateCommandContinued( DriveState.LINE_ALIGN ) );
//     }

//     /*
//      * Reference GameDriveManager to use game-specific implementation of mDrive command
//      * @param The desired rotation
//      * @param Turn feedforward
//      */
//     public Command setToGenericHeadingAlign(Supplier<Rotation2d> pGoalRotation, TurnPointFeedforward pTurnPointFeedforward) {
//         return setToGenericHeadingAlign( pGoalRotation, pTurnPointFeedforward, DriveState.HEADING_ALIGN );
//     }

//     /* Accounts for velocity of drive when turning */
//     public Command setToGenericHeadingAlign(Supplier<Rotation2d> pGoalRotation, Supplier<Pose2d> pGoalPoseSupplier) {
//         return Commands.runOnce(() -> mGoalPoseSup = pGoalPoseSupplier )
//             .andThen( setToGenericHeadingAlign( pGoalRotation, getDefaultTurnPointFF() ));
//     }

//     /*
//      * Reference GameDriveManager to use game-specific implementation of mDrive command
//      * @param The desired rotation
//      * @param Turn feedforward
//      */
//     public Command setToGenericHeadingAlignAuton(Supplier<Rotation2d> pGoalRotation, TurnPointFeedforward pTurnPointFeedforward) {
//         return setToGenericHeadingAlign( pGoalRotation, pTurnPointFeedforward, DriveState.AUTON_HEADING_ALIGN );
//     }

//     /* Accounts for velocity of drive when turning */
//     public Command setToGenericHeadingAlignAuton(Supplier<Rotation2d> pGoalRotation, Supplier<Pose2d> pGoalPoseSupplier) {
//         return Commands.runOnce(() -> mGoalPoseSup = pGoalPoseSupplier ).andThen(setToGenericHeadingAlignAuton( pGoalRotation, getDefaultTurnPointFF() ));
//     }

//     public Command setToGenericHeadingAlign(Supplier<Rotation2d> pGoalRotation, TurnPointFeedforward pTurnPointFeedforward, DriveState headingState) {
//         return new InstantCommand(() -> {
//             if(!validHeadingState(headingState)) Telemetry.reportIssue(new DriveErrors.WrongHeadingState());
//             mGoalRotationSup = pGoalRotation;
//             mHeadingController.setHeadingGoal(mGoalRotationSup);
//             mHeadingController.reset(
//                 mDrive.getPoseEstimate().getRotation(), 
//                 mDrive.getRobotRotationVelocity());
//             mHeadingController.setTurnPointFF(pTurnPointFeedforward);
//         }).andThen( setDriveStateCommandContinued( headingState ) );
//     }

//     public TurnPointFeedforward getDefaultTurnPointFF() {
//         return new TurnPointFeedforward(
//             () -> mDrive.getPoseEstimate(), 
//             () -> mDrive.getDesiredChassisSpeeds(), 
//             mGoalPoseSup, 
//             () -> new ChassisSpeeds());
//     }

//     ///////////// SETTERS \\\\\\\\\\\\\
//     public void acceptJoystickInputs(
//             DoubleSupplier pXSupplier, DoubleSupplier pYSupplier,
//             DoubleSupplier pThetaSupplier, Supplier<Rotation2d> pPOVSupplier) {
//         mTeleopController.acceptJoystickInputs(pXSupplier, pYSupplier, pThetaSupplier, pPOVSupplier);
//     }

//     public Command setDriveProfile(DriverProfiles profile) {
//         return new InstantCommand(() -> mTeleopController.updateTuneablesWithProfiles(profile));
//     }

//     public void setPPDesiredSpeeds(ChassisSpeeds speeds) {
//         mPPDesiredSpeeds = speeds;
//     }

//     ///////////// GETTERS \\\\\\\\\\\\\
//     public BooleanSupplier waitUntilHeadingAlignFinishes() {
//         return () -> inHeadingTolerance();
//     }

//     @AutoLogOutput(key = "Drive/Tolerance/HeadingController")
//     public boolean inHeadingTolerance() {
//         /* Accounts for angle wrapping issues with rotation 2D error */
//         return mHeadingController.inTolerance(mDrive.getRobotRotation(), Rotation2d.fromDegrees(5.0));
//     }

//     public boolean validHeadingState(DriveState state) {
//         return state.equals(DriveState.AUTON_HEADING_ALIGN) || state.equals(DriveState.HEADING_ALIGN);
//     }

//     public BooleanSupplier waitUntilAutoAlignFinishes() {
//         return () -> mAutoAlignTimeout.calculate(mAutoAlignController.atGoal());
//     }

//     @AutoLogOutput(key = "Drive/OdometryPP/DesiredChassisSpeeds")
//     public ChassisSpeeds getPPDesiredChassisSpeeds() {
//         return mPPDesiredSpeeds;
//     }

//     public ManualTeleopController getTeleopController() {
//         return mTeleopController;
//     }

//     public HolonomicController getAutoAlignController() {
//         return mAutoAlignController;
//     }

//     public HeadingController getHeadingController() {
//         return mHeadingController;
//     }

//     public LineController getLineAlignController() {
//         return mLineAlignController;
//     }

//     public DriveState getDriveState() {
//         return mDriveState;
//     }

//     /**
//      * Generates a command to pathfind around the bump obstacle,
//      * inserting corner waypoints wherever the straight-line path
//      * would intersect the bounding box.
//      *
//      * @param goalPose The target pose to navigate to.
//      * @return A SequentialCommandGroup driving through safe waypoints.
//      */
//     public Command pathFindAroundBump(Pose2d goalPose) {
//         // --- Obstacle bounding box (alliance-flipped) ---
//         double boxX1 = AllianceFlipUtil.applyX(3.481);
//         double boxX2 = AllianceFlipUtil.applyX(5.679);
//         double boxY1 = AllianceFlipUtil.applyY(0.659);
//         double boxY2 = AllianceFlipUtil.applyY(7.451);

//         double boxMinX = Math.min(boxX1, boxX2);
//         double boxMaxX = Math.max(boxX1, boxX2);
//         double boxMinY = Math.min(boxY1, boxY2);
//         double boxMaxY = Math.max(boxY1, boxY2);

//         Pose2d robotPose = mDrive.getPoseEstimate();
//         double robotX    = robotPose.getX();
//         double robotY    = robotPose.getY();
//         double goalX     = goalPose.getX();
//         double goalY     = goalPose.getY();

//         double dx = goalX - robotX;
//         double dy = goalY - robotY;

//         // --- Determine traversal direction for per-leg speed selection ---
//         TraversalDirection direction = TraversalDirection.from(dx);

//         // --- Compute where the direct path crosses each vertical box edge ---
//         List<Pair<Pose2d, ChassisSpeeds>> cornerWaypoints = new ArrayList<>();

//         for (double edgeX : getOrderedEdges(robotX, boxMinX, boxMaxX)) {
//             if (Math.abs(dx) < 1e-9) break; // Vertical path — no X-edge crossings

//             double t = (edgeX - robotX) / dx;
//             if (t <= 0.0 || t >= 1.0) continue; // Crossing behind robot or past goal

//             double yAtEdge = robotY + t * dy;
//             if (EqualsUtil.inBetween(yAtEdge, boxMinY, boxMaxY)) {
//                 double cornerY = EqualsUtil.getClosest(robotY, boxMinY, boxMaxY);
//                 Pose2d corner  = new Pose2d(edgeX, cornerY, Rotation2d.k180deg);

//                 // Speed is based on which edge we're crossing
//                 ChassisSpeeds speeds = direction.speedsForEdge(edgeX, boxMinX, boxMaxX);
//                 cornerWaypoints.add(Pair.of(corner, speeds));
//             }
//         }

//         // --- Build sequential command ---
//         SequentialCommandGroup command = new SequentialCommandGroup();
//         Transform2d tolerance = new Transform2d(0.1, 0.1, Rotation2d.fromDegrees(5.0));

//         for (Pair<Pose2d, ChassisSpeeds> waypoint : cornerWaypoints) {
//             command.addCommands(
//                 setToGenericAutoAlignWithGeneratorReset(
//                     () -> waypoint.getFirst(),
//                     () -> waypoint.getSecond(),
//                     ConstraintType.LINEAR
//                 ).onlyWhile(() -> !mAutoAlignController.inTolerance(tolerance, mDrive.getPoseEstimate()))
//             );
//         }

//         command.addCommands(setToGenericAutoAlign(() -> goalPose, ConstraintType.LINEAR));
//         return command;
//     }

//     /**
//      * Represents the direction the robot is traversing the bounding box,
//      * and owns the speed profiles for each edge crossing in that direction.
//      *
//      * APPROACHING : robot is moving in the +X direction (toward higher X)
//      *   - Entry edge (minX): accelerate into the box corner
//      *   - Exit edge  (maxX): decelerate out of the box corner
//      *
//      * RETREATING  : robot is moving in the -X direction (toward lower X)
//      *   - Entry edge (maxX): accelerate into the box corner
//      *   - Exit edge  (minX): decelerate out of the box corner
//      */
//     private enum TraversalDirection {
//         APPROACHING {
//             @Override
//             public ChassisSpeeds speedsForEdge(double edgeX, double boxMinX, double boxMaxX) {
//                 return (edgeX == boxMinX)
//                     ? new ChassisSpeeds( 2.0, 0.0, 0.0)   // Entry — accelerate
//                     : new ChassisSpeeds( 1.0, 0.0, 0.0);  // Exit  — slow down
//             }
//         },
//         RETREATING {
//             @Override
//             public ChassisSpeeds speedsForEdge(double edgeX, double boxMinX, double boxMaxX) {
//                 return (edgeX == boxMaxX)
//                     ? new ChassisSpeeds(-2.0, 0.0, 0.0)   // Entry — accelerate (negative X)
//                     : new ChassisSpeeds(-1.0, 0.0, 0.0);  // Exit  — slow down  (negative X)
//             }
//         };

//         public abstract ChassisSpeeds speedsForEdge(double edgeX, double boxMinX, double boxMaxX);

//         public static TraversalDirection from(double dx) {
//             return (dx >= 0) ? APPROACHING : RETREATING;
//         }
//     }

//     /**
//      * Returns the box's vertical edges ordered by proximity to the robot
//      * so waypoints are inserted in traversal order.
//      */
//     private double[] getOrderedEdges(double robotX, double boxMinX, double boxMaxX) {
//         if (robotX <= boxMinX) return new double[]{ boxMinX, boxMaxX };
//         if (robotX >= boxMaxX) return new double[]{ boxMaxX, boxMinX };
//         return new double[]{ boxMinX, boxMaxX }; // Robot is inside — check both
//     }
    
// }