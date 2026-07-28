// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.systems.drive;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.controls.TurnPointFeedforward;
import frc.lib.math.AllianceFlipUtil;
import frc.lib.pathplanner.AzimuthFeedForward;
import frc.lib.pathplanner.SwerveSetpoint;
import frc.lib.telemetry.Telemetry;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.Mode;
import frc.robot.logging.DriveErrors;
import frc.robot.systems.drive.controllers.HeadingController;
import frc.robot.systems.drive.controllers.HolonomicController;
import frc.robot.systems.drive.controllers.HolonomicController.ConstraintType;
import frc.robot.systems.drive.controllers.LineController;
import frc.robot.systems.drive.controllers.ManualTeleopController;
import frc.robot.systems.drive.controllers.ManualTeleopController.DriverProfiles;
import frc.robot.util.LocalADStarAK;


import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
    public static enum DriveState {
        // TELEOP AND AUTON CONTROLS
        TELEOP,
        TELEOP_SNIPER,
        POV_SNIPER,
        HEADING_ALIGN,
        HEADING_X_LOCK,
        AUTO_ALIGN,
        LINE_ALIGN,
        AUTON,
        AUTON_HEADING_ALIGN,
        STOP,

        // TUNING
        DRIFT_TEST,
        LINEAR_TEST,
        SYSID_CHARACTERIZATION,
        WHEEL_CHARACTERIZATION
    }

    @AutoLogOutput(key = "Drive/State")
    private DriveState mDriveState = DriveState.TELEOP;

    private ChassisSpeeds desiredSpeeds;
    private ChassisSpeeds mPPDesiredSpeeds = new ChassisSpeeds();
    private Rotation2d mRobotRotation;

    private final ManualTeleopController mTeleopController = new ManualTeleopController();
    private final HeadingController mHeadingController = new HeadingController(TurnPointFeedforward.zeroTurnPointFF());
    private Supplier<Rotation2d> mGoalRotationSup = () -> new Rotation2d();
    private final HolonomicController mAutoAlignController = new HolonomicController();
    private final LineController mLineAlignController = new LineController(
        () -> 0.0, 
        () -> 1.0, 
        () -> false);

    private Supplier<Pose2d> mGoalPoseSup = () -> new Pose2d();
    private Supplier<ChassisSpeeds> mChassisSpeedSup = () -> new ChassisSpeeds();
    private final Debouncer mAutoAlignTimeout = new Debouncer(0.1, DebounceType.kRising);

    
    // TunerConstants doesn't include these constants, so they are declared locally
    static final double ODOMETRY_FREQUENCY = TunerConstants.kCANBus.isNetworkFD() ? 250.0 : 100.0;
    public static final double DRIVE_BASE_RADIUS =
        Math.max(
            Math.max(
                Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
                Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
            Math.max(
                Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
                Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

    // PathPlanner config constants
    private static final double ROBOT_MASS_KG = 74.088;
    private static final double ROBOT_MOI = 6.883;
    private static final double WHEEL_COF = 1.2;
    private static final RobotConfig PP_CONFIG =
        new RobotConfig(
            ROBOT_MASS_KG,
            ROBOT_MOI,
            new ModuleConfig(
                TunerConstants.FrontLeft.WheelRadius,
                TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
                WHEEL_COF,
                DCMotor.getKrakenX60Foc(1)
                    .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
                TunerConstants.FrontLeft.SlipCurrent,
                1),
            getModuleTranslations());

    static final Lock odometryLock = new ReentrantLock();
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Module[] modules = new Module[4]; // FL, FR, BL, BR
    private final SysIdRoutine sysId;
    private final Alert gyroDisconnectedAlert =
        new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
    private Rotation2d rawGyroRotation = Rotation2d.kZero;
    private SwerveModulePosition[] lastModulePositions = // For delta tracking
        new SwerveModulePosition[] {
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition()
        };
    private SwerveDrivePoseEstimator poseEstimator;

    public Drive(
        GyroIO gyroIO,
        ModuleIO flModuleIO,
        ModuleIO frModuleIO,
        ModuleIO blModuleIO,
        ModuleIO brModuleIO) {
      this.gyroIO = gyroIO;
      modules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft);
      modules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight);
      modules[2] = new Module(blModuleIO, 2, TunerConstants.BackLeft);
      modules[3] = new Module(brModuleIO, 3, TunerConstants.BackRight);
      mRobotRotation = gyroInputs.yawPosition;
      poseEstimator = new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, Pose2d.kZero);


      // Usage reporting for swerve template
      HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

      // Start odometry thread
      PhoenixOdometryThread.getInstance().start();

      // Configure AutoBuilder for PathPlanner
      AutoBuilder.configure(
          this::getPose,
          this::setPose,
          this::getChassisSpeeds,
          this::runVelocity,
          new PPHolonomicDriveController(
              new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
          PP_CONFIG,
          () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
          this);
      Pathfinding.setPathfinder(new LocalADStarAK());
      PathPlannerLogging.setLogActivePathCallback(
          (activePath) -> {
            Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[0]));
          });
      PathPlannerLogging.setLogTargetPoseCallback(
          (targetPose) -> {
            Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
          });

      // Configure SysId
      sysId =
          new SysIdRoutine(
              new SysIdRoutine.Config(
                  null,
                  null,
                  null,
                  (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
              new SysIdRoutine.Mechanism(
                  (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
    }

    @Override
    public void periodic() {
      odometryLock.lock(); // Prevents odometry updates while reading data
      gyroIO.updateInputs(gyroInputs);
      Logger.processInputs("Drive/Gyro", gyroInputs);
      for (var module : modules) {
        module.periodic();
      }
      odometryLock.unlock();

      // Stop moving when disabled
      if (DriverStation.isDisabled()) {
        for (var module : modules) {
          module.stop();
        }
      }

      // Log empty setpoint states when disabled
      if (DriverStation.isDisabled()) {
        Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
        Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
      }

      // Update odometry
      double[] sampleTimestamps =
          modules[0].getOdometryTimestamps(); // All signals are sampled together
      int sampleCount = sampleTimestamps.length;
      for (int i = 0; i < sampleCount; i++) {
        // Read wheel positions and deltas from each module
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
        SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
        for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
          modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
          moduleDeltas[moduleIndex] =
              new SwerveModulePosition(
                  modulePositions[moduleIndex].distanceMeters
                      - lastModulePositions[moduleIndex].distanceMeters,
                  modulePositions[moduleIndex].angle);
          lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
        }

        // Update gyro angle
        if (gyroInputs.connected) {
          // Use the real gyro angle
          rawGyroRotation = gyroInputs.odometryYawPositions[i];
        } else {
          // Use the angle delta from the kinematics and module deltas
          Twist2d twist = kinematics.toTwist2d(moduleDeltas);
          rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
        }

        // Apply update
        poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
      }

      // Update gyro alert
      gyroDisconnectedAlert.set(!gyroInputs.connected && RobotConstants.kCurrentMode != Mode.SIM);

      
    }

    public void setDriveState(DriveState pState) {
      mDriveState = pState;
      // SETTING DESIRED SPEEDS FROM DRIVE STATE
      switch (mDriveState) {
            case TELEOP:
                break;
            case TELEOP_SNIPER:
                desiredSpeeds = mTeleopController.computeChassisSpeeds(
                   getPose().getRotation(), 
                   true, 
                   true);
                break;
            case POV_SNIPER:
                desiredSpeeds = mTeleopController.computeSniperPOVChassisSpeeds(
                   getPose().getRotation(), 
                   false);
                break;
            case HEADING_ALIGN:
                desiredSpeeds = new ChassisSpeeds(
                    desiredSpeeds.vxMetersPerSecond, 
                    desiredSpeeds.vyMetersPerSecond,
                    mHeadingController.getSnapOutputRadians(getPose().getRotation()));
                break;
            case HEADING_X_LOCK:
                desiredSpeeds = new ChassisSpeeds();
                runXLock(
                    DriveConstants.kTrackWidthXMeters, 
                    DriveConstants.kTrackWidthYMeters, 
                    getModules());
                break;
            case AUTO_ALIGN:
                desiredSpeeds = mAutoAlignController.calculate(
                    mGoalPoseSup.get(), 
                    mChassisSpeedSup.get(),
                    getPose());
                break;
            case LINE_ALIGN:
                desiredSpeeds = mLineAlignController.calculate(
                    desiredSpeeds, 
                    mGoalPoseSup.get(), 
                    getPose());
                break;
            case AUTON:
                desiredSpeeds = mPPDesiredSpeeds;
                break;
            case AUTON_HEADING_ALIGN:
                desiredSpeeds = new ChassisSpeeds(
                    mPPDesiredSpeeds.vxMetersPerSecond, 
                    mPPDesiredSpeeds.vyMetersPerSecond,
                    mHeadingController.getSnapOutputRadians(getPose().getRotation()));
                break;
            // case DRIFT_TEST:
            //     desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            //         new ChassisSpeeds(
            //             Drive.tLinearTestSpeedMPS.get(), 
            //             0.0, 
            //             Math.toRadians(
            //                 Drive.tRotationDriftTestSpeedDeg.get())),
            //         getPose().getRotation());
            //     break;
            // case LINEAR_TEST:
            //     desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            //         new ChassisSpeeds(
            //             Drive.tLinearTestSpeedMPS.get(),
            //             0.0,
            //             0.0), 
            //         getPose().getRotation());
            //     break;
                /* Set by characterization commands in the CHARACTERIZATION header. Wheel characterization is currently unimplemented */
            case SYSID_CHARACTERIZATION:
            case WHEEL_CHARACTERIZATION:
                /* If null, then PID isn't set, so characterization can set motors w/o interruption */
                desiredSpeeds = new ChassisSpeeds();
                break;
            case STOP:
                desiredSpeeds = new ChassisSpeeds();
                break;
            default:
                /* Defaults to Teleop control if no other cases are run*/
      }
    }

    /*
     * REGULAR DRIVER CONTROL 
     */
    public Command setToTeleop() {
        return new InstantCommand(() -> setDriveState(DriveState.TELEOP));
    }

    /*
     * SLOWED DRIVER CONTROL 
     */
    public Command setToTeleopSniper() {
        return new InstantCommand(() -> setDriveState(DriveState.TELEOP_SNIPER));
    }

    /*
     * DRIVER CONTROL USING XBOX POV BUTTONS
     */
    public Command setToPOVSniper() {
        return new InstantCommand(() -> setDriveState(DriveState.POV_SNIPER));
    }

    /*
     * STOPS DRIVE
     */
    public Command setToStop() {
        return new InstantCommand(() -> setDriveState(DriveState.STOP));
    }

    public Command setToHeadingXLock() {
        return new InstantCommand(() -> setDriveState(DriveState.HEADING_X_LOCK));
    }

    /*
     * TESTS ROTATION WHILE TRANSLATION
     */
    public Command setToDriftTest() {
        return new InstantCommand(() -> setDriveState(DriveState.DRIFT_TEST));
    }

    /*
     * TESTS 0 TO X SPEED DISTANCE
     */
    public Command setToLinearTest() {
        return new InstantCommand(() -> setDriveState(DriveState.LINEAR_TEST));
    }

    /*
     * SYS ID
     */
    public Command setToSysIDCharacterization() {
        return new InstantCommand(() -> setDriveState(DriveState.SYSID_CHARACTERIZATION));
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */

    public Module[] getModules() {
        return this.modules;
    }

    public void runVelocity(ChassisSpeeds speeds) {
      // Calculate module setpoints
      desiredSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
      SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(desiredSpeeds);
      SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);

      // Log unoptimized setpoints and setpoint speeds
      Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
      Logger.recordOutput("SwerveChassisSpeeds/Setpoints", desiredSpeeds);

      // Send setpoints to modules
      for (int i = 0; i < 4; i++) {
        modules[i].runSetpoint(setpointStates[i]);
      }

      // Log optimized setpoints (runSetpoint mutates each state)
      Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    }

    /** Runs the drive in a straight line with the specified drive output. */
    public void runCharacterization(double output) {
      for (int i = 0; i < 4; i++) {
        modules[i].runCharacterization(output);
      }
    }

    /** Stops the drive. */
    public void stop() {
      runVelocity(new ChassisSpeeds());
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
     * return to their normal orientations the next time a nonzero velocity is requested.
     */
    public void stopWithX() {
      Rotation2d[] headings = new Rotation2d[4];
      for (int i = 0; i < 4; i++) {
        headings[i] = getModuleTranslations()[i].getAngle();
      }
      kinematics.resetHeadings(headings);
      stop();
    }

    /** Returns a command to run a quasistatic test in the specified direction. */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
      return run(() -> runCharacterization(0.0))
          .withTimeout(1.0)
          .andThen(sysId.quasistatic(direction));
    }

    /** Returns a command to run a dynamic test in the specified direction. */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
      return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
    }

    /** Returns the module states (turn angles and drive velocities) for all of the modules. */
    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
      SwerveModuleState[] states = new SwerveModuleState[4];
      for (int i = 0; i < 4; i++) {
        states[i] = modules[i].getState();
      }
      return states;
    }

    /** Returns the module positions (turn angles and drive positions) for all of the modules. */
    private SwerveModulePosition[] getModulePositions() {
      SwerveModulePosition[] states = new SwerveModulePosition[4];
      for (int i = 0; i < 4; i++) {
        states[i] = modules[i].getPosition();
      }
      return states;
    }

    /** Returns the measured chassis speeds of the robot. */
    @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
    private ChassisSpeeds getChassisSpeeds() {
      return kinematics.toChassisSpeeds(getModuleStates());
    }

    /** Returns the position of each module in radians. */
    public double[] getWheelRadiusCharacterizationPositions() {
      double[] values = new double[4];
      for (int i = 0; i < 4; i++) {
        values[i] = modules[i].getWheelRadiusCharacterizationPosition();
      }
      return values;
    }

    /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
    public double getFFCharacterizationVelocity() {
      double output = 0.0;
      for (int i = 0; i < 4; i++) {
        output += modules[i].getFFCharacterizationVelocity() / 4.0;
      }
      return output;
    }

    public boolean validHeadingState(DriveState state) {
        return state.equals(DriveState.AUTON_HEADING_ALIGN) || state.equals(DriveState.HEADING_ALIGN);
    }

    /** Returns the current odometry pose. */
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
      return poseEstimator.getEstimatedPosition();
    }

    /** Returns the current odometry rotation. */
    public Rotation2d getRotation() {
      return getPose().getRotation();
    }

    /** Resets the current odometry pose. */
    public void setPose(Pose2d pose) {
      poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    }

    /** Adds a new timestamped vision measurement. */
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs) {
      poseEstimator.addVisionMeasurement(
          visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    /** Returns the maximum linear speed in meters per sec. */
    public double getMaxLinearSpeedMetersPerSec() {
      return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    }

    /** Returns the maximum angular speed in radians per sec. */
    public double getMaxAngularSpeedRadPerSec() {
      return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
    }

    /** Returns an array of module translations. */
    public static Translation2d[] getModuleTranslations() {
      return new Translation2d[] {
        new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
        new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
        new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
        new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
      };
    }

    public DriveState getDriveState() {
        return mDriveState;
    }

    public TurnPointFeedforward getDefaultTurnPointFF() {
        return new TurnPointFeedforward(
            () -> getPose(), 
            () -> getChassisSpeeds(), 
            mGoalPoseSup, 
            () -> new ChassisSpeeds());
    }

    public Rotation2d getRotationVelocity() {
        return gyroInputs.yawVelocityPerSecDeg;
    }

    public BooleanSupplier waitUntilAutoAlignFinishes() {
        return () -> mAutoAlignTimeout.calculate(mAutoAlignController.atGoal());
    }
    
    public static SwerveSetpoint resetGeneratoFromModuleStates(Module[] quadModules, ChassisSpeeds currentSpeeds) {
        return new SwerveSetpoint(
            currentSpeeds, 
            new SwerveModuleState[] {
                quadModules[0].getState(),
                quadModules[1].getState(),
                quadModules[2].getState(),
                quadModules[3].getState()
            }, 
            DriveFeedforwards.zeros(4), 
            new AzimuthFeedForward(new double[] {
                0.0, 
                0.0, 
                0.0, 
                0.0}));
    }

    public void resetSetpointGenerator() {
      resetGeneratoFromModuleStates(modules, getChassisSpeeds());
    }

    public SwerveModulePosition[] getModulePositionsHighF() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) positions[i] = modules[i].getPosition();
        return positions;
    }

    public void resetGyro() {
        /* Robot is usually facing the other way(relative to field) when doing cycles on red side, so gyro is reset to 180 */
        mRobotRotation = AllianceFlipUtil.shouldFlip() 
            ? Rotation2d.k180deg 
            : Rotation2d.kZero;

        gyroIO.resetGyro(mRobotRotation);

        poseEstimator.resetPosition(
            getRotation(), 
            getModulePositionsHighF(), 
            new Pose2d(
                getPose().getTranslation(),
                getRotation()
        ));

        // mOdometry.resetPosition(
        //     getRotation(), 
        //     getModulePositionsHighF(), 
        //     new Pose2d(
        //         getOdometryPose().getTranslation(),
        //         getRotation()
        // ));
    }

    public static void runXLock(double pTrackWidthXMeters, double pTrackWidthYMeters, Module[] pModules) {
        pModules[0].runSetpoint(
            new SwerveModuleState(
                0.0, 
                new Rotation2d(
                    pTrackWidthXMeters, 
                    pTrackWidthYMeters)));
        pModules[1].runSetpoint(
            new SwerveModuleState(
                0.0, 
                new Rotation2d(
                    -pTrackWidthXMeters,
                     pTrackWidthYMeters
        )));
        pModules[2].runSetpoint(
            new SwerveModuleState(
                0.0, 
                new Rotation2d(
                     pTrackWidthXMeters,
                    -pTrackWidthYMeters
        )));
        pModules[3].runSetpoint(
            new SwerveModuleState(
                0.0, 
                new Rotation2d(
                    -pTrackWidthXMeters,
                    -pTrackWidthYMeters
        )));
    }

    /////////////////ALIGN SETTERS////////////////////
        /*
     * Reference GameDriveManager to use game-specific implementation of mDrive command
     * @param Goal strategy, based on where you're aligning
     * @param Constraint type, linear or on an axis
     */
    public Command setToGenericAutoAlign(Supplier<Pose2d> pGoalPoseSup, ConstraintType pConstraintType) {
        return new InstantCommand(() -> {
            mGoalPoseSup = pGoalPoseSup;
            mChassisSpeedSup = () -> new ChassisSpeeds();
            mAutoAlignController.setConstraintType(pConstraintType);
            mAutoAlignController.reset(
                getPose(), 
                ChassisSpeeds.fromRobotRelativeSpeeds(
                    getChassisSpeeds(), 
                    getPose().getRotation()),
                mGoalPoseSup.get());
            }).andThen(new InstantCommand(() -> setDriveState( DriveState.AUTO_ALIGN )));
    }

    /*
     * Reference GameDriveManager to use game-specific implementation of mDrive command
     * @param Goal strategy, based on where you're aligning
     * @param Constraint type, linear or on an axis
     */
    public Command setToGenericAutoAlign(Supplier<Pose2d> pGoalPoseSup, Supplier<ChassisSpeeds> speedSup, ConstraintType pConstraintType) {
        return new InstantCommand(() -> {
            mGoalPoseSup = pGoalPoseSup;
            mChassisSpeedSup = speedSup;
            mChassisSpeedSup = () -> new ChassisSpeeds();
            mAutoAlignController.setConstraintType(pConstraintType);
            mAutoAlignController.reset(
                getPose(), 
                ChassisSpeeds.fromRobotRelativeSpeeds(
                    getChassisSpeeds(), 
                    getPose().getRotation()),
                mGoalPoseSup.get());
            }).andThen(new InstantCommand(() -> setDriveState( DriveState.AUTO_ALIGN ) ));
    }

    /*
     * Reference GameDriveManager to use game-specific implementation of mDrive command
     * Resets
     * @param Goal strategy, based on where you're aligning
     * @param Constraint type, linear or on an axis
     */
    public Command setToGenericAutoAlignWithGeneratorReset(Supplier<Pose2d> pGoalPoseSup, ConstraintType pConstraintType) {
        return new InstantCommand(() -> {
            mGoalPoseSup = pGoalPoseSup;
            mChassisSpeedSup = () -> new ChassisSpeeds();
            mAutoAlignController.setConstraintType(pConstraintType);
            mAutoAlignController.reset(
                getPose(), 
                ChassisSpeeds.fromRobotRelativeSpeeds(
                    getChassisSpeeds(), 
                    getPose().getRotation()),
                mGoalPoseSup.get());
            resetSetpointGenerator();
            }).andThen(new InstantCommand(() -> setDriveState( DriveState.AUTO_ALIGN ) ));
    }

    public Command setToGenericAutoAlignWithGeneratorReset(Supplier<Pose2d> pGoalPoseSup, Supplier<ChassisSpeeds> speedSup, ConstraintType pConstraintType) {
        return new InstantCommand(() -> {
            mGoalPoseSup = pGoalPoseSup;
            mChassisSpeedSup = speedSup;
            mAutoAlignController.setConstraintType(pConstraintType);
            mAutoAlignController.reset(
                getPose(), 
                ChassisSpeeds.fromRobotRelativeSpeeds(
                    getChassisSpeeds(), 
                    getPose().getRotation()),
                mGoalPoseSup.get());
            resetSetpointGenerator();
            }).andThen(new InstantCommand(() -> setDriveState( DriveState.AUTO_ALIGN ) ));
    }

    public Command setToGenericLineAlign(Supplier<Pose2d> pGoalPoseSup, Supplier<Rotation2d> pLineAngle, DoubleSupplier pTelScal, BooleanSupplier pTeleopInvert) {
        return new InstantCommand(() -> {
            mGoalPoseSup = pGoalPoseSup;
            mLineAlignController.setControllerGoalSettings(
                pTelScal, 
                () -> pLineAngle.get().getTan(), 
                pTeleopInvert);
            mLineAlignController.reset(
                getPose(), 
                mGoalPoseSup.get(),
                getChassisSpeeds());
        }).andThen(new InstantCommand(() -> setDriveState( DriveState.LINE_ALIGN ) ));
    }

    /*
     * Reference GameDriveManager to use game-specific implementation of mDrive command
     * @param The desired rotation
     * @param Turn feedforward
     */
    public Command setToGenericHeadingAlign(Supplier<Rotation2d> pGoalRotation, TurnPointFeedforward pTurnPointFeedforward) {
        return setToGenericHeadingAlign( pGoalRotation, pTurnPointFeedforward, DriveState.HEADING_ALIGN );
    }

    /* Accounts for velocity of drive when turning */
    public Command setToGenericHeadingAlign(Supplier<Rotation2d> pGoalRotation, Supplier<Pose2d> pGoalPoseSupplier) {
        return Commands.runOnce(() -> mGoalPoseSup = pGoalPoseSupplier )
            .andThen( setToGenericHeadingAlign( pGoalRotation, getDefaultTurnPointFF() ));
    }

    /*
     * Reference GameDriveManager to use game-specific implementation of mDrive command
     * @param The desired rotation
     * @param Turn feedforward
     */
    public Command setToGenericHeadingAlignAuton(Supplier<Rotation2d> pGoalRotation, TurnPointFeedforward pTurnPointFeedforward) {
        return setToGenericHeadingAlign( pGoalRotation, pTurnPointFeedforward, DriveState.AUTON_HEADING_ALIGN );
    }

    /* Accounts for velocity of drive when turning */
    public Command setToGenericHeadingAlignAuton(Supplier<Rotation2d> pGoalRotation, Supplier<Pose2d> pGoalPoseSupplier) {
        return Commands.runOnce(() -> mGoalPoseSup = pGoalPoseSupplier ).andThen(setToGenericHeadingAlignAuton( pGoalRotation, getDefaultTurnPointFF() ));
    }

    public Command setToGenericHeadingAlign(Supplier<Rotation2d> pGoalRotation, TurnPointFeedforward pTurnPointFeedforward, DriveState headingState) {
        return new InstantCommand(() -> {
            if(!validHeadingState(headingState)) Telemetry.reportIssue(new DriveErrors.WrongHeadingState());
            mGoalRotationSup = pGoalRotation;
            mHeadingController.setHeadingGoal(mGoalRotationSup);
            mHeadingController.reset(
                getPose().getRotation(), 
                getRotationVelocity());
            mHeadingController.setTurnPointFF(pTurnPointFeedforward);
        }).andThen(new InstantCommand(() -> setDriveState( headingState ) ));
    }

    public void acceptJoystickInputs(
            DoubleSupplier pXSupplier, DoubleSupplier pYSupplier,
            DoubleSupplier pThetaSupplier, Supplier<Rotation2d> pPOVSupplier) {
        mTeleopController.acceptJoystickInputs(pXSupplier, pYSupplier, pThetaSupplier, pPOVSupplier);
    }

    public BooleanSupplier waitUntilHeadingAlignFinishes() {
        return () -> inHeadingTolerance();
    }

    @AutoLogOutput(key = "Drive/Tolerance/HeadingController")
    public boolean inHeadingTolerance() {
        /* Accounts for angle wrapping issues with rotation 2D error */
        return mHeadingController.inTolerance(getRotation(), Rotation2d.fromDegrees(2.5));
    }

    public Command setDriveProfile(DriverProfiles profile) {
        return new InstantCommand(() -> mTeleopController.updateTuneablesWithProfiles(profile));
    }
}