// REBELLION 10014

package frc.robot.systems.drive;

import static frc.robot.systems.drive.DriveConstants.*;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.DriveFeedforwards;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.AllianceFlipUtil;
import frc.lib.math.GeomUtil;
import frc.lib.optimizations.PPRobotConfigLoader;
import frc.lib.pathplanner.AzimuthFeedForward;
import frc.lib.pathplanner.SwerveSetpoint;
import frc.lib.pathplanner.SwerveSetpointGenerator;
import frc.lib.telemetry.Telemetry;
import frc.lib.tuning.LoggedTunableNumber;
import frc.robot.systems.apriltag.ATagVision;
import frc.robot.systems.apriltag.ATagVision.VisionObservation;
import frc.robot.systems.drive.DriveManager.DriveState;
import frc.robot.systems.drive.controllers.SpeedErrorController;
import frc.robot.systems.drive.gyro.GyroIO;
import frc.robot.systems.drive.gyro.GyroInputsAutoLogged;
import frc.robot.systems.drive.modules.Module;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class Drive extends SubsystemBase {
    private final Module[] mModules;
    private final GyroIO mGyro;
    private final GyroInputsAutoLogged mGyroInputs = new GyroInputsAutoLogged();
    private final ATagVision mVision;

    private Rotation2d mRobotRotation;
    private final SwerveDriveOdometry mOdometry;
    private final SwerveDrivePoseEstimator mPoseEstimator;
    private final Field2d mField = new Field2d();
    private final Debouncer mSkidFactorDebouncer = new Debouncer(kTrustTime, DebounceType.kFalling);
    private final Debouncer mCollisionDebouncer = new Debouncer(kTrustTime, DebounceType.kFalling);
    private final Debouncer mTiltDebouncer = new Debouncer(kTrustTime, DebounceType.kFalling);

    @AutoLogOutput(key = "Drive/Odometry/AccountForSkidding")
    private boolean mHasSkidded = false;
    private double mSkidRatio = 0.0;
    private double mSkidFactor = 0.0;
    private double mGyroFactor = 1.0;
    private double mTiltFactor = 1.0;
    private double mVisionFactor = 1.0;

    public static RobotConfig mRobotConfig;
    private final SwerveSetpointGenerator mSetpointGenerator;
    private SwerveSetpoint mPreviousSetpoint =
        new SwerveSetpoint(
            new ChassisSpeeds(), 
            SwerveHelper.zeroStates(), 
            DriveFeedforwards.zeros(4), 
            AzimuthFeedForward.zeros());

    private ChassisSpeeds mDesiredSpeeds = new ChassisSpeeds();

    private SwerveModuleState[] mModuleTorques = SwerveHelper.zeroStates();
    private SwerveModuleState[] mUnOptimizedStates = SwerveHelper.zeroStates();
    private SwerveModuleState[] mSetpointStates = SwerveHelper.zeroStates();
    private SwerveModuleState[] mOptimizedStates = SwerveHelper.zeroStates();

    private DriveFeedforwards mPathPlanningFF = DriveFeedforwards.zeros(4);
    private DriveFeedforwards mDefaultFF = DriveFeedforwards.zeros(4);
    @AutoLogOutput(key = "Drive/Feedforward/Choreo")
    private boolean mUseChoreoFeedForward = false;
    @AutoLogOutput(key = "Drive/Feedforward/Filter")
    private boolean mUseFeedForward = false;
    private final PathConstraints mDriveConstraints = DriveConstants.kAutoConstraints;

    private SwerveModulePosition[] mPrevPositions = SwerveHelper.zeroPositions();
    // private Rotation2d[] mAngleDeltas = SwerveHelper.zeroRotations();
    @AutoLogOutput(key="Drive/Swerve/PreviousDriveAmps")
    private double[] mPrevDriveAmps = new double[] {0.0, 0.0, 0.0, 0.0};

    private SwerveModulePosition[] modulePositionsHighF = SwerveHelper.zeroPositions();
    private SwerveModulePosition[] moduleDeltasHighF = SwerveHelper.zeroPositions();

    private boolean kUseGenerator = true;
    private boolean resetFromModuleStates = false;
    private final SpeedErrorController mSpeedErrorController = new SpeedErrorController();

    private DriveManager mDriveManager;

    public static final LoggedTunableNumber tDriftRate = new LoggedTunableNumber("Drive/DriftRate", DriveConstants.kDriftRate);
    public static final LoggedTunableNumber tDriveFFAggressiveness = new LoggedTunableNumber("Drive/Teleop/DriveFFAggressiveness", kDriveFFAggressiveness);

    public static final LoggedTunableNumber tRotationDriftTestSpeedDeg = new LoggedTunableNumber("Drive/DriftRotationTestDeg", 360);
    public static final LoggedTunableNumber tLinearTestSpeedMPS = new LoggedTunableNumber("Drive/LinearTestMPS", 3.5);
    public static final LoggedTunableNumber tDriveCharacterizationVoltage = new LoggedTunableNumber("Drive/DriveCharacterizationVoltage", 0);
    public static final LoggedTunableNumber tDriveCharacterizationAmperage = new LoggedTunableNumber("Drive/DriveCharacterizationAmperage", 0);
    public static final LoggedTunableNumber tAzimuthCharacterizationVoltage = new LoggedTunableNumber("Drive/AzimuthCharacterizationVoltage", 0);
    public static final LoggedTunableNumber tAzimuthCharacterizationAmps = new LoggedTunableNumber("Drive/AzimuthCharacterizationAmps", 0);

    public Drive(Module[] modules, GyroIO gyro, ATagVision vision) {
        this.mModules = modules;
        this.mGyro = gyro;
        this.mVision = vision;

        mRobotRotation = mGyroInputs.iYawPosition;

        mOdometry = new SwerveDriveOdometry(kKinematics, getRobotRotation(), getModulePositionsHighF());
        mPoseEstimator = new SwerveDrivePoseEstimator(kKinematics, getRobotRotation(), getModulePositionsHighF(), new Pose2d());

        mRobotConfig = PPRobotConfigLoader.load();
        mSetpointGenerator = new SwerveSetpointGenerator(mRobotConfig, kMaxAzimuthAngularRadiansPS);
        mDriveManager = new DriveManager(this);

        PhoenixOdometryThread.getInstance().start();

        AutoBuilder.configure(
            this::getPoseEstimate, 
            this::setPose, 
            this::getRobotChassisSpeeds,
            (speeds, ff) -> {
                mDriveManager.setToAuton();;
                mDriveManager.setPPDesiredSpeeds(speeds);
                mPathPlanningFF = ff;
            },
            new PPHolonomicDriveController(kPPTranslationPID, kPPRotationPID),
            mRobotConfig, 
            () -> AllianceFlipUtil.shouldFlip(), 
            this);

        SwerveHelper.setUpPathPlanner();
        SmartDashboard.putData(mField);
    }

    public DriveManager getDriveManager() {
        return mDriveManager;
    }

    ///// ENTRY POINT TO THE DRIVE \\\\\
    @Override
    public void periodic() {
        updateSensorsAndOdometry();
        runSwerve(mDriveManager.computeDesiredSpeedsFromState());
    }

    private void updateSensorsAndOdometry() {
        try {
            kOdometryLock.lock();
            for (Module module : mModules) module.periodic();

            mGyro.updateInputs(mGyroInputs);
            Logger.processInputs("Drive/Gyro", mGyroInputs);
        } catch (Exception e) {
            Telemetry.reportException(e);
        } finally {
            kOdometryLock.unlock();
        }

        for (int i = 0; i < mModules[0].getOdometryTimeStamps().length; i++) {
            // Read wheel positions and deltas from each module
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositionsHighF[moduleIndex] = new SwerveModulePosition(
                    mModules[moduleIndex].getDrivePositions()[i],
                    mModules[moduleIndex].getAzimuthRotatinos()[i]
                );

                // mAngleDeltas[moduleIndex] = mAngleDeltas[moduleIndex].plus(
                //     GeomUtil.getSmallestChangeInRotation(
                //         modulePositionsHighF[moduleIndex].angle, 
                //         mPrevPositions[moduleIndex].angle));

                moduleDeltasHighF[moduleIndex] = new SwerveModulePosition(
                    modulePositionsHighF[moduleIndex].distanceMeters 
                        - mPrevPositions[moduleIndex].distanceMeters,
                    modulePositionsHighF[moduleIndex].angle);

                mPrevPositions[moduleIndex] = modulePositionsHighF[moduleIndex];
            }

            // Update gyro angle
            // Use the real gyro angle
            if (mGyroInputs.iConnected) mRobotRotation = mGyroInputs.odometryYawPositions[i];
            // Use the angle delta from the kinematics and module deltas
            else mRobotRotation = mRobotRotation.plus(Rotation2d.fromRadians(kKinematics.toTwist2d(moduleDeltasHighF).dtheta));

            mPoseEstimator.updateWithTime(mModules[0].getOdometryTimeStamps()[i], mRobotRotation, modulePositionsHighF);
        }

        mSkidRatio = SwerveHelper.skidRatio(getModuleStates());

        mHasSkidded = mSkidRatio > kSkidRatioCap;

        mSkidFactor = (mSkidFactorDebouncer.calculate(mHasSkidded)) 
            ? kSkidScalar 
            : 0;

        mGyroFactor = ( mCollisionDebouncer.calculate(shouldAccountForCollision())) 
            ? kCollisionScalar 
            : 1.0;

        mTiltFactor = mTiltDebouncer.calculate(shouldAccountForFlip()) 
            ? Math.min(
                kMinimumTiltFactor, 
                mGyroInputs.iPitchPosition.getCos() 
                    * mGyroInputs.iPitchPosition.getCos())
            : 1.0;
        
        mVisionFactor = (mSkidFactor + mGyroFactor);
        
        /* VISION */
        mVision.periodic(mPoseEstimator.getEstimatedPosition(), mOdometry.getPoseMeters());
        VisionObservation[] observations = mVision.getVisionObservations();
        for (VisionObservation observation : observations) {
            if (observation.hasObserved()) {
                mPoseEstimator.addVisionMeasurement(
                    observation.pose(), 
                    observation.timeStamp(), 
                    observation.stdDevs().times(mTiltFactor / mVisionFactor));
            }
        }

        mField.setRobotPose(getPoseEstimate());
        Logger.recordOutput("Drive/Odometry/SkidRatio", mSkidRatio);
        Logger.recordOutput("Drive/Odometry/HasSkidded", mHasSkidded);
        if(DriveConstants.kDoExtraLogging) {
            Logger.recordOutput("Drive/Odometry/SkidFactor", mSkidFactor);
            Logger.recordOutput("Drive/Odometry/VisionFactor", mVisionFactor);
            Logger.recordOutput("Drive/Odometry/GyroFactor", mGyroFactor);
            Logger.recordOutput("Drive/Odometry/TiltFactor", mTiltFactor);
            mOdometry.update(mRobotRotation, getModulePositionsHighF());
            Logger.recordOutput("Drive/Odometry/OdometryPose", getOdometryPose());
        }
    }

    ////////////// CHASSIS SPEED TO MODULES \\\\\\\\\\\\\\\\
    /* Sets the desired swerve module states to the robot */
    public void runSwerve(Optional<ChassisSpeeds> speeds) {
        if(speeds.isEmpty()) return;
        mDesiredSpeeds = mSpeedErrorController.correctSpeed(
            getRobotChassisSpeeds(), 
            SwerveHelper.discretize(
                speeds.get(), 
                tDriftRate.get()));

        if(DriverStation.isDisabled()) {
            mDesiredSpeeds = new ChassisSpeeds();
            mPreviousSetpoint = SwerveHelper.emptySwerveSetpoint(mModules);
        }

        if(resetFromModuleStates) {
            mPreviousSetpoint = SwerveHelper.resetGeneratoFromModuleStates(mModules, getRobotChassisSpeeds());
            resetFromModuleStates = false;
        }

        /* Logs all the possible drive states, great for debugging */
        SwerveHelper.logPossibleDriveStates(
            kDoExtraLogging, 
            mDesiredSpeeds, 
            getModuleStates(), 
            mPreviousSetpoint, 
            mRobotRotation);

        // mUnOptimizedStates = new SwerveModuleState[4];
        mSetpointStates = kKinematics.toSwerveModuleStates(mDesiredSpeeds);

        // mOptimizedStates = new SwerveModuleState[4];

        /* Saturates it for you */
        mPreviousSetpoint = mSetpointGenerator.generateSetpoint(
            mPreviousSetpoint, 
            mDesiredSpeeds,  
            mDriveConstraints, 
            SwerveHelper.dt,
            kInputVoltage);

        /* Only for logging purposes */
        mModuleTorques = SwerveHelper.zeroStates();

        mDefaultFF = mPreviousSetpoint.feedforwards();
        
        kUseGenerator = !mDriveManager.getDriveState().equals(DriveState.AUTON);
        Logger.recordOutput("Drive/Swerve/UseGenerator", kUseGenerator);

        if (kUseGenerator) {
            for (int i = 0; i < 4; i++) {
                /* Logs the drive feedforward stuff */
                // SwerveHelper.logDriveFeedforward(mDefaultFF, i);

                mSetpointStates[i] = new SwerveModuleState(
                    mPreviousSetpoint.moduleStates()[i].speedMetersPerSecond,
                    /* setpointAngle = currentAngle if the speed is less than 0.01 */
                    SwerveHelper.removeAzimuthJitter(
                        mPreviousSetpoint.moduleStates()[i], 
                        mModules[i].getCurrentState()));
                mUnOptimizedStates[i] = SwerveHelper.copyState(mSetpointStates[i]);

                mSetpointStates[i].optimize(mModules[i].getCurrentState().angle);

                /* Feedforward cases based on driveState */
                double driveAmps = 
                    calculateDriveFeedforward(mUnOptimizedStates, i);

                // Multiplies by cos(angleError) to stop the drive from going in the wrong direction
                mSetpointStates[i].cosineScale(mModules[i].getCurrentState().angle);

                mOptimizedStates[i] = mModules[i].setDesiredStateWithFF(
                    mSetpointStates[i], 
                    driveAmps, 
                    Rotation2d.fromRadians(
                        mPreviousSetpoint.azimuthFeedforwards().azimuthSpeedRadiansPS()[i]));

                /* Normalized for logging */
                mModuleTorques[i] = new SwerveModuleState(
                    (driveAmps * kMaxLinearSpeedMPS) 
                        / kDriveFOCAmpLimit, 
                    mOptimizedStates[i].angle);
            } 
        } else {
            for (int i = 0; i < 4; i++) {
                SwerveDriveKinematics.desaturateWheelSpeeds(mSetpointStates, kMaxLinearSpeedMPS);

                mSetpointStates[i] = new SwerveModuleState(
                    mSetpointStates[i].speedMetersPerSecond,
                    SwerveHelper.removeAzimuthJitter(
                        mSetpointStates[i], 
                        mModules[i].getCurrentState()));
                mUnOptimizedStates[i] = SwerveHelper.copyState(mSetpointStates[i]);

                double driveAmps = 0.0;
                if(mUseChoreoFeedForward && mDriveManager.getDriveState().equals(DriveState.AUTON)) {
                    driveAmps = SwerveHelper.convertChoreoNewtonsToAmps(
                        getModuleStates()[i], 
                        mUnOptimizedStates[i],
                        mPathPlanningFF, 
                        i);
                }

                mSetpointStates[i].optimize(mModules[i].getCurrentState().angle);
                mSetpointStates[i].cosineScale(mModules[i].getCurrentState().angle);
                mOptimizedStates[i] = mModules[i].setDesiredStateWithFF(
                    mSetpointStates[i],
                    /* GOT THE DATA -> Set to zero due to lack of data to justify using feedforward */
                    0.0);

                mPrevDriveAmps[i] = driveAmps;

                mUnOptimizedStates[i] = SwerveHelper.copyState(mSetpointStates[i]);

                mModuleTorques[i] = new SwerveModuleState(
                    (driveAmps * kMaxLinearSpeedMPS) 
                        / kDriveFOCAmpLimit, 
                    mOptimizedStates[i].angle);
            }
        }

        Telemetry.log("Drive/Swerve/SetpointsOptimized", mOptimizedStates);
        Telemetry.log("Drive/Swerve/SetpointsChassisSpeeds", kKinematics.toChassisSpeeds(mOptimizedStates));
        if(DriveConstants.kDoExtraLogging) {
            Telemetry.log("Drive/Swerve/Setpoints", mUnOptimizedStates);
            Telemetry.log("Drive/Odometry/FieldSetpointChassisSpeed", ChassisSpeeds.fromRobotRelativeSpeeds(
            kKinematics.toChassisSpeeds(mOptimizedStates), mRobotRotation));
        }
        Telemetry.log("Drive/Swerve/ModuleTorqueFF", mModuleTorques);
    }

    /* Calculates DriveFeedforward based off state */
    public double calculateDriveFeedforward(SwerveModuleState[] unoptimizedSetpointStates, int i) {
        double driveAmps = (mUseChoreoFeedForward) 
            ? SwerveHelper.convertChoreoNewtonsToAmps(
                getModuleStates()[i], 
                unoptimizedSetpointStates[i],
                mPathPlanningFF, 
                i)
            : mDefaultFF.torqueCurrentsAmps()[i] 
                * SwerveHelper.ppFFScalar(
                    getModuleStates()[i], 
                    unoptimizedSetpointStates[i],
                    i);

        if(!mUseFeedForward) {
            driveAmps = 0.0; 
            // SwerveHelper.lowPassFilter(
            //     mPrevDriveAmps[i], 
            //     driveAmps, 
            //     tDriveFFAggressiveness.get());
        }

        mPrevDriveAmps[i] = driveAmps;
        return driveAmps;
    }

    public void setFFModel(boolean pUseChoreoFeedForward, boolean pUseFeedForward) {
        mUseChoreoFeedForward = pUseChoreoFeedForward;
        mUseFeedForward = pUseFeedForward;
    }

    public void setDriveFeedforwards(DriveFeedforwards ffs) {
        mPathPlanningFF = ffs;
    }

    ////////////// LOCALIZATION(MAINLY RESETING LOGIC) \\\\\\\\\\\\\\\\
    public void resetGyro() {
        /* Robot is usually facing the other way(relative to field) when doing cycles on red side, so gyro is reset to 180 */
        mRobotRotation = AllianceFlipUtil.shouldFlip() 
            ? Rotation2d.k180deg 
            : Rotation2d.kZero;

        mGyro.resetGyro(mRobotRotation);

        mPoseEstimator.resetPosition(
            getRobotRotation(), 
            getModulePositionsHighF(), 
            new Pose2d(
                getPoseEstimate().getTranslation(),
                getRobotRotation()
        ));

        mOdometry.resetPosition(
            getRobotRotation(), 
            getModulePositionsHighF(), 
            new Pose2d(
                getOdometryPose().getTranslation(),
                getRobotRotation()
        ));
    }

    public void setPose(Pose2d pose) {
        setPoses(pose, pose);
    }

    public void setPoses(Pose2d estimatorPose, Pose2d odometryPose) {
        mRobotRotation = estimatorPose.getRotation();
        mGyro.resetGyro(mRobotRotation);
        // Safe to pass in odometry poses because of the syncing
        // between gyro and pose estimator in reset gyro function
        mPoseEstimator.resetPosition(getRobotRotation(), getModulePositionsHighF(), estimatorPose);
        mOdometry.resetPosition(getRobotRotation(), getModulePositionsHighF(), odometryPose);
    }

    public void resetModulesEncoders() {
        for (int i = 0; i < 4; i++) mModules[i].resetAzimuthEncoder();
    }

    ///////////////////////// GETTERS \\\\\\\\\\\\\\\\\\\\\\\\
    @AutoLogOutput(key = "Drive/Swerve/MeasuredStates")
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) states[i] = mModules[i].getCurrentState();
        return states;
    }

    // @AutoLogOutput(key = "Drive/Swerve/ModulePositions")
    public SwerveModulePosition[] getModulePositionsHighF() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) positions[i] = mModules[i].getCurrentPosition();
        return positions;
    }

    @AutoLogOutput(key = "Drive/Odometry/PoseEstimate")
    public Pose2d getPoseEstimate() {
        return mPoseEstimator.getEstimatedPosition();
    }

    // @AutoLogOutput(key = "Drive/Odometry/OdometryPose")
    public Pose2d getOdometryPose() {
        return mOdometry.getPoseMeters();
    }

    @AutoLogOutput(key = "Drive/Odometry/RobotRotation")
    public Rotation2d getRobotRotation() {
        return mRobotRotation;
    }

    // @AutoLogOutput(key = "Drive/Odometry/RobotRotationVelocity")
    public Rotation2d getRobotRotationVelocity() {
        return mGyroInputs.iYawVelocityPS;
    }

    @AutoLogOutput(key = "Drive/Odometry/RobotChassisSpeeds")
    public ChassisSpeeds getRobotChassisSpeeds() {
        return kKinematics.toChassisSpeeds(getModuleStates());
    }

    @AutoLogOutput(key = "Drive/Odometry/DesiredChassisSpeeds")
    public ChassisSpeeds getDesiredChassisSpeeds() {
        return mDesiredSpeeds;
    }

    @AutoLogOutput( key = "Drive/Odometry/AccountForFlip")
    public boolean shouldAccountForFlip() {
        return kAccountForTilt && (
            (DriveConstants.kTiltCutOffPitch.getDegrees() < mGyroInputs.iPitchPosition.getDegrees())
                ||
            (DriveConstants.kTiltCutOffRoll.getDegrees() < mGyroInputs.iRollPosition.getDegrees())
        );
    }

    @AutoLogOutput(key = "Drive/Odometry/AccountForCollision")
    public boolean shouldAccountForCollision() {
        return getAccelerationVectorWithoutGravityMPS2() > kCollisionCapG;
    }

    public RobotConfig getPPRobotConfig() {
        return mRobotConfig;
    }

    public DriveFeedforwards getDriveFeedforwards() {
        return mPathPlanningFF;
    }

    public Module[] getModules() {
        return this.mModules;
    }

    public boolean isRobotStationary() {
        return (getRobotChassisSpeeds().vxMetersPerSecond < 0.05) &&
                (getRobotChassisSpeeds().vyMetersPerSecond < 0.05) &&
                (Math.toDegrees(getRobotChassisSpeeds().omegaRadiansPerSecond) < 5.0);
    }

    public void setDriveFeedforwardsFromChoreo(SwerveSample sample) {
        setDriveFeedforwards(new DriveFeedforwards(
            new double[] {0.0, 0.0, 0.0, 0.0}, 
            new double[] {0.0, 0.0, 0.0, 0.0}, 
            new double[] {0.0, 0.0, 0.0, 0.0}, 
            sample.moduleForcesX(), 
            sample.moduleForcesY()));
    }

    @AutoLogOutput(key = "Drive/Odometry/AccelNoGrav")
    public double getAccelerationVectorWithoutGravityMPS2() {
        return GeomUtil.hypot(
            mGyroInputs.iAccXG, 
            mGyroInputs.iAccYG);
    }

    public void resetSetpointGenerator() {
        resetFromModuleStates = true;
    }
}