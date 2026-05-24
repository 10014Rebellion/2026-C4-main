package frc.robot.systems.drive;

import edu.wpi.first.math.Vector;

// import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;

import static frc.robot.systems.drive.DriveConstants.kAzimuthDriveScalar;
import static frc.robot.systems.drive.DriveConstants.kDriveMotorGearing;
import static frc.robot.systems.drive.DriveConstants.kKinematics;
import static frc.robot.systems.drive.DriveConstants.kMaxLinearSpeedMPS;
import static frc.robot.systems.drive.DriveConstants.kSkidRatioCap;
import static frc.robot.systems.drive.DriveConstants.kWheelInertia;
import static frc.robot.systems.drive.DriveConstants.kWheelRadiusMeters;

import java.util.Arrays;

import org.littletonrobotics.junction.Logger;

import frc.lib.pathplanner.AzimuthFeedForward;
import frc.lib.pathplanner.SwerveSetpoint;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.math.EqualsUtil;
import frc.lib.telemetry.Telemetry;
import frc.robot.systems.drive.modules.Module;

public class SwerveHelper {
    public static final double dt = 0.02;
    public static final DCMotor kKrakenFOCModel = DCMotor.getKrakenX60Foc(1);
    public static final double kJitterThreshold = 0.01;

    /* 
     * Helps driftRate value helps account for translation drift while rotating and riving
     * Also makes speed go from continous to discrete. Technical details:
     * https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964
     */
    public static ChassisSpeeds discretize(ChassisSpeeds speeds, double driftRate) {
        var desiredDeltaPose = new Pose2d(
            speeds.vxMetersPerSecond * dt,
            speeds.vyMetersPerSecond * dt,
            new Rotation2d(speeds.omegaRadiansPerSecond * dt * driftRate));
        var twist = new Pose2d().log(desiredDeltaPose);

        double discOmegaRadiansPerSecond = ChassisSpeeds.discretize(speeds, dt).omegaRadiansPerSecond;

        return new ChassisSpeeds((twist.dx / dt), (twist.dy / dt), (discOmegaRadiansPerSecond));
    }

    public static SwerveModuleState copyState(SwerveModuleState state) {
        return new SwerveModuleState(state.speedMetersPerSecond, state.angle);
    }

    /* If setpoint is very low keep module at current angle, as it causes jitter trying to control such little movement */
    public static Rotation2d removeAzimuthJitter(SwerveModuleState setpoint, SwerveModuleState current) {
        return Math.abs(setpoint.speedMetersPerSecond / kMaxLinearSpeedMPS) < kJitterThreshold ? current.angle : setpoint.angle;
    }


    public static double projectTorque(SwerveModuleState currentState, Vector<N2> wheelForce) {
        Vector<N2> wheelDirection = VecBuilder.fill(currentState.angle.getCos(), currentState.angle.getSin());
        double wheelForceN = wheelForce.dot(wheelDirection);
        return wheelForceN; 
    }

    // PATHPLANNER TORQUE UTILS \\ 
    /* Torque isn't directionally changed when the velocity flip case happen SwerveModuleState.optimize() */
    public static double optimizeTorque(SwerveModuleState unOptimized, SwerveModuleState optimized, double motorAmperage, SwerveModuleState current, int i) {
        return (isSpeedOptimized(unOptimized, optimized, i) ? -motorAmperage : motorAmperage) * Math.abs(optimized.angle.minus(current.angle).getCos());
    }

    /* Check if optimize changed module velocity direction */
    public static boolean isSpeedOptimized(SwerveModuleState state, SwerveModuleState optimizedState, int i) {
        boolean isSpeedOptimized = !EqualsUtil.epsilonEquals(state.speedMetersPerSecond, optimizedState.speedMetersPerSecond);
        Telemetry.log("Drive/Swerve/Feedforward/"+i+"/isSpeedOptimized", isSpeedOptimized);
        return isSpeedOptimized;
    }

    // ASSUMES THE CHOREO'S MOTOR TORQUE DOESN'T ALREADY EXCEED THE MOTOR'S LIMIT
    public static double convertChoreoNewtonsToAmps(SwerveModuleState currentState, SwerveModuleState unoptimizedDesiredState, DriveFeedforwards ff, int i) {
        double choreoLinearForceNewtons = 
            ppFFScalar(
                currentState, 
                unoptimizedDesiredState,
                i) 
            * Math.hypot(
                ff.robotRelativeForcesXNewtons()[i], 
                ff.robotRelativeForcesYNewtons()[i])
            * Math.signum(projectTorque(currentState, 
                VecBuilder.fill(
                    ff.robotRelativeForcesXNewtons()[i], 
                    ff.robotRelativeForcesYNewtons()[i])));

        // NEWTONS -> GEARBOX TORQUE -> MOTOR TORQUE
        double driveMotorTorque = (choreoLinearForceNewtons * kWheelRadiusMeters) / kDriveMotorGearing;
        double driveMotorAmperage = kKrakenFOCModel.getCurrent(driveMotorTorque);

        return driveMotorAmperage;
    }

    public static double ppFFScalar(SwerveModuleState currentState, SwerveModuleState unoptimizedDesiredState, int i) {
        boolean currentSpeedZero = ((EqualsUtil.epsilonEquals(currentState.speedMetersPerSecond, 0.0)));

        Vector<N2> wheelDirection = VecBuilder.fill(
            ((currentSpeedZero) ? 1.0 : Math.signum(currentState.speedMetersPerSecond)) 
                * currentState.angle.getCos(), 
            ((currentSpeedZero) ? 1.0 : Math.signum(currentState.speedMetersPerSecond)) 
                * currentState.angle.getSin());

        boolean unoptimizedSpeedZero = (EqualsUtil.epsilonEquals(unoptimizedDesiredState.speedMetersPerSecond, 0.0));

        Vector<N2> setpointDirection = VecBuilder.fill(
            ((unoptimizedSpeedZero) ? 1.0 : Math.signum(unoptimizedDesiredState.speedMetersPerSecond)) 
                * unoptimizedDesiredState.angle.getCos(), 
            ((unoptimizedSpeedZero) ? 1.0 : Math.signum(unoptimizedDesiredState.speedMetersPerSecond)) 
                * unoptimizedDesiredState.angle.getSin());

        Logger.recordOutput("Drive/ppFfScalar/"+i, wheelDirection.dot(setpointDirection));
        
        return wheelDirection.dot(setpointDirection);
    }

    public static void logDriveFeedforward(DriveFeedforwards ff, int i) {
        Telemetry.log("Drive/Swerve/Feedforward/"+i+"/Acceleration", ff.accelerationsMPSSq()[i]);
        Telemetry.log("Drive/Swerve/Feedforward/"+i+"/Force", ff.linearForcesNewtons()[i]);
        Telemetry.log("Drive/Swerve/Feedforward/"+i+"/Current", ff.torqueCurrentsAmps()[i]);
    }

    /* Log different variations of the desired swerve module states */
    public static void logPossibleDriveStates(boolean doLogging, ChassisSpeeds desiredSpeeds, SwerveModuleState[] currentStates, SwerveSetpoint previousSetpoint, Rotation2d robotRotation) {
        if(doLogging) {
            /* Regular setpoint generation */
            SwerveModuleState[] unOptimizedSetpointStates = kKinematics.toSwerveModuleStates(desiredSpeeds);
            for(int i = 0; i < 4; i++) {
                unOptimizedSetpointStates[i] = new SwerveModuleState(
                    unOptimizedSetpointStates[i].speedMetersPerSecond,
                    removeAzimuthJitter(unOptimizedSetpointStates[i], currentStates[i]));
                unOptimizedSetpointStates[i].optimize(currentStates[i].angle);
                unOptimizedSetpointStates[i].cosineScale(currentStates[i].angle);
            }
            SwerveDriveKinematics.desaturateWheelSpeeds(unOptimizedSetpointStates, kMaxLinearSpeedMPS);
            Telemetry.log("Drive/Swerve/preOptimizedSetpoints", unOptimizedSetpointStates);

            unOptimizedSetpointStates = kKinematics.toSwerveModuleStates(desiredSpeeds);
            for(int i = 0; i < 4; i++) {
                unOptimizedSetpointStates[i] = new SwerveModuleState(
                    unOptimizedSetpointStates[i].speedMetersPerSecond,
                    removeAzimuthJitter(unOptimizedSetpointStates[i], currentStates[i]));
                unOptimizedSetpointStates[i].optimize(currentStates[i].angle);
                unOptimizedSetpointStates[i].cosineScale(currentStates[i].angle);
            }
            Telemetry.log("Drive/Swerve/saturatedPreOptimizedSetpoints", unOptimizedSetpointStates);
            Telemetry.log("Drive/Odometry/preOptimizedChassisSpeeds", kKinematics.toChassisSpeeds(unOptimizedSetpointStates));

            /* Non-generated swerve setpoints */
            SwerveModuleState[] swerveModuleStates = kKinematics.toSwerveModuleStates(desiredSpeeds);

            Telemetry.log("Drive/Swerve/RegularSetpoints", swerveModuleStates);

            SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxLinearSpeedMPS);

            Telemetry.log("Drive/Swerve/SaturatedRegularSetpoints", swerveModuleStates);
            Telemetry.log("Drive/Odometry/FieldRegularChassisSpeeds", ChassisSpeeds.fromRobotRelativeSpeeds(
                kKinematics.toChassisSpeeds(swerveModuleStates), robotRotation));
        }
    }

    public static SwerveModuleState[] zeroStates() {
        return new SwerveModuleState[] {
            new SwerveModuleState(), new SwerveModuleState(),
            new SwerveModuleState(), new SwerveModuleState()
        };
    }

    public static SwerveModulePosition[] zeroPositions() {
        return new SwerveModulePosition[] {
            new SwerveModulePosition(), new SwerveModulePosition(),
            new SwerveModulePosition(), new SwerveModulePosition()
        };
    }

    public static Rotation2d[] zeroRotations() {
        return new Rotation2d[] {
            new Rotation2d(), new Rotation2d(), new Rotation2d(), new Rotation2d()
        };
    }

    public static double getTorqueOfKrakenDriveMotor(double amps) {
        return kKrakenFOCModel.getTorque(amps);
    }

    public static double lowPassFilter(double previous, double input, double alpha) {
        return alpha * input + (1 - alpha) * previous;
    }

    public static double skidRatio(SwerveModuleState[] deltas) {
        ChassisSpeeds speeds = kKinematics.toChassisSpeeds(deltas);
        ChassisSpeeds rotationalSpeeds = new ChassisSpeeds(0.0, 0.0, speeds.omegaRadiansPerSecond);
        SwerveModuleState[] rotationalModuleStates = kKinematics.toSwerveModuleStates(rotationalSpeeds);

        double[] moduleTranslationMagnitudes = new double[4];
        for(int i = 0; i < 4; i++) {
            moduleTranslationMagnitudes[i] = Math.hypot(
                (deltas[i].speedMetersPerSecond * deltas[i].angle.getCos())
                    -
                (rotationalModuleStates[i].speedMetersPerSecond * rotationalModuleStates[i].angle.getCos()),
                (deltas[i].speedMetersPerSecond * deltas[i].angle.getSin())
                    -
                (rotationalModuleStates[i].speedMetersPerSecond * rotationalModuleStates[i].angle.getSin()));
        }

        Arrays.sort(moduleTranslationMagnitudes);

        double ratio = moduleTranslationMagnitudes[3] / moduleTranslationMagnitudes[0];
        if(moduleTranslationMagnitudes[0] < 0.1 && moduleTranslationMagnitudes[3] < 0.1) return 0.0;
        if(moduleTranslationMagnitudes[0] == 0.0 || moduleTranslationMagnitudes[3] == 0.0) return 0.0;
        if(Double.isNaN(ratio) || Double.isInfinite(ratio)) return kSkidRatioCap+1.0;

        return ratio;
    }

    public static double skidRatio(ChassisSpeeds speeds, ChassisSpeeds rotSpeeds) {
        SwerveModuleState[] deltas = kKinematics.toSwerveModuleStates(speeds);
        SwerveModuleState[] rotationalModuleStates = kKinematics.toSwerveModuleStates(rotSpeeds);

        double[] moduleTranslationMagnitudes = new double[4];
        for(int i = 0; i < 4; i++) {
            moduleTranslationMagnitudes[i] = Math.hypot(
                (deltas[i].speedMetersPerSecond * deltas[i].angle.getCos())
                    -
                (rotationalModuleStates[i].speedMetersPerSecond * rotationalModuleStates[i].angle.getCos()),
                (deltas[i].speedMetersPerSecond * deltas[i].angle.getSin())
                    -
                (rotationalModuleStates[i].speedMetersPerSecond * rotationalModuleStates[i].angle.getSin()));
        }

        Arrays.sort(moduleTranslationMagnitudes);

        double ratio = moduleTranslationMagnitudes[0] / moduleTranslationMagnitudes[3];

        if(moduleTranslationMagnitudes[0] == 0.0 || moduleTranslationMagnitudes[3] == 0.0) return 0.0;
        if(Double.isNaN(ratio) || Double.isInfinite(ratio)) return kSkidRatioCap+1.0;

        return ratio;
    }

    /* Intended as a brute force sign conversion, not technically correct if  if velocity change is zero as FF can occure even if vel = 0 */
    public static double correctAmpFFDirection(SwerveModuleState optimizedState, SwerveModuleState prevOptimizedState, double driveAmps, int modNumber) {
        double directionOfVelChange = Math.signum(
            optimizedState.speedMetersPerSecond - prevOptimizedState.speedMetersPerSecond);
        Telemetry.log("Drive/Module/Feedforward/" + modNumber + "/dir", directionOfVelChange);
        driveAmps = Math.abs(driveAmps) * Math.signum(directionOfVelChange);
        return driveAmps;
    }

    public static void setUpPathPlanner() {
        // Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback((activePath) ->
            Telemetry.log("Drive/Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()])));
        PathPlannerLogging.setLogTargetPoseCallback(
            (targetPose) -> Telemetry.log("Drive/Odometry/TrajectorySetpoint", targetPose));
    }

    public static double deadReckoningTurnToDriveConv(Rotation2d angleDelta, double gearing, double wheelRadius) {
        return angleDelta.getRadians() * gearing * wheelRadius;
    }

    public static double deadReckoningFeedforward(Rotation2d angleDelta) {
        double distM = deadReckoningTurnToDriveConv(angleDelta, kDriveMotorGearing, kWheelRadiusMeters);
        double accelerationM = (2 * distM) / (dt * dt);

        return -kKrakenFOCModel.getCurrent(kWheelRadiusMeters * accelerationM * kWheelInertia) * kAzimuthDriveScalar;
    }

    public static SwerveSetpoint emptySwerveSetpoint(Module[] quadModules) {
        return new SwerveSetpoint(
            new ChassisSpeeds(), 
            new SwerveModuleState[] {
                new SwerveModuleState(
                    0.0,
                    quadModules[0].getCurrentState().angle),
                new SwerveModuleState(
                    0.0,
                    quadModules[1].getCurrentState().angle),
                new SwerveModuleState(
                    0.0,
                    quadModules[2].getCurrentState().angle),
                new SwerveModuleState(
                    0.0,
                    quadModules[3].getCurrentState().angle)
            }, 
            DriveFeedforwards.zeros(4), 
            new AzimuthFeedForward(new double[] {
                0.0, 
                0.0, 
                0.0, 
                0.0}));
    }

    public static SwerveSetpoint resetGeneratoFromModuleStates(Module[] quadModules, ChassisSpeeds currentSpeeds) {
        return new SwerveSetpoint(
            currentSpeeds, 
            new SwerveModuleState[] {
                quadModules[0].getCurrentState(),
                quadModules[1].getCurrentState(),
                quadModules[2].getCurrentState(),
                quadModules[3].getCurrentState()
            }, 
            DriveFeedforwards.zeros(4), 
            new AzimuthFeedForward(new double[] {
                0.0, 
                0.0, 
                0.0, 
                0.0}));
    }

    public static void runXLock(double pTrackWidthXMeters, double pTrackWidthYMeters, Module[] pModules) {
        pModules[0].setDesiredState(
            new SwerveModuleState(
                0.0, 
                new Rotation2d(
                    pTrackWidthXMeters, 
                    pTrackWidthYMeters)));
        pModules[1].setDesiredState(
            new SwerveModuleState(
                0.0, 
                new Rotation2d(
                    -pTrackWidthXMeters,
                     pTrackWidthYMeters
        )));
        pModules[2].setDesiredState(
            new SwerveModuleState(
                0.0, 
                new Rotation2d(
                     pTrackWidthXMeters,
                    -pTrackWidthYMeters
        )));
        pModules[3].setDesiredState(
            new SwerveModuleState(
                0.0, 
                new Rotation2d(
                    -pTrackWidthXMeters,
                    -pTrackWidthYMeters
        )));
    }
}