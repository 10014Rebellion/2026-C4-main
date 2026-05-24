package frc.robot.commands;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.events.EventScheduler;
import com.pathplanner.lib.path.*;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PPLibTelemetry;
import com.pathplanner.lib.util.PathPlannerLogging;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.*;
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.commands.PathPlannerAuto;
import frc.lib.math.AllianceFlipUtil;

/** Base command for following a path. Uses pathplanner's pathing command with Choreo's trigger system */
public class FollowPathCommand extends Command {
    private final Timer timer = new Timer();
    private final PathPlannerPath originalPath;
    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<ChassisSpeeds> speedsSupplier;
    private final BiConsumer<ChassisSpeeds, DriveFeedforwards> output;
    private final PathFollowingController controller;
    private final RobotConfig robotConfig;
    private final BooleanSupplier shouldFlipPath;
    private final boolean shouldResetOdometry;
    private final Consumer<Pose2d> resetPose;
    private final EventScheduler eventScheduler;

    private AutoEvent mAuto;
    private int pollCount = 0;
    private EventLoop loop;

    private PathPlannerPath path;
    private PathPlannerTrajectory trajectory;

    private boolean mHasEnded = false;
    private boolean mIsRunning = false;

    public FollowPathCommand(
        PathPlannerPath path,
        Supplier<Pose2d> poseSupplier,
        Supplier<ChassisSpeeds> speedsSupplier,
        BiConsumer<ChassisSpeeds, DriveFeedforwards> output,
        PathFollowingController controller,
        RobotConfig robotConfig,
        BooleanSupplier shouldFlipPath,
        boolean shouldResetOdometry,
        Consumer<Pose2d> resetPose,
        Subsystem... requirements) {
        this(
            path, 
            poseSupplier, 
            speedsSupplier, 
            output, 
            controller, 
            robotConfig, 
            shouldFlipPath, 
            shouldResetOdometry, 
            resetPose, 
            new AutoEvent("EmptyAuto", new Subsystem() {
                
            }), 
            requirements);
            loop = new EventLoop();
    }

    /**
     * Construct a base path following command
     *
     * @param path The path to follow
     * @param poseSupplier Function that supplies the current field-relative pose of the robot
     * @param speedsSupplier Function that supplies the current robot-relative chassis speeds
     * @param output Output function that accepts robot-relative ChassisSpeeds and feedforwards for
     *     each drive motor. If using swerve, these feedforwards will be in FL, FR, BL, BR order. If
     *     using a differential drive, they will be in L, R order.
     *     <p>NOTE: These feedforwards are assuming unoptimized module states. When you optimize your
     *     module states, you will need to reverse the feedforwards for modules that have been flipped
     * @param controller Path following controller that will be used to follow the path
     * @param robotConfig The robot configuration
     * @param shouldFlipPath Should the path be flipped to the other side of the field? This will
     *     maintain a global blue alliance origin.
     * @param requirements Subsystems required by this command, usually just the drive subsystem
     */
    public FollowPathCommand(
        PathPlannerPath path,
        Supplier<Pose2d> poseSupplier,
        Supplier<ChassisSpeeds> speedsSupplier,
        BiConsumer<ChassisSpeeds, DriveFeedforwards> output,
        PathFollowingController controller,
        RobotConfig robotConfig,
        BooleanSupplier shouldFlipPath,
        boolean shouldResetOdometry,
        Consumer<Pose2d> resetPose,
        AutoEvent pAuto,
        Subsystem... requirements) {
        this.originalPath = path;
        this.poseSupplier = poseSupplier;
        this.speedsSupplier = speedsSupplier;
        this.output = output;
        this.controller = controller;
        this.robotConfig = robotConfig;
        this.shouldFlipPath = shouldFlipPath;
        this.shouldResetOdometry = shouldResetOdometry;
        this.resetPose = resetPose;
        this.eventScheduler = new EventScheduler();

        this.mAuto = pAuto;
        this.loop = pAuto.getLoop();

        Set<Subsystem> driveRequirements = Set.of(requirements);
        addRequirements(requirements);

        // Add all event scheduler requirements to this command's requirements
        var eventReqs = EventScheduler.getSchedulerRequirements(this.originalPath);
        if (!Collections.disjoint(driveRequirements, eventReqs)) {
            throw new IllegalArgumentException(
            "Events that are triggered during path following cannot require the drive subsystem");
        }
        addRequirements(eventReqs);

        this.path = this.originalPath;
        // Ensure the ideal trajectory is generated
        Optional<PathPlannerTrajectory> idealTrajectory =
            this.path.getIdealTrajectory(this.robotConfig);
        idealTrajectory.ifPresent(traj -> this.trajectory = traj);
    }

    @Override
    public void initialize() {
        mHasEnded = false;
        mIsRunning = true;
        pollCount = 0;

        if (shouldFlipPath.getAsBoolean() && !originalPath.preventFlipping) {
            path = originalPath.flipPath();
        } else {
            path = originalPath;
        }

        Pose2d currentPose = poseSupplier.get();
        ChassisSpeeds currentSpeeds = speedsSupplier.get    ();

        controller.reset(currentPose, currentSpeeds);

        double linearVel = Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);

        if (path.getIdealStartingState() != null) {
            // Check if we match the ideal starting state
            boolean idealVelocity =
                Math.abs(linearVel - path.getIdealStartingState().velocityMPS()) <= 0.25;
            boolean idealRotation =
                !robotConfig.isHolonomic
                    || Math.abs(
                        currentPose
                          .getRotation()
                          .minus(path.getIdealStartingState().rotation())
                          .getDegrees())
                    <= 30.0;
            if (idealVelocity && idealRotation) {
                // We can use the ideal trajectory
                trajectory = path.getIdealTrajectory(robotConfig).orElseThrow();
            } else {
                // We need to regenerate
                trajectory = path.generateTrajectory(currentSpeeds, currentPose.getRotation(), robotConfig);
            }
        } else {
            // No ideal starting state, generate the trajectory
            trajectory = path.generateTrajectory(currentSpeeds, currentPose.getRotation(), robotConfig);
        }

        if(shouldResetOdometry) resetPose.accept(trajectory.getInitialPose());

        PathPlannerAuto.setCurrentTrajectory(trajectory);
        PathPlannerAuto.currentPathName = originalPath.name;

        PathPlannerLogging.logActivePath(path);
        PPLibTelemetry.setCurrentPath(path);

        eventScheduler.initialize(trajectory);

        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        pollCount++;
        double currentTime = timer.get();
        var targetState = trajectory.sample(currentTime);
        if (!controller.isHolonomic() && path.isReversed()) {
            targetState = targetState.reverse();
        }

        Pose2d currentPose = poseSupplier.get();
        ChassisSpeeds currentSpeeds = speedsSupplier.get();

        ChassisSpeeds targetSpeeds = controller.calculateRobotRelativeSpeeds(currentPose, targetState);

        double currentVel =
            Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);

        PPLibTelemetry.setCurrentPose(currentPose);
        PathPlannerLogging.logCurrentPose(currentPose);

        PPLibTelemetry.setTargetPose(targetState.pose);
        PathPlannerLogging.logTargetPose(targetState.pose);

        PPLibTelemetry.setVelocities(
            currentVel,
            targetState.linearVelocity,
            currentSpeeds.omegaRadiansPerSecond,
            targetSpeeds.omegaRadiansPerSecond);

        output.accept(targetSpeeds, targetState.feedforwards);

        eventScheduler.execute(currentTime);
    }

    @Override
    public boolean isFinished() {
        double totalTime = trajectory.getTotalTimeSeconds();
        return timer.hasElapsed(totalTime) || !Double.isFinite(totalTime);
    }

    @Override
    public void end(boolean interrupted) {
        pollCount = 0;
        timer.stop();
        PathPlannerAuto.currentPathName = "";
        PathPlannerAuto.setCurrentTrajectory(null);

        // Only output 0 speeds when ending a path that is supposed to stop, this allows interrupting
        // the command to smoothly transition into some auto-alignment routine
        if (!interrupted && path.getGoalEndState().velocityMPS() < 0.1) {
            output.accept(new ChassisSpeeds(), DriveFeedforwards.zeros(robotConfig.numModules));
        }

        PathPlannerLogging.logActivePath(null);

        eventScheduler.end();

        mHasEnded = true;
        mIsRunning = false;
    }

    private Trigger timeTrigger(double targetTime, Timer timer) {
        // Make the trigger only be high for 1 cycle when the time has elapsed
        return new Trigger(
            loop,
            new BooleanSupplier() {
                double lastTimestamp = -1.0;
                OptionalInt pollTarget = OptionalInt.empty();

                public boolean getAsBoolean() {
                    if (!timer.isRunning()) {
                        lastTimestamp = -1.0;
                        pollTarget = OptionalInt.empty();
                        return false;
                    }
                    double nowTimestamp = timer.get();
                    try {
                        boolean timeAligns = 
                            lastTimestamp < targetTime 
                                && nowTimestamp >= targetTime;
                        if (pollTarget.isEmpty() && timeAligns) {
                            // if the time aligns for this cycle and it hasn't aligned previously this cycle
                            pollTarget = OptionalInt.of(pollCount);
                            return true;
                        } else if (pollTarget.isPresent() && pollCount == pollTarget.getAsInt()) {
                            // if the time aligned previously this cycle
                            return true;
                        } else if (pollTarget.isPresent()) {
                            // if the time aligned last cycle
                            pollTarget = OptionalInt.empty();
                            return false;
                        }
                        return false;
                    } finally {
                        lastTimestamp = nowTimestamp;
                    }
                }
            });
    }

    private Trigger enterExitTrigger(Trigger enter, Trigger exit) {
        return new Trigger(
            loop,
            new BooleanSupplier() {
                boolean output = false;

                @Override
                public boolean getAsBoolean() {
                    if (enter.getAsBoolean()) {
                        output = true;
                    }
                    if (exit.getAsBoolean()) {
                        output = false;
                    }
                    return output;
                }
        });
    }

    /**
     * Returns a trigger that rises to true a number of cycles after the trajectory ends and falls
     * after one pulse.
     *
     * <p>This is different from isRunning() in a few ways.
     *
     * <ul>
     *   <li>This will never be true if the trajectory is interrupted
     *   <li>This will never be true before the trajectory is run
     *   <li>This will fall the next cycle after the trajectory ends
     * </ul>
     *
     * <p>Why does the trigger need to fall?
     *
     * <pre><code>
     * //Lets say we had this code segment
     * Trigger hasGamepiece = ...;
     * Trigger noGamepiece = hasGamepiece.negate();
     *
     * AutoTrajectory rushMidTraj = ...;
     * AutoTrajectory goShootGamepiece = ...;
     * AutoTrajectory pickupAnotherGamepiece = ...;
     *
     * routine.enabled().onTrue(rushMidTraj.cmd());
     *
     * rushMidTraj.doneDelayed(10).and(noGamepiece).onTrue(pickupAnotherGamepiece.cmd());
     * rushMidTraj.doneDelayed(10).and(hasGamepiece).onTrue(goShootGamepiece.cmd());
     *
     * // If done never falls when a new trajectory is scheduled
     * // then these triggers leak into the next trajectory, causing the next note pickup
     * // to trigger goShootGamepiece.cmd() even if we no longer care about these checks
     * </code></pre>
     *
     * @param seconds The seconds to delay the trigger from rising to true.
     * @return A trigger that is true when the trajectory is finished.
     */
    public Trigger doneDelayed(double seconds) {
        return mAuto.loggedCondition(
            path.name+"/DoneDelayedSeconds/"+seconds, 
            timeTrigger(seconds, timer).and(new Trigger(loop, () -> mHasEnded)),
            false,
            loop);
    }

    /**
     * Returns a trigger that rises to true when the trajectory ends and falls after one pulse.
     *
     * <p>This is different from isRunning() in a few ways.
     *
     * <ul>
     *   <li>This will never be true if the trajectory is interrupted
     *   <li>This will never be true before the trajectory is run
     *   <li>This will fall the next cycle after the trajectory ends
     * </ul>
     *
     * <p>Why does the trigger need to fall?
     *
     * <pre><code>
     * //Lets say we had this code segment
     * Trigger hasGamepiece = ...;
     * Trigger noGamepiece = hasGamepiece.negate();
     *
     * AutoTrajectory rushMidTraj = ...;
     * AutoTrajectory goShootGamepiece = ...;
     * AutoTrajectory pickupAnotherGamepiece = ...;
     *
     * routine.enabled().onTrue(rushMidTraj.cmd());
     *
     * rushMidTraj.done().and(noGamepiece).onTrue(pickupAnotherGamepiece.cmd());
     * rushMidTraj.done().and(hasGamepiece).onTrue(goShootGamepiece.cmd());
     *
     * // If done never falls when a new trajectory is scheduled
     * // then these triggers leak into the next trajectory, causing the next note pickup
     * // to trigger goShootGamepiece.cmd() even if we no longer care about these checks
     * </code></pre>
     *
     * @return A trigger that is true when the trajectory is finished.
     */
    public Trigger done() {
        return doneDelayed(0);
    }

    /**
     * Returns a trigger that stays true for a number of cycles after the trajectory ends.
     *
     * @param seconds Seconds to stay true after the trajectory ends.
     * @return A trigger that stays true for a number of cycles after the trajectory ends.
     */
    public Trigger doneFor(double seconds) {
        return enterExitTrigger(doneDelayed(0), doneDelayed(seconds));
    }

    /**
     * A shorthand for `.done().onTrue(otherTrajectory.cmd())`
     *
     * @param otherTrajectory The other trajectory to run when this one is done.
     */
    public void chain(AutoTrajectory otherTrajectory) {
        done().onTrue(otherTrajectory.cmd());
    }

    /**
     * Returns a trigger that will go true for 1 cycle when the desired time has elapsed
     *
     * @param timeSinceStart The time since the command started in seconds.
     * @return A trigger that is true when timeSinceStart has elapsed.
     */
    public Trigger atTime(double timeSinceStart) {
        // The timer should never be negative so report this as a warning
        if (timeSinceStart < 0 || timeSinceStart > trajectory.getTotalTimeSeconds()) {
            return new Trigger(() -> false);
        }

        return mAuto.loggedCondition(
            path.name+"/AtTime/"+timeSinceStart,
            timeTrigger(timeSinceStart, timer),
            false,
            loop);
    }

    /**
     * Returns a trigger that will go true for 1 cycle when the desired before the end of the
     * trajectory time.
     *
     * @param timeBeforeEnd The time before the end of the trajectory.
     * @return A trigger that is true when timeBeforeEnd has elapsed.
     */
    public Trigger atTimeBeforeEnd(double timeBeforeEnd) {
        return atTime(trajectory.getTotalTimeSeconds() - timeBeforeEnd);
    }

    private boolean withinTolerance(Rotation2d lhs, Rotation2d rhs, double toleranceRadians) {
        if (Math.abs(toleranceRadians) > Math.PI) {
            return true;
        }
        double dot = lhs.getCos() * rhs.getCos() + lhs.getSin() * rhs.getSin();
        // cos(θ) >= cos(tolerance) means |θ| <= tolerance, for tolerance in [-pi, pi], as pre-checked
        // above.
        return dot > Math.cos(toleranceRadians);
    }

    /**
     * Returns a trigger that is true when the robot is within toleranceMeters of the given pose.
     *
     * <p>The pose is flipped if alliance flipping is enabled and the alliance supplier returns Red.
     *
     * <p>While alliance flipping is enabled and the alliance supplier returns empty, the trigger will
     * return false.
     *
     * @param pose The pose to check against, unflipped.
     * @param toleranceMeters The tolerance in meters.
     * @param toleranceRadians The heading tolerance in radians.
     * @return A trigger that is true when the robot is within toleranceMeters of the given pose.
     */
    public Trigger atPose(Pose2d pose, double toleranceMeters, double toleranceRadians) {
        return mAuto.loggedCondition(
            path.name+"/AtPose/X: "+pose.getX()+"Y: "+pose.getY()+"ThetaDeg: "+pose.getRotation().getDegrees(),
            new Trigger(
            loop,
            () -> {
                  boolean transValid =
                      poseSupplier.get().getTranslation().getDistance(AllianceFlipUtil.apply(pose.getTranslation()))
                          < toleranceMeters;
                  boolean rotValid =
                      withinTolerance(
                          poseSupplier.get().getRotation(), AllianceFlipUtil.apply(pose.getRotation()), toleranceRadians);
                  return transValid && rotValid;
            }).and(isRunning()),
            true,
            loop);
    }

    /**
     * Returns a trigger that is true when the robot is within toleranceMeters of the given
     * translation.
     *
     * <p>The translation is flipped if alliance flipping is enabled and the alliance supplier returns
     * Red.
     *
     * <p>While alliance flipping is enabled and the alliance supplier returns empty, the trigger will
     * return false.
     *
     * @param translation The translation to check against, unflipped.
     * @param toleranceMeters The tolerance in meters.
     * @return A trigger that is true when the robot is within toleranceMeters of the given
     *     translation.
     */
    public Trigger atTranslation(Translation2d translation, double toleranceMeters) {
        return mAuto.loggedCondition(
            path.name+"/AtTranslation/X: "+translation.getX()+"Y: "+translation.getY(),
            new Trigger(
                loop,
                () -> {
                    return poseSupplier.get().getTranslation().getDistance(AllianceFlipUtil.apply(translation)) < toleranceMeters;
                })
            .and(isRunning()),
            true,
            loop);
    }

    public Trigger isRunning() {
        return new Trigger(loop, () -> mIsRunning);
    }

    public Trigger hasEnded() {
        return new Trigger(loop, () -> mHasEnded);
    }

   /**
    * Create a command to warmup on-the-fly generation, replanning, and the path following command
    *
    * @return Path following warmup command
    */
    public static Command warmupCommand() {
        List<Waypoint> waypoints =
            PathPlannerPath.waypointsFromPoses(
                new Pose2d(0.0, 0.0, Rotation2d.kZero), new Pose2d(6.0, 6.0, Rotation2d.kZero));
        PathPlannerPath path =
            new PathPlannerPath(
                waypoints,
                new PathConstraints(4.0, 4.0, 4.0, 4.0),
                new IdealStartingState(0.0, Rotation2d.kZero),
                new GoalEndState(0.0, Rotation2d.kCW_90deg));

        return new FollowPathCommand(
            path,
            () -> Pose2d.kZero,
            ChassisSpeeds::new,
            (speeds, feedforwards) -> {},
            new PPHolonomicDriveController(
                new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
            new RobotConfig(
                75,
                6.8,
                new ModuleConfig(
                    0.048, 5.0, 1.2, DCMotor.getKrakenX60(1).withReduction(6.14), 60.0, 1),
                0.55),
            () -> true, 
            false, 
            (pose) -> {})
        .andThen(Commands.print("[PathPlanner] FollowPathCommand finished warmup"))
        .ignoringDisable(true);
    }
}

