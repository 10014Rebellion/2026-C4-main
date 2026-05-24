// REBELLION 10014

package frc.robot.systems.drive.controllers;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.lib.math.GeomUtil;
import frc.lib.tuning.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;

public class HolonomicController {
    public static enum ConstraintType {
        AXIS,
        LINEAR
    }

    public static final LoggedTunableNumber tXP = new LoggedTunableNumber("AutoAlign/X/kP", 5.0);
    public static final LoggedTunableNumber tXD = new LoggedTunableNumber("AutoAlign/X/kD", 0.0);
    // public static final LoggedTunableNumber tXI = new LoggedTunableNumber("AutoAlign/X/kI", 0.0);
    // public static final LoggedTunableNumber tXIZone = new LoggedTunableNumber("AutoAlign/X/kIZone", 0.0);
    // public static final LoggedTunableNumber tXIRange = new LoggedTunableNumber("AutoAlign/X/kIRange", 0.0);
    public static final LoggedTunableNumber tXMaxVMPS = new LoggedTunableNumber("AutoAlign/X/kMaxVMPS", 3.4);
    public static final LoggedTunableNumber tXMaxAMPSS = new LoggedTunableNumber("AutoAlign/X/kMaxVMPSS", 10.0);

    public static final LoggedTunableNumber tXS = new LoggedTunableNumber("AutoAlign/X/kS", 0.0);
    public static final LoggedTunableNumber tXV = new LoggedTunableNumber("AutoAlign/X/kV", 0.7);

    public static final LoggedTunableNumber tXToleranceMeters = new LoggedTunableNumber("AutoAlign/X/ToleranceMeters", 0.05);

    public static final LoggedTunableNumber tYP = new LoggedTunableNumber("AutoAlign/Y/kP", 5.0);
    public static final LoggedTunableNumber tYD = new LoggedTunableNumber("AutoAlign/Y/kD", 0.0);
    // public static final LoggedTunableNumber tYI = new LoggedTunableNumber("AutoAlign/Y/kI", 0.0);
    // public static final LoggedTunableNumber tYIZone = new LoggedTunableNumber("AutoAlign/Y/kIZone", 0.0);
    // public static final LoggedTunableNumber tYIRange = new LoggedTunableNumber("AutoAlign/Y/kIRange", 0.0);
    public static final LoggedTunableNumber tYMaxVMPS = new LoggedTunableNumber("AutoAlign/Y/kMaxVMPS", 3.4);
    public static final LoggedTunableNumber tYMaxAMPSS = new LoggedTunableNumber("AutoAlign/Y/kMaxVMPSS", 10.0);

    public static final LoggedTunableNumber tYS = new LoggedTunableNumber("AutoAlign/Y/kS", 0.0);
    public static final LoggedTunableNumber tYV = new LoggedTunableNumber("AutoAlign/Y/kV", 0.7);

    public static final LoggedTunableNumber tYToleranceMeters = new LoggedTunableNumber("AutoAlign/Y/ToleranceMeters", 0.05);

    public static final LoggedTunableNumber tOmegaP = new LoggedTunableNumber("AutoAlign/Omega/kP", 3.5);
    public static final LoggedTunableNumber tOmegaD = new LoggedTunableNumber("AutoAlign/Omega/kD", 0.0);

    // public static final LoggedTunableNumber tOmegaI = new LoggedTunableNumber("AutoAlign/Omega/kI", 0.0);
    // public static final LoggedTunableNumber tOmegaIZone = new LoggedTunableNumber("AutoAlign/Omega/kIZone", 0.0);
    // public static final LoggedTunableNumber tOmegaIRange = new LoggedTunableNumber("AutoAlign/Omega/kIRange", 0.0);

    public static final LoggedTunableNumber tOmegaMaxVDPS = new LoggedTunableNumber("AutoAlign/Omega/kMaxVDPS", 180);
    public static final LoggedTunableNumber tOmegaMaxADPSS = new LoggedTunableNumber("AutoAlign/Omega/kMaxVDPSS", 360);

    public static final LoggedTunableNumber tOmegaS = new LoggedTunableNumber("AutoAlign/Omega/kS", 0.0);
    public static final LoggedTunableNumber tOmegaV = new LoggedTunableNumber("AutoAlign/Omega/kV", 0.25);

    public static final LoggedTunableNumber tOmegaToleranceDegrees = new LoggedTunableNumber("AutoAlign/Omega/ToleranceDegrees", 5.0);
    public static final LoggedTunableNumber tDistanceMaxVMPS = new LoggedTunableNumber("AutoAlign/Distance/kMaxVMPS", 3.4);
    public static final LoggedTunableNumber tDistanceMaxAMPSS = new LoggedTunableNumber("AutoAlign/Distance/kMaxVMPSS", 13.0);

    public static final LoggedTunableNumber tDistanceToleranceMeters = new LoggedTunableNumber("AutoAlign/Distance/ToleranceMeters", 0.05);

    public static final LoggedTunableNumber tFFRadius = new LoggedTunableNumber("AutoAlign/ffRadius", 0.25);

    private ProfiledPIDController tXController;
    private ProfiledPIDController tYController;
    private ProfiledPIDController tOmegaController;

    private SimpleMotorFeedforward tXFeedforward;
    private SimpleMotorFeedforward tYFeedforward;
    private SimpleMotorFeedforward tOmegaFeedforward;

    private ConstraintType tType = ConstraintType.AXIS;

    private LinearFilter xValueFilter = LinearFilter.movingAverage(2);
    private LinearFilter yValueFilter = LinearFilter.movingAverage(2);

    public HolonomicController() {
        this.tXController = new ProfiledPIDController(
            tXP.get(), 0.0, tXD.get(), new Constraints(tXMaxVMPS.get(), tXMaxAMPSS.get()));
        tXController.setIntegratorRange(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
        tXController.setIZone(0.0);
        tXController.setConstraints(new Constraints(tXMaxVMPS.get(), tXMaxAMPSS.get()));
        tXController.setTolerance(tXToleranceMeters.get());
        this.tXFeedforward = new SimpleMotorFeedforward(tXS.get(), tXV.get());

        this.tYController = new ProfiledPIDController(
            tYP.get(), 0.0, tYD.get(), new Constraints(tYMaxVMPS.get(), tYMaxAMPSS.get()));
        tYController.setIntegratorRange(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
        tYController.setIZone(0.0);
        tYController.setConstraints(new Constraints(tYMaxVMPS.get(), tYMaxAMPSS.get()));
        tYController.setTolerance(tYToleranceMeters.get());
        this.tYFeedforward = new SimpleMotorFeedforward(tYS.get(), tYV.get());

        this.tOmegaController = new ProfiledPIDController(
            tOmegaP.get(), 0.0, tOmegaD.get(), new Constraints(tOmegaMaxVDPS.get(), tOmegaMaxADPSS.get()));
        tOmegaController.enableContinuousInput(-180.0, 180.0);
        tOmegaController.setIntegratorRange(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
        tOmegaController.setIZone(0.0);
        tOmegaController.setConstraints(new Constraints(tOmegaMaxVDPS.get(), tOmegaMaxADPSS.get()));
        tOmegaController.setTolerance(tOmegaToleranceDegrees.get());
        this.tOmegaFeedforward = new SimpleMotorFeedforward(tOmegaS.get(), tOmegaV.get());
    }

    /* WHEN USING LINEAR CONSTRAINT, GOAL */
    public void setConstraintType(ConstraintType pType) {
        this.tType = pType;
    }

    public void reset(Pose2d pStartPose, Pose2d pGoalPose) {
        reset(pStartPose, new ChassisSpeeds(), pGoalPose);
    }

    /*
     * Resets the robot based on the position and the speed of the robot
     * Resetting with the speed allows the robot to stay controlled
     * while the driver is moving before drive to pose is activated
     * GOAL POSE IS ONLY UTLIIZED WHEN IN LINEAR CONSTRAINT
     */
    public void reset(Pose2d pRobotPose, ChassisSpeeds pRobotChassisSpeeds, Pose2d pGoalPose) {
        if (tType.equals(ConstraintType.LINEAR)) {
            Rotation2d heading =
                new Rotation2d(pGoalPose.getX() - pRobotPose.getX(), pGoalPose.getY() - pRobotPose.getY());

            // Telemetry.log("AutoAlign/Linear/X/Vel", distanceMaxVMPS.get() *  heading.getCos());
            // Telemetry.log("AutoAlign/Linear/X/Accel", distanceMaxVMPS.get() *  heading.getCos());

            // Telemetry.log("AutoAlign/Linear/Y/Vel", distanceMaxVMPS.get() *  heading.getCos());
            // Telemetry.log("AutoAlign/Linear/Y/Accel", distanceMaxVMPS.get() *  heading.getCos());

            tXController.setConstraints(new TrapezoidProfile.Constraints(
                tDistanceMaxVMPS.get() * Math.abs(heading.getCos()),
                tDistanceMaxAMPSS.get() * Math.abs(heading.getCos())));

            tYController.setConstraints(new TrapezoidProfile.Constraints(
                tDistanceMaxVMPS.get() * Math.abs(heading.getSin()),
                tDistanceMaxAMPSS.get() * Math.abs(heading.getSin())));
        } else {
            tXController.setConstraints(new TrapezoidProfile.Constraints(tXMaxVMPS.get(), tXMaxAMPSS.get()));
            tYController.setConstraints(new TrapezoidProfile.Constraints(tYMaxVMPS.get(), tYMaxAMPSS.get()));
        }

        tXController.reset(new State(pRobotPose.getX(), pRobotChassisSpeeds.vxMetersPerSecond));
        tYController.reset(new State(pRobotPose.getY(), pRobotChassisSpeeds.vyMetersPerSecond));
        tOmegaController.reset(new State(pRobotPose.getRotation().getDegrees(), 0.0));
    }

    public ChassisSpeeds calculate(Pose2d pGoalPose, Pose2d pCurrentPose) {
        return calculate(pGoalPose, new ChassisSpeeds(), pCurrentPose);
    }

    /* Uses 3 PID controllers to set the chassis speeds */
    public ChassisSpeeds calculate(Pose2d pGoalPose, ChassisSpeeds pGoalSpeed, Pose2d pCurrentPose) {
        double ffScalar = Math.min(
                Math.hypot(pGoalPose.getX() - pCurrentPose.getX(), pGoalPose.getY() - pCurrentPose.getY()) / tFFRadius.get(),
                1.0);
        Pose2d currentPose = filterTranslationOfPose(pCurrentPose);

        // if(pGoalPose.minus(currentPose).getTranslation().getNorm() < 0.03) return new ChassisSpeeds(
        //     0.0, 0.0,
        //     (Math.toRadians(tOmegaController.calculate(
        //         currentPose.getRotation().getDegrees(),
        //         new TrapezoidProfile.State(
        //             pGoalPose.getRotation().getDegrees(),
        //             Math.toDegrees(pGoalSpeed.omegaRadiansPerSecond)))
        //     + tOmegaFeedforward.calculate(tOmegaController.getSetpoint().velocity)))
        // );

        return ChassisSpeeds.fromFieldRelativeSpeeds(
            (tXController.calculate(
                currentPose.getX(),
                new TrapezoidProfile.State(pGoalPose.getX(), pGoalSpeed.vxMetersPerSecond))
            + ffScalar * tXFeedforward.calculate(tXController.getSetpoint().velocity)),
            (tYController.calculate(
                currentPose.getY(),
                    new TrapezoidProfile.State(pGoalPose.getY(), pGoalSpeed.vyMetersPerSecond))
            + ffScalar * tYFeedforward.calculate(tYController.getSetpoint().velocity)),
            (Math.toRadians(tOmegaController.calculate(
                currentPose.getRotation().getDegrees(),
                new TrapezoidProfile.State(
                    pGoalPose.getRotation().getDegrees(),
                    Math.toDegrees(pGoalSpeed.omegaRadiansPerSecond)))
            + tOmegaFeedforward.calculate(tOmegaController.getSetpoint().velocity))),
            currentPose.getRotation());
    }

    ////////////////////////// GETTERS \\\\\\\\\\\\\\\\\\\\\\\\\\\\
    @AutoLogOutput(key = "Drive/HolonomicController/AtGoal")
    public boolean atGoal() {
        return tXController.atGoal() && tYController.atGoal() && tOmegaController.atGoal();
    }

    public boolean inTolerance(Transform2d transform, Pose2d robotPose) {
        return 
            (Math.abs(tXController.getGoal().position - robotPose.getX()) < transform.getX())
              && 
            (Math.abs(tYController.getGoal().position - robotPose.getY()) < transform.getY())
              &&
            GeomUtil.withinTolerance(
                robotPose.getRotation(), 
                Rotation2d.fromDegrees(tOmegaController.getGoal().position), 
                transform.getRotation());
    }

    @AutoLogOutput(key = "Drive/HolonomicController/PositionGoal")
    public Pose2d getPositionGoal() {
        return new Pose2d(
                new Translation2d(tXController.getGoal().position, tYController.getGoal().position),
                Rotation2d.fromDegrees(tOmegaController.getGoal().position));
    }

    @AutoLogOutput(key = "Drive/HolonomicController/PositionSetpoint")
    public Pose2d getPositionSetpoint() {
        return new Pose2d(
                new Translation2d(tXController.getSetpoint().position, tYController.getSetpoint().position),
                Rotation2d.fromDegrees(tOmegaController.getSetpoint().position));
    }

    // @AutoLogOutput(key = "Drive/HolonomicController/AtPositionTimeout")
    public boolean atPositionTimeout() {
        return getPositionGoal().equals(getPositionSetpoint());
    }

    // @AutoLogOutput(key = "Drive/HolonomicController/VelocityGoal")
    public ChassisSpeeds getVelocityGoal() {
        return new ChassisSpeeds(
            tXController.getGoal().velocity,
            tYController.getGoal().velocity,
            Math.toRadians( tOmegaController.getGoal().velocity ) );
    }

    @AutoLogOutput(key = "Drive/HolonomicController/VelocitySetpoint")
    public ChassisSpeeds getVelocitySetpoint() {
        return new ChassisSpeeds(
                tXController.getSetpoint().velocity,
                tYController.getSetpoint().velocity,
                Math.toRadians(tOmegaController.getSetpoint().velocity));
    }

    // @AutoLogOutput(key = "Drive/HolonomicController/PoseError")
    public Pose2d getPoseError() {
        return new Pose2d(
                tXController.getPositionError(),
                tYController.getPositionError(),
                new Rotation2d(tOmegaController.getPositionError()));
    }

    public Pose2d filterTranslationOfPose(Pose2d pose) {
        return new Pose2d(
            xValueFilter.calculate(pose.getX()),
            yValueFilter.calculate(pose.getY()),
            pose.getRotation()
        );
    }

    public double getLongestTime(Pose2d currentPose, ChassisSpeeds currentSpeed) {
        TrapezoidProfile xProfile = new TrapezoidProfile(tXController.getConstraints());
        TrapezoidProfile yProfile = new TrapezoidProfile(tYController.getConstraints());
        TrapezoidProfile thetaProfile = new TrapezoidProfile(tOmegaController.getConstraints());

        xProfile.calculate(
            0, 
            new TrapezoidProfile.State(
                currentPose.getX(), 
                currentSpeed.vxMetersPerSecond), 
                tXController.getGoal());

        yProfile.calculate(
            0, 
            new TrapezoidProfile.State(
                currentPose.getY(), 
                currentSpeed.vyMetersPerSecond), 
                tYController.getGoal());

        thetaProfile.calculate(
            0, 
            new TrapezoidProfile.State(
                currentPose.getRotation().getDegrees(), 
                Math.toDegrees(currentSpeed.omegaRadiansPerSecond)), 
                tOmegaController.getGoal());

        return Math.max(
            Math.max(xProfile.totalTime(), yProfile.totalTime()), thetaProfile.totalTime());
    }

    ////////////////////////// SETTERS \\\\\\\\\\\\\\\\\\\\\\\\\\\\
    public void updateControllers() {
        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> {
                    tXController.setPID(tXP.get(), 0.0, tXD.get());
                    if (tType.equals(ConstraintType.LINEAR))
                        tXController.setConstraints(new Constraints(tXMaxVMPS.get(), tXMaxAMPSS.get()));
                },
                tXP,
                tXD,
                tXMaxVMPS,
                tXMaxAMPSS);

        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> {
                    tXFeedforward = new SimpleMotorFeedforward(tXS.get(), tXV.get());
                },
                tXS,
                tXV);

        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> {
                    tYController.setPID(tYP.get(), 0.0, tYD.get());
                    if (tType.equals(ConstraintType.LINEAR))
                        tYController.setConstraints(new Constraints(tYMaxVMPS.get(), tYMaxAMPSS.get()));
                },
                tYP,
                tYD,
                tYMaxVMPS,
                tYMaxAMPSS);

        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> {
                    tYFeedforward = new SimpleMotorFeedforward(tYS.get(), tYV.get());
                },
                tYS,
                tYV);

        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> {
                    tOmegaController.setPID(tOmegaP.get(), 0.0, tOmegaD.get());
                    tOmegaController.setConstraints(new Constraints(tOmegaMaxVDPS.get(), tOmegaMaxADPSS.get()));
                },
                tOmegaP,
                tOmegaD,
                tOmegaMaxVDPS,
                tOmegaMaxADPSS);

        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> {
                    tOmegaFeedforward = new SimpleMotorFeedforward(tOmegaS.get(), tOmegaV.get());
                },
                tOmegaS,
                tOmegaV);

        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> {
                    tXController.setTolerance(tXToleranceMeters.get());
                    tYController.setTolerance(tYToleranceMeters.get());
                    tOmegaController.setTolerance(tOmegaToleranceDegrees.get());
                },
                tXToleranceMeters,
                tYToleranceMeters,
                tOmegaToleranceDegrees);
    }
}
