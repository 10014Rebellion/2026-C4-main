package frc.robot.systems.drive.controllers;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.lib.tuning.LoggedTunableNumber;

public class LineController {
    public static final LoggedTunableNumber tXP = new LoggedTunableNumber("LineAlign/X/kP", 3.0);
    public static final LoggedTunableNumber tXD = new LoggedTunableNumber("LineAlign/X/kD", 0.0);
    // public static final LoggedTunableNumber tXI = new LoggedTunableNumber("LineAlign/X/kI", 0.0);
    // public static final LoggedTunableNumber tXIZone = new LoggedTunableNumber("LineAlign/X/kIZone", 0.0);
    // public static final LoggedTunableNumber tXIRange = new LoggedTunableNumber("LineAlign/X/kIRange", 0.0);
    public static final LoggedTunableNumber tXMaxVMPS = new LoggedTunableNumber("LineAlign/X/kMaxVMPS", 3.5);
    public static final LoggedTunableNumber tXMaxAMPSS = new LoggedTunableNumber("LineAlign/X/kMaxVMPSS", 7.0);

    public static final LoggedTunableNumber tXS = new LoggedTunableNumber("LineAlign/X/kS", 0.0);
    public static final LoggedTunableNumber tXV = new LoggedTunableNumber("LineAlign/X/kV", 0.65);

    public static final LoggedTunableNumber tXToleranceMeters = new LoggedTunableNumber("LineAlign/X/ToleranceMeters", 0.03);

    public static final LoggedTunableNumber tOmegaP = new LoggedTunableNumber("LineAlign/Omega/kP", 2.5);
    public static final LoggedTunableNumber tOmegaD = new LoggedTunableNumber("LineAlign/Omega/kD", 0.0);

    // public static final LoggedTunableNumber tOmegaI = new LoggedTunableNumber("LineAlign/Omega/kI", 0.0);
    // public static final LoggedTunableNumber tOmegaIZone = new LoggedTunableNumber("LineAlign/Omega/kIZone", 0.0);
    // public static final LoggedTunableNumber tOmegaIRange = new LoggedTunableNumber("LineAlign/Omega/kIRange", 0.0);

    public static final LoggedTunableNumber tOmegaMaxVDPS = new LoggedTunableNumber("LineAlign/Omega/kMaxVDPS", 180);
    public static final LoggedTunableNumber tOmegaMaxADPSS = new LoggedTunableNumber("LineAlign/Omega/kMaxVDPSS", 360);

    public static final LoggedTunableNumber tOmegaS = new LoggedTunableNumber("LineAlign/Omega/kS", 0.0);
    public static final LoggedTunableNumber tOmegaV = new LoggedTunableNumber("LineAlign/Omega/kV", 0.25);

    public static final LoggedTunableNumber tOmegaToleranceDegrees = new LoggedTunableNumber("LineAlign/Omega/ToleranceDegrees", 1.5);

    public static final LoggedTunableNumber tFFRadius = new LoggedTunableNumber("LineAlign/ffRadius", 1.0);

    private ProfiledPIDController tXController;
    private SimpleMotorFeedforward tXFeedforward;

    private ProfiledPIDController tOmegaController;
    private SimpleMotorFeedforward tOmegaFeedforward;

    private DoubleSupplier mSlope;
    private DoubleSupplier mTeleopScalar;
    private BooleanSupplier mInvertTeleop;

    public LineController(DoubleSupplier pSlope, DoubleSupplier pTeleopScalar, BooleanSupplier pInvertTeleop) {
        mSlope = pSlope;
        mTeleopScalar = pTeleopScalar;
        mInvertTeleop = pInvertTeleop;

        this.tXController = new ProfiledPIDController(
        tXP.get(), 0.0, tXD.get(), new Constraints(tXMaxVMPS.get(), tXMaxAMPSS.get()));
        tXController.setIntegratorRange(0.0, 0.0);
        tXController.setIZone(0.0);
        tXController.setConstraints(new Constraints(tXMaxVMPS.get(), tXMaxAMPSS.get()));
        tXController.setTolerance(tXToleranceMeters.get());
        this.tXFeedforward = new SimpleMotorFeedforward(tXS.get(), tXV.get());

        this.tOmegaController = new ProfiledPIDController(
            tOmegaP.get(), 0.0, tOmegaD.get(), new Constraints(tOmegaMaxVDPS.get(), tOmegaMaxADPSS.get()));
        tOmegaController.enableContinuousInput(-180.0, 180.0);
        tOmegaController.setIntegratorRange(0.0, 0.0);
        tOmegaController.setIZone(0.0);
        tOmegaController.setConstraints(new Constraints(tOmegaMaxVDPS.get(), tOmegaMaxADPSS.get()));
        tOmegaController.setTolerance(tOmegaToleranceDegrees.get());
        this.tOmegaFeedforward = new SimpleMotorFeedforward(tOmegaS.get(), tOmegaV.get());
    }


    public ChassisSpeeds calculate(ChassisSpeeds teleopSpeeds, Pose2d goalPose, Pose2d robotPose) {
        double distance = getDistanceFromLine(
            mSlope.getAsDouble(), 
            goalPose.getX(), 
            goalPose.getY(),
            robotPose);

        Rotation2d lineDirection;
        if(Double.isNaN(mSlope.getAsDouble())) {
            lineDirection = Rotation2d.kCCW_Pi_2;
        } else {
            lineDirection = new Rotation2d(1.0, mSlope.getAsDouble());
        }

        double parallelControl = teleopSpeeds.vxMetersPerSecond;

        Rotation2d perpendicularLineDirection = lineDirection.minus(Rotation2d.fromDegrees(90));

        double ffScalar = Math.min(Math.abs(distance) / tFFRadius.get(), 1.0);

        double perpendicularControl = 
            tXController.calculate(distance, 0) 
                + 
            ffScalar * tXFeedforward.calculate(tXController.getSetpoint().velocity);

        double rotationalControl = (Math.toRadians(tOmegaController.calculate(
            robotPose.getRotation().getDegrees(),
            new TrapezoidProfile.State(
                goalPose.getRotation().getDegrees(),
                Math.toDegrees(0.0)))
            + tOmegaFeedforward.calculate(tOmegaController.getSetpoint().velocity)));

        return ChassisSpeeds.fromFieldRelativeSpeeds(
                teleopInverConstant() * mTeleopScalar.getAsDouble() * parallelControl * lineDirection.getCos() + perpendicularControl * perpendicularLineDirection.getCos(),
                teleopInverConstant() * mTeleopScalar.getAsDouble() * parallelControl * lineDirection.getSin() + perpendicularControl * perpendicularLineDirection.getSin(),
                rotationalControl,
                robotPose.getRotation());
    }

    private double getDistanceFromLine(double slope, double anchorX, double anchorY, Pose2d robotPose) {
        if(!Double.isNaN(slope) && !Double.isInfinite(slope)) {
            double m = slope;
            double b = - anchorX * m + anchorY;

            double x1 = robotPose.getX();
            double y1 = robotPose.getY();

            Pose2d goalPose = new Pose2d(
                ( x1 
                    + m * y1 
                    - m * b ) 
                    / 
                ( 1 + m * m),
                    
                ( m * x1 
                    + m * m * y1 
                    + b ) 
                    / 
                ( 1 + m * m),

                Rotation2d.kZero);

            double lineSignum = Math.signum(
                (m * x1 + b) 
                    - 
                y1);

            return lineSignum * goalPose.minus(robotPose).getTranslation().getNorm();
        } else {
            return anchorX - robotPose.getX();

        }       
    }

    public double teleopInverConstant() {
        return (mInvertTeleop.getAsBoolean()) ? -1.0 : 1.0;
    }

    /* Call after setting goal */
    public void reset(Pose2d pRobotPose, Pose2d goalPose) {
        reset(pRobotPose, goalPose, new ChassisSpeeds());
    }

    public void reset(Pose2d pRobotPose, Pose2d goalPose, ChassisSpeeds robotSpeeds) {
        double chassisVelMagnitudeMPS = Math.hypot(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond);
        double perpendicularSlope = - 1.0 / mSlope.getAsDouble();
        Rotation2d perpendicularLineDirection;
        if(Double.isNaN(perpendicularSlope)) {
            perpendicularLineDirection = Rotation2d.kCCW_Pi_2;
        } else {
            perpendicularLineDirection = new Rotation2d(1.0, mSlope.getAsDouble());
        }

        tXController.reset(new State(
            getDistanceFromLine(
                mSlope.getAsDouble(),  
                goalPose.getX(), 
                goalPose.getY(), 
                pRobotPose), 
                perpendicularLineDirection.getCos() * chassisVelMagnitudeMPS));

        tOmegaController.reset(new State(pRobotPose.getRotation().getDegrees(), 0.0));
    }

    @AutoLogOutput(key="Drive/LineController/atGoal")
    public boolean atGoal(){
        return tOmegaController.atGoal() && tXController.atGoal();
    }

    public void resetController() {
        tOmegaController.setGoal(-100);
        tXController.setGoal(-100);
    }

    public void setControllerGoalSettings(DoubleSupplier pTeleopScalar, DoubleSupplier pSlope, BooleanSupplier pInvertTeleop) {
        mTeleopScalar = pTeleopScalar;
        mSlope = pSlope;
        mInvertTeleop = pInvertTeleop;
    }

    public void updateControllers() {
        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> {
                    tXController.setPID(tXP.get(), 0.0, tXD.get());
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
                    tOmegaController.setTolerance(tOmegaToleranceDegrees.get());
                },
                tXToleranceMeters,
                tOmegaToleranceDegrees);
    }
}
