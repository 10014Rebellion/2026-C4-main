// REBELLION 10014

package frc.robot.systems.drive.controllers;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.lib.controls.TurnPointFeedforward;
import frc.lib.math.GeomUtil;
import frc.lib.telemetry.Telemetry;
import frc.lib.tuning.LoggedTunableNumber;
import frc.robot.systems.drive.DriveConstants;

import java.util.function.Supplier;

public class HeadingController {
    public static final LoggedTunableNumber mSnapP = new LoggedTunableNumber("SwerveHeadingController/Snap/kP", 2.5);
    public static final LoggedTunableNumber mSnapD = new LoggedTunableNumber("SwerveHeadingController/Snap/kD", 0.0);
    public static final LoggedTunableNumber mSnapMaxVDPS =
            new LoggedTunableNumber("SwerveHeadingController/Snap/kMaxV", 600.0);
    public static final LoggedTunableNumber mSnapMaxADPSS =
            new LoggedTunableNumber("SwerveHeadingController/Snap/kMaxA", 12000.0);

    public static final LoggedTunableNumber tKv = new LoggedTunableNumber("SwerveHeadingController/Snap/kV", 0.5);

    // public static final LoggedTunableNumber stablizingP =
    //     new LoggedTunableNumber("SwerveHeadingController/Stabilizing/kP", 2.5);
    // public static final LoggedTunableNumber stablizingI =
    //     new LoggedTunableNumber("SwerveHeadingController/Stabilizing/kI", 0.0);
    // public static final LoggedTunableNumber stablizingD =
    //     new LoggedTunableNumber("SwerveHeadingController/Stabilizing/kD", 0.0);

    public static final LoggedTunableNumber mToleranceDegrees =
            new LoggedTunableNumber("SwerveHeadingController/ToleranceDeg", 0.5);

    private ProfiledPIDController mSnapController;
    private TurnPointFeedforward mTurnPointFF;

    // private PIDController mStabilizingController;

    private Supplier<Rotation2d> mGoal;

    public HeadingController(TurnPointFeedforward pTurnPointFF) {
        mSnapController = new ProfiledPIDController(
                mSnapP.get(),
                0.0,
                mSnapD.get(),
                new TrapezoidProfile.Constraints(mSnapMaxVDPS.get(), mSnapMaxADPSS.get()));

        mSnapController.enableContinuousInput(0, 360);
        mSnapController.setTolerance(5.0);

        setTurnPointFF(pTurnPointFF);

        // stabilizingController =
        //     new PIDController(stablizingP.get(), stablizingI.get(), stablizingD.get());
        // stabilizingController.enableContinuousInput(0, 360);
        // stabilizingController.setTolerance(0.0);
    }

    public void setHeadingGoal(Supplier<Rotation2d> pGoalSupplier) {
        mGoal = pGoalSupplier;
    }

    public void reset(Rotation2d pRobotRotation, Rotation2d pRobotRotationPerSecond) {
        mSnapController.reset(pRobotRotation.getDegrees(), pRobotRotationPerSecond.getDegrees());
    }

    // Designed for large jumps toward a setpoint, kind of like an azimuth alignment
    public double getSnapOutputRadians(Rotation2d pRobotRotation) {
        Telemetry.log(
            "Drive/HeadingController/HeadingSetpoint",
            Rotation2d.fromDegrees(mSnapController.getSetpoint().position));

        Rotation2d pidOutput = Rotation2d.fromDegrees(mSnapController.calculate(pRobotRotation.getDegrees(), mGoal.get().getDegrees()));
        Rotation2d profileFFOutput = Rotation2d.fromDegrees(mSnapController.getSetpoint().velocity);
        Rotation2d turnPointFFOutput = mTurnPointFF.computeOmegaFeedforward();

        double outputRadians = pidOutput.getRadians() + tKv.get() * profileFFOutput.getRadians() + turnPointFFOutput.getRadians();

        Rotation2d setpointError = Rotation2d.fromDegrees( mSnapController.getSetpoint().position - pRobotRotation.getDegrees() );
        Rotation2d goalError = Rotation2d.fromDegrees( mSnapController.getGoal().position - pRobotRotation.getDegrees() );

        double adjustedOutputRadians = outputRadians;
        if (Math.abs(goalError.getDegrees()) < mToleranceDegrees.get()) adjustedOutputRadians *= 0.0;

        Telemetry.log("Drive/HeadingController/setpointError", setpointError);
        Telemetry.log("Drive/HeadingController/goalError", goalError);

        if(DriveConstants.kDoExtraLogging) {
            Telemetry.log("Drive/HeadingController/unAdjustedOutput", outputRadians);
            Telemetry.log("Drive/HeadingController/adjustedOutput", adjustedOutputRadians);
            Telemetry.log("Drive/HeadingController/pidOutput", pidOutput);
            Telemetry.log("Drive/HeadingController/profileFFOutput", profileFFOutput);
            Telemetry.log("Drive/HeadingController/turnPointFFOutput", turnPointFFOutput);
        }

        return adjustedOutputRadians;
    }

    // Designed for shoot on move and short distance. In most cases velocityDPS is 0.
    // public double getStabilizingOutput(Rotation2d pRobotRotation, double pVelocityDPS) {
    //     return Math.toRadians(mStabilizingController.calculate(
    //                     pRobotRotation.getDegrees(), mGoal.get().getDegrees())
    //             + pVelocityDPS);
    // }

    public void updateController() {
        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> {
                    mSnapController.setPID(mSnapP.get(), 0.0, mSnapD.get());
                    mSnapController.setConstraints(new Constraints(mSnapMaxVDPS.get(), mSnapMaxADPSS.get()));
                },
                mSnapP,
                mSnapD,
                mSnapMaxVDPS,
                mSnapMaxADPSS);

        // LoggedTunableNumber.ifChanged(hashCode(), () -> {
        //     stabilizingController.setPID(stablizingP.get(), stablizingI.get(), stablizingD.get());
        // }, stablizingP, stablizingI, stablizingD);
    }

    public void setTurnPointFF(TurnPointFeedforward pTurnPointFF) {
        mTurnPointFF = pTurnPointFF;
    }

    public boolean inTolerance(Rotation2d robotRotation, Rotation2d tolerance) {
        return GeomUtil.withinTolerance(Rotation2d.fromDegrees(mSnapController.getGoal().position), robotRotation, tolerance);
    }
}
