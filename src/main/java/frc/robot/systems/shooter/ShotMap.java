package frc.robot.systems.shooter;

import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.game.GameGoalPoseChooser;

public class ShotMap {
    private static ShotMap mInstance;
    
    public static ShotMap getInstance() {
        if(mInstance == null) {
            mInstance = new ShotMap();
        }
        return mInstance;
    }

    private Supplier<Pose2d> robotPose = () -> new Pose2d();
    public double offsetM = 0.0;
    public double velocityRPSOffset = -3.8; // negative = slowed down (weaker). positive = sped up (farther)

    public record ShotMapSetpoint(double distance, Rotation2d pHoodSetpoint, Rotation2d flywheelSpeedPS) {}

    public ShotMapSetpoint[] setpoints = new ShotMapSetpoint[] {
        new ShotMapSetpoint(1.551, Rotation2d.fromDegrees(11.0), Rotation2d.fromRotations(41.5)),
        new ShotMapSetpoint(2.124, Rotation2d.fromDegrees(12.0), Rotation2d.fromRotations(44.0)),
        new ShotMapSetpoint(2.624, Rotation2d.fromDegrees(12.5), Rotation2d.fromRotations(45.5)),
        new ShotMapSetpoint(3.124, Rotation2d.fromDegrees(14.5), Rotation2d.fromRotations(47.5)),
        new ShotMapSetpoint(3.46, Rotation2d.fromDegrees(14.0), Rotation2d.fromRotations(49.5)),
        new ShotMapSetpoint(3.973, Rotation2d.fromDegrees(15.0), Rotation2d.fromRotations(51.5)),
        new ShotMapSetpoint(4.42, Rotation2d.fromDegrees(15.0), Rotation2d.fromRotations(52.5)),
        new ShotMapSetpoint(5.332, Rotation2d.fromDegrees(17.0), Rotation2d.fromRotations(56))
        // new ShotMapSetpoint(5.05, Rotation2d.fromDegrees(16.0), Rotation2d.fromRotations(65.5 + velocityRPSOffset))
    };

    public InterpolatingDoubleTreeMap distanceVelRPSMap = new InterpolatingDoubleTreeMap();
    public InterpolatingDoubleTreeMap distanceAngleDegMap = new InterpolatingDoubleTreeMap();

    private ShotMap() {
        for(int i = 0; i < setpoints.length; i++) {
            distanceVelRPSMap.put(setpoints[i].distance(), setpoints[i].flywheelSpeedPS.getRotations());
        }

        for(int i = 0; i < setpoints.length; i++) {
            distanceAngleDegMap.put(setpoints[i].distance(), setpoints[i].pHoodSetpoint().getDegrees());
        }

        CommandScheduler.getInstance().getActiveButtonLoop()
            .bind(() -> {
                Logger.recordOutput("ShotMap/DistanceToHub", distanceToHub());
                Logger.recordOutput("ShotMap/AngleOfHood", getHoodAngle());
                Logger.recordOutput("ShotMap/FlywheelRPS", getFlywheelVel());
            });
    }

    public double distanceToHub() {
        return 
        Math.max(setpoints[0].distance() + 0.01,
            Math.min(
                setpoints[setpoints.length - 1].distance() - 0.01,
                Units.inchesToMeters(5.5) + Math.hypot(
                    robotPose.get().getX() - GameGoalPoseChooser.getHub().getX(),
                    robotPose.get().getY() - GameGoalPoseChooser.getHub().getY()
            ))
        );
    }

    public Rotation2d getHoodAngle() {
        return Rotation2d.fromDegrees(distanceAngleDegMap.get(distanceToHub()));
    }

    public Rotation2d getFlywheelVel() {
        return Rotation2d.fromRotations(distanceVelRPSMap.get(distanceToHub()));
    }

    public void setPoseSupplier(Supplier<Pose2d> pPoseSup) {
        robotPose = pPoseSup;
    }

}
