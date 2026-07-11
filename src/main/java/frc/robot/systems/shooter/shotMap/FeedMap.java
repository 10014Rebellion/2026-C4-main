package frc.robot.systems.shooter.shotMap;

import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.game.GameGoalPoseChooser;

public class FeedMap {
    private static FeedMap mInstance;
    
    public static FeedMap getInstance() {
        if(mInstance == null) {
            mInstance = new FeedMap();
        }
        return mInstance;
    }

    private Supplier<Pose2d> robotPose = () -> new Pose2d();
    public double offsetM = 0.0;
    public double velocityRPSOffset = 2.0; // negative = slowed down (weaker). positive = sped up (farther)

    public record ShotMapSetpoint(double distance, Rotation2d pHoodSetpoint, Rotation2d flywheelSpeedPS) {}
    public static final Rotation2d kRestingTrueAngle = Rotation2d.fromDegrees(7.5);
    public ShotMapSetpoint[] setpoints = new ShotMapSetpoint[] {
        new ShotMapSetpoint(0, Rotation2d.fromDegrees(27.5+5).minus(kRestingTrueAngle), Rotation2d.fromRotations(40.0)),
        new ShotMapSetpoint(4, Rotation2d.fromDegrees(27.5+5).minus(kRestingTrueAngle), Rotation2d.fromRotations(40.0)),
        new ShotMapSetpoint(6, Rotation2d.fromDegrees(27.5+5).minus(kRestingTrueAngle), Rotation2d.fromRotations(50.0)),
        new ShotMapSetpoint(8, Rotation2d.fromDegrees(27.5+5).minus(kRestingTrueAngle), Rotation2d.fromRotations(60.0))
        // new ShotMapSetpoint(5.05, Rotation2d.fromDegrees(16.0), Rotation2dc.fromRotations(65.5 + velocityRPSOffset))
    };

    public InterpolatingDoubleTreeMap distanceVelRPSMap = new InterpolatingDoubleTreeMap();
    public InterpolatingDoubleTreeMap distanceAngleDegMap = new InterpolatingDoubleTreeMap();

    private FeedMap() {
        for(int i = 0; i < setpoints.length; i++) {
            distanceVelRPSMap.put(setpoints[i].distance(), setpoints[i].flywheelSpeedPS.getRotations());
        }

        for(int i = 0; i < setpoints.length; i++) {
            distanceAngleDegMap.put(setpoints[i].distance(), setpoints[i].pHoodSetpoint().getDegrees());
        }

        CommandScheduler.getInstance().getActiveButtonLoop()
            .bind(() -> {
                Logger.recordOutput("FeedMap/DistanceToAllianceWall", distanceToAllianceWall());
                Logger.recordOutput("FeedMap/AngleOfHood", getHoodAngle());
                Logger.recordOutput("FeedMap/FlywheelRPS", getFlywheelVel());
            });
    }

    public double distanceToAllianceWall() {
        return 
        Math.max(setpoints[0].distance() + 0.01,
            Math.min(
                setpoints[setpoints.length - 1].distance() - 0.01,
                Units.inchesToMeters(5.5) +
                    Math.abs((robotPose.get().getX() - GameGoalPoseChooser.getWall().getX()))
            )
        );
    }

    public Rotation2d getHoodAngle() {
        return Rotation2d.fromDegrees(distanceAngleDegMap.get(distanceToAllianceWall()));
    }

    public Rotation2d getFlywheelVel() {
        return Rotation2d.fromRotations(distanceVelRPSMap.get(distanceToAllianceWall()));
    }

    public void setPoseSupplier(Supplier<Pose2d> pPoseSup) {
        robotPose = pPoseSup;
    }

}
