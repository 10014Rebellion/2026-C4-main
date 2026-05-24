package frc.lib.controls;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.math.EqualsUtil;

public class TurnPointFeedforward {
    private Supplier<Pose2d> mRobotFieldPose;
    private Supplier<ChassisSpeeds> mRobotFieldSpeeds;

    private Supplier<Pose2d> mPointFieldPose;
    private Supplier<ChassisSpeeds> mPointFieldSpeeds;

    public TurnPointFeedforward
        (
            Supplier<Pose2d> pRobotFieldPose, 
            Supplier<ChassisSpeeds> pRobotFieldSpeeds, 
            Supplier<Pose2d> pPointFieldPose, 
            Supplier<ChassisSpeeds> pPointFieldSpeeds) {

        mRobotFieldPose = pRobotFieldPose;
        mRobotFieldSpeeds = pRobotFieldSpeeds;

        mPointFieldPose = pPointFieldPose;
        mPointFieldSpeeds = pPointFieldSpeeds;
    }

    /**
     * Computes omega feedforward to oppose change in rotation caused by robot translating relative to the point
     * 
     */
    public Rotation2d computeOmegaFeedforward() {
        double deltaX = mPointFieldPose.get().getX() - mRobotFieldPose.get().getX();
        double deltaY = mPointFieldPose.get().getY() - mRobotFieldPose.get().getY();

        double deltaVX = mPointFieldSpeeds.get().vxMetersPerSecond - mRobotFieldSpeeds.get().vxMetersPerSecond;
        double deltaVY = mPointFieldSpeeds.get().vyMetersPerSecond - mRobotFieldSpeeds.get().vyMetersPerSecond;

        // Equation derived using tan(theta) = (xPoint - xRobot) / (yPoint - yRobot)
        if(EqualsUtil.epsilonEquals(deltaX, 0.0) && EqualsUtil.epsilonEquals(deltaY, 0.0)) return Rotation2d.kZero;

        double omega = 
            (deltaVY * deltaX - deltaY * deltaVX) 
                /
            (deltaX * deltaX + deltaY * deltaY);

        // Inverted to oppose change of goal
        return Rotation2d.fromRadians(-omega);
    }
    
    public static TurnPointFeedforward zeroTurnPointFF() {
        return new TurnPointFeedforward(
            () -> new Pose2d(), 
            () -> new ChassisSpeeds(), 
            () -> new Pose2d(), 
            () -> new ChassisSpeeds());
    }
}
