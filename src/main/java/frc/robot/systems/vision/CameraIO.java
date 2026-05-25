package frc.robot.systems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public interface CameraIO {
    @AutoLog
    public static class CameraIOInputs {
        public boolean iIsConnected = false;
        public boolean iHasTarget = false;
        public boolean iHasBeenUpdated = false;
        public double iPoseAmbiguity = 0.0;
        public double iLatestTimestamp = 0.0;
        public int iNumberOfTargets = 0;
        public int iSingleTagAprilTagID = 0;
        public Pose3d iLatestEstimatedRobotPose = new Pose3d();
        public double iYaw = 0.0;
        public double iPitch = 0.0;
        public double iArea = 0.0;

        public Transform3d[] iLatestTagTransforms = new Transform3d[] {};
        public double[] iLatestTagAmbiguities = new double[] {};
    }

    @AutoLog
    public static class CameraIOConfigInputs {
        public String iCamName = "";
        public Transform3d iCameraTransform = new Transform3d();
    }

    public default void updateInputs(CameraIOInputs pInputs) {}

}
