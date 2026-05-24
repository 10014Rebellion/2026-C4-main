// REBELLION 10014

package frc.robot.systems.apriltag;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.RobotConstants;
import frc.robot.game.FieldConstants;
import frc.robot.systems.apriltag.ATagVisionConstants.ATagCameraHardware;
import frc.robot.systems.apriltag.ATagVisionConstants.CameraSimConfigs;
import frc.robot.systems.apriltag.ATagVisionConstants.Orientation;

import org.littletonrobotics.junction.Logger;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import frc.lib.math.GeomUtil;

public class ATagCameraIOPV implements ATagCameraIO {
    private String mCamName;
    private PhotonCamera mPhotonCam;
    private PhotonPoseEstimator mPoseEstimator;
    private Transform3d mCameraTransform;
    private final Orientation mOrientation;

    private VisionSystemSim mVisionSim;
    private PhotonCameraSim mCameraSim;

    private AprilTagCameraIOConfigInputsAutoLogged mConfigInputs = new AprilTagCameraIOConfigInputsAutoLogged();

    public ATagCameraIOPV(ATagCameraHardware camHardware) {
        this(camHardware.name(), camHardware.camTransform(), camHardware.orientation());
    }

    public ATagCameraIOPV(String pName, Transform3d pCameraTransform, Orientation pOrientation) {
        this.mCamName = pName;
        this.mPhotonCam = new PhotonCamera(mCamName);
        this.mCameraTransform = pCameraTransform;
        this.mOrientation = pOrientation;

        mPoseEstimator = new PhotonPoseEstimator(FieldConstants.kApriltagLayout, pCameraTransform);

        if (RobotConstants.isSim()) setupSimulation();

        processConfigInputs();
    }

    @Override
    public void updateInputs(AprilTagIOInputs pInputs, Pose2d pLastRobotPose, Pose2d pSimOdomPose) {
        try {
            if (RobotConstants.isSim()) mVisionSim.update(pSimOdomPose);
            
            pInputs.iIsConnected = mPhotonCam.isConnected();
            List<PhotonPipelineResult> unreadResults = mPhotonCam.getAllUnreadResults();

            pInputs.iHasBeenUpdated = !unreadResults.isEmpty();
            if (!pInputs.iHasBeenUpdated) return;

            PhotonPipelineResult latestValidResult = unreadResults.get(unreadResults.size() - 1);

            pInputs.iHasTarget = !(latestValidResult == null || !latestValidResult.hasTargets());

            if(pInputs.iHasTarget) {
                PhotonTrackedTarget target = latestValidResult.getBestTarget();
                pInputs.iSingleTagAprilTagID = target.getFiducialId();
                pInputs.iPoseAmbiguity = target.getPoseAmbiguity();
                // pInputs.iYaw = target.getYaw();
                // pInputs.iPitch = target.getPitch();
                // pInputs.iArea = target.getArea();
                // pInputs.iLatencySeconds = latestValidResult.metadata.getLatencyMillis() / 1000.0;
                pInputs.iLatestTimestamp = latestValidResult.getTimestampSeconds();
            }

            Optional<EstimatedRobotPose> latestEstimatedRobotPose = mPoseEstimator.estimateCoprocMultiTagPose(latestValidResult);

            if (latestEstimatedRobotPose.isEmpty()){
                latestEstimatedRobotPose = mPoseEstimator.estimateClosestToReferencePose(
                    latestValidResult, GeomUtil.toPose3d(pLastRobotPose));
            }

            latestEstimatedRobotPose.ifPresent(est -> {
                if (mOrientation == Orientation.BACK)
                    pInputs.iLatestEstimatedRobotPose = est.estimatedPose.transformBy(
                            new Transform3d(new Translation3d(), new Rotation3d(0.0, 0.0, Math.PI)));
                else pInputs.iLatestEstimatedRobotPose = est.estimatedPose;

                int count = est.targetsUsed.size();
                Transform3d[] tagTransforms = new Transform3d[count];
                double[] ambiguities = new double[count];

                for (int i = 0; i < count; i++) {
                    tagTransforms[i] = est.targetsUsed.get(i).getBestCameraToTarget();
                    ambiguities[i] = est.targetsUsed.get(i).getPoseAmbiguity();
                }

                pInputs.iNumberOfTargets = count;
                pInputs.iLatestTagTransforms = tagTransforms;
                pInputs.iLatestTagAmbiguities = ambiguities;
            });

        } catch (Exception e) {
            e.printStackTrace();
            resetInputs(pInputs);
        }
    }

    @Override
    public AprilTagCameraIOConfigInputs getConfigInputs() {
        return mConfigInputs;
    }

    private void setupSimulation() {
        mVisionSim = new VisionSystemSim("main");
        mVisionSim.addAprilTags(FieldConstants.kApriltagLayout);

        SimCameraProperties cameraProps = new SimCameraProperties();
        cameraProps.setCalibration(
                (int) CameraSimConfigs.resWidth.value,
                (int) CameraSimConfigs.resHeight.value,
                new Rotation2d(CameraSimConfigs.fovDeg.value));
        cameraProps.setCalibError(CameraSimConfigs.avgErrorPx.value, CameraSimConfigs.errorStdDevPx.value);
        cameraProps.setFPS(CameraSimConfigs.fps.value);
        cameraProps.setAvgLatencyMs(CameraSimConfigs.avgLatencyMs.value);
        cameraProps.setLatencyStdDevMs(CameraSimConfigs.latencyStdDevMs.value);

        mCameraSim = new PhotonCameraSim(mPhotonCam, cameraProps);
        mVisionSim.addCamera(mCameraSim, mCameraTransform);
    }

    private void processConfigInputs() {
        mConfigInputs.iCamName = mCamName;
        mConfigInputs.iCameraTransform = mCameraTransform;

        Logger.processInputs("Vision/"+mCamName+"/ConfigInputs", mConfigInputs);
    }

    private void resetInputs(AprilTagIOInputs pInputs) {
        pInputs.iIsConnected = false;
        pInputs.iHasTarget = false;
        pInputs.iHasBeenUpdated = false;
        // pInputs.iYaw = 0.0;
        // pInputs.iPitch = 0.0;
        // pInputs.iArea = 0.0;
        // pInputs.iLatencySeconds = 0.0;
        pInputs.iPoseAmbiguity = 0.0;
        pInputs.iSingleTagAprilTagID = 0;
        pInputs.iNumberOfTargets = 0;
        pInputs.iLatestTimestamp = 0.0;
        pInputs.iLatestEstimatedRobotPose = new Pose3d();
        pInputs.iLatestTagTransforms = new Transform3d[0];
        pInputs.iLatestTagAmbiguities = new double[0];
    }
}
