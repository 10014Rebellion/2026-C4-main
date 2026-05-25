package frc.robot.systems.vision;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public class CameraIOPV implements CameraIO{
    private PhotonCamera mPhotonCamera;
    private Transform3d mPhotonCameraPosition;

    public CameraIOPV(String pCameraName, Transform3d pCameraPosition) {
        mPhotonCamera = new PhotonCamera(pCameraName);
        mPhotonCameraPosition = pCameraPosition;
    }

    @Override
    public void updateInputs(CameraIOInputs pInputs) { // >>> TODO: incorporate multitag approach so that I can get pose of robot
        try { // >>> For now only updates for single tags | Does not update odometry based on pose of robot
            pInputs.iIsConnected = mPhotonCamera.isConnected();

            List<PhotonPipelineResult> unreadResults = mPhotonCamera.getAllUnreadResults();
                pInputs.iHasBeenUpdated = !unreadResults.isEmpty();
            if (!pInputs.iHasBeenUpdated)
                return;

            PhotonPipelineResult latestValidResult = unreadResults.get(unreadResults.size() - 1);

            pInputs.iHasTarget = !(latestValidResult == null || !latestValidResult.hasTargets());

            if(pInputs.iHasTarget) {
                    PhotonTrackedTarget target = latestValidResult.getBestTarget();
                    pInputs.iSingleTagAprilTagID = target.getFiducialId();
                    pInputs.iPoseAmbiguity = target.getPoseAmbiguity();
                    pInputs.iYaw = target.getYaw();
                    pInputs.iPitch = target.getPitch();
                    pInputs.iArea = target.getArea();
                    pInputs.iLatestTimestamp = latestValidResult.getTimestampSeconds();
            }
        }
        catch(Exception e) {
            e.printStackTrace();
            resetInputs(pInputs);
        }
    }

    private void resetInputs(CameraIOInputs pInputs) {
        pInputs.iIsConnected = false;
        pInputs.iHasTarget = false;
        pInputs.iHasBeenUpdated = false;
        pInputs.iYaw = 0.0;
        pInputs.iPitch = 0.0;
        pInputs.iArea = 0.0;
        pInputs.iPoseAmbiguity = 0.0;
        pInputs.iSingleTagAprilTagID = 0;
        pInputs.iNumberOfTargets = 0;
        pInputs.iLatestTimestamp = 0.0;
        pInputs.iLatestEstimatedRobotPose = new Pose3d();
        pInputs.iLatestTagTransforms = new Transform3d[0];
        pInputs.iLatestTagAmbiguities = new double[0];
    }
}
