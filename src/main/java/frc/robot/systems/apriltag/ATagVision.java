// REBELLION 10014

package frc.robot.systems.apriltag;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;
import frc.lib.telemetry.Telemetry;
import frc.lib.tuning.LoggedTunableNumber;
import frc.robot.systems.apriltag.ATagCameraIO.AprilTagCameraIOConfigInputs;

import static frc.robot.systems.apriltag.ATagVisionConstants.kAmbiguityThreshold;
import static frc.robot.systems.apriltag.ATagVisionConstants.kMultiStdDevs;
import static frc.robot.systems.apriltag.ATagVisionConstants.kRotStdDevScalar;
import static frc.robot.systems.apriltag.ATagVisionConstants.kSingleStdDevs;

import org.littletonrobotics.junction.Logger;

public class ATagVision {
    private static final LoggedTunableNumber tSingleXYStdev =
        new LoggedTunableNumber("Vision/kSingleXYStdev", kSingleStdDevs.get(0));
    private static final LoggedTunableNumber tMultiXYStdev =
        new LoggedTunableNumber("Vision/kMultiXYStdev", kMultiStdDevs.get(0));

    private ATagCameraIO[] mCameras;
    private AprilTagIOInputsAutoLogged[] mCamerasData;

    private VisionObservation[] mCurrentVisionObservation = new VisionObservation[4];


    public ATagVision(ATagCameraIO[] pCameras) {
        this.mCameras = pCameras;
        this.mCamerasData = new AprilTagIOInputsAutoLogged[pCameras.length];

        for (int i = 0; i < pCameras.length; i++) {
            mCamerasData[i] = new AprilTagIOInputsAutoLogged();
        }
    }

    public void periodic(Pose2d pLastRobotPose, Pose2d pSimOdomPose) {
        for (int i = 0; i < mCameras.length; i++) {
            mCameras[i].updateInputs(mCamerasData[i], pLastRobotPose, pSimOdomPose);
            Logger.processInputs("Vision/" + mCameras[i].getConfigInputs().iCamName, mCamerasData[i]);
        }
    }

    public VisionObservation[] getVisionObservations() {
        VisionObservation[] observations = new VisionObservation[mCameras.length];
        for (int i = 0; i < mCamerasData.length; i++) {
            observations[i] = processCameraObservation(mCamerasData[i], mCameras[i].getConfigInputs(),  i);
        }
        return observations;
    }

    private VisionObservation processCameraObservation(AprilTagIOInputsAutoLogged pCamData, AprilTagCameraIOConfigInputs pCamConfig, int id) {
        if (!pCamData.iHasTarget || !pCamData.iHasBeenUpdated) {
            return makeInvalidObservation(pCamData, pCamConfig.iCamName);
        }

        int usableTags = 0;
        double totalDistance = 0.0;

        for (int i = 0; i < pCamData.iLatestTagTransforms.length; i++) {
            if (pCamData.iLatestTagTransforms[i] != null && pCamData.iLatestTagAmbiguities[i] < kAmbiguityThreshold) {
                totalDistance +=
                    pCamData.iLatestTagTransforms[i].getTranslation().getNorm();
                usableTags++;
            }
        }

        if (usableTags == 0) {
            return makeUntrustedObservation(pCamData, pCamConfig.iCamName);
        }

        double avgDist = totalDistance / usableTags;
        double xyScalar = Math.pow(avgDist, 2) / usableTags;

        if (usableTags == 1) {
            mCurrentVisionObservation[id] = processSingleTagObservation(pCamData, pCamConfig, avgDist, xyScalar);
        } else {
            mCurrentVisionObservation[id] = makeVisionObservation(
                pCamData.iLatestEstimatedRobotPose.toPose2d(), 
                tMultiXYStdev.get() * xyScalar, 
                pCamData, 
                pCamConfig.iCamName,
                true);
        }

        Telemetry.logVisionObservationStdDevs(mCurrentVisionObservation[id]);

        return mCurrentVisionObservation[id];
    }

    private VisionObservation processSingleTagObservation(
            AprilTagIOInputsAutoLogged pCamData, AprilTagCameraIOConfigInputs pCamConfig, double pAvgDist, double pXYScalar) {
        if (pAvgDist > ATagVisionConstants.kMaxTrustDistanceMSingletag) {
            return makeUntrustedObservation(pCamData, pCamConfig.iCamName);
        }

        Pose2d pose = pCamData.iLatestEstimatedRobotPose.toPose2d();

        return makeVisionObservation(pose, tSingleXYStdev.get() * pXYScalar, pCamData, pCamConfig.iCamName, false);
    }

    private VisionObservation makeVisionObservation(Pose2d pPose, double pXYStdev, AprilTagIOInputsAutoLogged pCamData, String pCamName, boolean useRotation) {
        return new VisionObservation(
            true,
            pPose,
            VecBuilder.fill(pXYStdev, pXYStdev, (useRotation) ? pXYStdev * kRotStdDevScalar : Double.MAX_VALUE),
            pCamData.iLatestTimestamp,
            pCamName);
    }

    private VisionObservation makeUntrustedObservation(AprilTagIOInputsAutoLogged pCamData, String pCamName) {
        return new VisionObservation(
            true,
            pCamData.iLatestEstimatedRobotPose.toPose2d(),
            VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE),
            pCamData.iLatestTimestamp,
            pCamName);
    }

    private VisionObservation makeInvalidObservation(AprilTagIOInputsAutoLogged pCamData, String pCamName) {
        return new VisionObservation(
            false,
            new Pose2d(),
            VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE),
            pCamData.iLatestTimestamp,
            pCamName);
    }

    public record VisionObservation(
        boolean hasObserved, Pose2d pose, Vector<N3> stdDevs, double timeStamp, String camName) {}
}
