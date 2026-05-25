package frc.robot.systems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;
import frc.lib.telemetry.Telemetry;
import frc.lib.tuning.LoggedTunableNumber;
import frc.robot.systems.vision.CameraIO.CameraIOConfigInputs;

public class Vision {
    private static final LoggedTunableNumber tSingleXYStdev =
        new LoggedTunableNumber("Vision/kSingleXYStdev", VisionConstants.kSingleStdDevs.get(0));
    private static final LoggedTunableNumber tMultiXYStdev =
        new LoggedTunableNumber("Vision/kMultiXYStdev", VisionConstants.kMultiStdDevs.get(0));

    private CameraIO[] mCameras;
    private CameraIOInputsAutoLogged[] mCamerasData;
    private VisionObservation[] mCurrentVisionObservation = new VisionObservation[4]; //what does this do?

    public Vision(CameraIO[] pCameras) {
        this.mCameras = pCameras;
        this.mCamerasData = new CameraIOInputsAutoLogged[pCameras.length];

        for (int i = 0; i < pCameras.length; i++) {
            mCamerasData[i] = new CameraIOInputsAutoLogged();
        }
    }

    public void periodic(Pose2d pLastRobotPose, Pose2d pSimOdomPose) {
        for (int i = 0; i < mCameras.length; i++) {
            mCameras[i].updateInputs(mCamerasData[i]);
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

    private VisionObservation processCameraObservation(CamearIOInputsAutoLogged pCamData, CameraIOConfigInputs pCamConfig, int id) {
        if (!pCamData.iHasTarget || !pCamData.iHasBeenUpdated) {
            return makeInvalidObservation(pCamData, pCamConfig.iCamName);
        }

        int usableTags = 0;
        double totalDistance = 0.0;

        for (int i = 0; i < pCamData.iLatestTagTransforms.length; i++) {
            if (pCamData.iLatestTagTransforms[i] != null && pCamData.iLatestTagAmbiguities[i] < VisionConstants.kAmbiguityThreshold) {
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
            CameraIOInputsAutoLogged pCamData, CameraIOConfigInputs pCamConfig, double pAvgDist, double pXYScalar) {
        if (pAvgDist > VisionConstants.kMaxTrustDistanceMSingletag) {
            return makeUntrustedObservation(pCamData, pCamConfig.iCamName);
        }

        Pose2d pose = pCamData.iLatestEstimatedRobotPose.toPose2d();

        return makeVisionObservation(pose, tSingleXYStdev.get() * pXYScalar, pCamData, pCamConfig.iCamName, false);
    }

    private VisionObservation makeVisionObservation(Pose2d pPose, double pXYStdev, CameraIOInputsAutoLogged pCamData, String pCamName, boolean useRotation) {
        return new VisionObservation(
            true,
            pPose,
            VecBuilder.fill(pXYStdev, pXYStdev, (useRotation) ? pXYStdev * VisionConstants.kRotStdDevScalar : Double.MAX_VALUE),
            pCamData.iLatestTimestamp,
            pCamName);
    }

    private VisionObservation makeUntrustedObservation(CameraIOInputsAutoLogged pCamData, String pCamName) {
        return new VisionObservation(
            true,
            pCamData.iLatestEstimatedRobotPose.toPose2d(),
            VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE),
            pCamData.iLatestTimestamp,
            pCamName);
    }

    private VisionObservation makeInvalidObservation(CameraIOInputsAutoLogged pCamData, String pCamName) {
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
