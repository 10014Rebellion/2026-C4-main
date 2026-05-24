// REBELLION 10014

package frc.robot.game;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.math.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;

/* Chooses pose based of strategy and psoe */
// TODO: UPDATE ALL POSES FOR GAME
public class GameGoalPoseChooser {
    public static enum CHOOSER_STRATEGY {
        kTest
    }

    public static final Pose2d kTopLeftBumpPose = new Pose2d(
            5.679,
            7.451,
            Rotation2d.k180deg);

    public static final Pose2d kTopRightBumpPose = new Pose2d(
            4.0,
            7.451,
            Rotation2d.k180deg
        );

    public static final Pose2d kBottomLeftBumpPose =new Pose2d(
            5.679,
            0.659,
            Rotation2d.k180deg
        );

    public static final Pose2d kBottomRightBumpPose = new Pose2d(
            4.0,
            0.659,
            Rotation2d.k180deg
        );
    

    /* Static positions */
    public static Pose2d getSafeScoringPosition() {
        return AllianceFlipUtil.apply(new Pose2d());
    }

    public static Pose2d getPreloadStartingPosition() {
        return AllianceFlipUtil.apply(new Pose2d(
            3.5746593475341797,
            4.012706756591797,
            Rotation2d.kZero
        ));
    }

    public static Pose2d getO() {
        return AllianceFlipUtil.apply(new Pose2d());
    }

    public static Pose2d getD() {
        return AllianceFlipUtil.apply(new Pose2d());
    }

    public static boolean onLeftY(Pose2d robotPose) {

        if(DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue) && robotPose.getY() > FieldConstants.kFieldYM / 2.0){
            return true;
        }

        if(DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red) && robotPose.getY() < FieldConstants.kFieldYM / 2.0){
            return true;
        }

        return false;
    }

    /* Accoumts for rotation from reef, and offsets for red-side logic */
    public static Rotation2d turnFromHub(Pose2d robotPose) {
        Pose2d hubCenter = getHub();
        Rotation2d angleFromhubCenter = Rotation2d.fromRadians(
                Math.atan2(hubCenter.getY() - robotPose.getY(), hubCenter.getX() - robotPose.getX()));

        Rotation2d finalAngle = angleFromhubCenter;
        // if (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red))
        //     finalAngle = angleFromhubCenter.plus(Rotation2d.k180deg).times(-1.0);
        Logger.recordOutput("Drive/GoalPoseAngle", finalAngle);
        return finalAngle;
    }

    /* Non-static Trench */
    public static Pose2d getClosestTrench(Pose2d robotPose) {
        Pose2d trenchPose = onLeftOrRightY(robotPose) ? getTL() : getTR();
        Logger.recordOutput("GamePoses/ClosestTrench", trenchPose);
        return trenchPose; 
    }

    public static Pose2d getClosestClimbPose(Pose2d robotPose){
        Pose2d bumpPose = onLeftY(robotPose) ? getCL() : getCR();
        Logger.recordOutput("GamePoses/ClosestClimb", bumpPose);
        return bumpPose; 
    }

    public static Pose2d getTL() {
        return AllianceFlipUtil.apply(FieldConstants.kTrenchPoseLeft);
    }

    public static Pose2d getTR() {
        return AllianceFlipUtil.apply(FieldConstants.kTrenchPoseRight);
    }

    public static Pose2d getCL() {
        return AllianceFlipUtil.apply(FieldConstants.kClimbLeftPose);
    }

    public static Pose2d getCR(){
        return AllianceFlipUtil.apply(FieldConstants.kClimbRightPose);
    }

    /* Nonstatic bump */
    public static Pose2d getClosestBump(Pose2d robotPose) {
        Pose2d bumpPose = onLeftOrRightY(robotPose) ? getBL() : getBR();
        Logger.recordOutput("GamePoses/ClosestBump", bumpPose);
        return bumpPose; 
    }

    public static Pose2d getBL() {
        return AllianceFlipUtil.apply(FieldConstants.kBumpPoseLeft);
    }

    public static Pose2d getBR() {
        return AllianceFlipUtil.apply(FieldConstants.kBumpPoseRight);
    }

    public static Pose2d getHub() {
        Logger.recordOutput("GamePoses/HubPose", AllianceFlipUtil.apply(FieldConstants.kHubPose));
        return AllianceFlipUtil.apply(FieldConstants.kHubPose);
    }

    public static Pose2d closestClimbPose(Pose2d robotPose){
        if(
            FieldConstants.kClimbLeftPose.getTranslation().getDistance(robotPose.getTranslation()) < 
            FieldConstants.kClimbRightPose.getTranslation().getDistance(robotPose.getTranslation())){
            return AllianceFlipUtil.apply(FieldConstants.kClimbLeftPose);
        } 
        
        return AllianceFlipUtil.apply(FieldConstants.kClimbRightPose);
    }

    /* Utils function */
    public static boolean onLeftOrRightY(Pose2d robotPose) {
        return 
            (robotPose.getY() > FieldConstants.kFieldYM / 2.0
                &&
            DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue)) 
                ||
            (!(robotPose.getY() > FieldConstants.kFieldYM / 2.0)
                &&
            !DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue));
    }

    public static boolean inCenter(Pose2d robotPose) {
        return GameGoalPoseChooser.inBetween(robotPose.getX(), 5.2, 11.312);
    }

    public static boolean inEitherTrenchXRange(Pose2d robotPose) {
        return (inBetween(robotPose.getX(), 3.747, 5.477)
                    &&
                inBetween(robotPose.getX(), FieldConstants.kFieldXM - 5.477, FieldConstants.kFieldXM - 3.747));
    }

    public static boolean inEitherSuperTrenchXRange(Pose2d robotPose) {
        return (inBetween(robotPose.getX(), 4.093, 5.172)
                    &&
                inBetween(robotPose.getX(), FieldConstants.kFieldXM - 5.172, FieldConstants.kFieldXM - 4.093));
    }

    public static boolean inLeftTrenchYRange(Pose2d robotPose) {
        return inBetween(robotPose.getY(), FieldConstants.kFieldYM - 1.262, FieldConstants.kFieldYM);
    }

    public static boolean inRightTrenchYRange(Pose2d robotPose) {
        return inBetween(robotPose.getY(), 0, 1.262);
    }

    public static Pose2d getHubPresetPose(Pose2d robotPose, double distance) {
        Rotation2d hubRotation = turnFromHub(robotPose);
        Pose2d hubPose = getHub();

        return new Pose2d(
            hubPose.getX() - distance * hubRotation.getCos(),
            hubPose.getY() - distance * hubRotation.getSin(),
            turnFromHub(robotPose)
        );
    }

    public static Pose2d getCloseShotPose() {
        Logger.recordOutput("Autos/CloseShot", AllianceFlipUtil.apply(new Pose2d(
            3.351194381713867 - 1.0, 
            4.036095142364502, 
            Rotation2d.kZero)));

        return AllianceFlipUtil.apply(new Pose2d(
            3.351194381713867 - 1.0, 
            4.036095142364502, 
            Rotation2d.kZero));
    }

    public static boolean inBetween(double value, double min, double max) {
        return (value > min) && (value < max);
    }

    public static Pose2d leftTrenchApproachPose() {
        return AllianceFlipUtil.apply(
            new Pose2d(
                5.679,
                7.451,
                Rotation2d.k180deg
            )
        );
    }

    public static Pose2d leftTrenchExitPose() {
        return AllianceFlipUtil.apply(
            new Pose2d(
                4.0,
                7.451,
                Rotation2d.k180deg
            )
        );
    }

    public static Pose2d rightTrenchApproachPose() {
        return AllianceFlipUtil.apply(
            new Pose2d(
                5.679,
                0.659,
                Rotation2d.k180deg
            )
        );
    }

    public static Pose2d rightTrenchExitPose() {
        return AllianceFlipUtil.apply(
            new Pose2d(
                4.0,
                0.659,
                Rotation2d.k180deg
            )
        );
    }
}