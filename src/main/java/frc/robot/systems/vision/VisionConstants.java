package frc.robot.systems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

public class VisionConstants {
        // Best to get these from CAD, or in person.
    //Camera 1
        public static final String kFrontLeftCamName = "SideLeft-OV2311"; 
        public static final Transform3d kFrontLeftCamTransform = new Transform3d( // >>> Camera's position relative to robot's center
            new Translation3d(
                Units.inchesToMeters(-4.4), // X: inches back 
                Units.inchesToMeters(12.6), // Y: inches left 
                Units.inchesToMeters(19.5) // Z: inches above ground 
            ),
            new Rotation3d(
                Units.degreesToRadians(0.0), // Roll: side tilt 
                Units.degreesToRadians(10), // Pitch: upward tilt 
                Units.degreesToRadians(-20.0) // Yaw: (angled inward/outward) 
            ));
    
    //Camera 2
        public static final String kFrontRightCamName = "SideRight-OV2311"; 
        public static final Transform3d kFrontRightCamTransform = new Transform3d( // >>> Camera's position relative to robot's center
            new Translation3d(
                Units.inchesToMeters(-4.3), // X: inches back 
                Units.inchesToMeters(-12.6), // Y: inches right 
                Units.inchesToMeters(19.5) // Z: inches above ground 
            ),
            new Rotation3d(
                Units.degreesToRadians(0.0), // Roll: side tilt 
                Units.degreesToRadians(10), // Pitch: upward tilt 
                Units.degreesToRadians(20.0) // Yaw: (angled inward/outward) 
            ));

    //Camera 3
        public static final String kBackLeftCamName = "BackLeft-OV2311";
        public static final Transform3d kBackLeftCamTransform = new Transform3d( // >>> Camera's position relative to robot's center
            new Translation3d(
                Units.inchesToMeters(-9.1), // X: inches back 
                Units.inchesToMeters(14.1), // Y: inches left 
                Units.inchesToMeters(19.5) // Z: inches above ground 
            ),
            new Rotation3d(
                Units.degreesToRadians(0.0), // Roll: side tilt 
                Units.degreesToRadians(10.0), // Pitch: upward tilt 
                Units.degreesToRadians(108.0) // Yaw: angled outward 
            ));

    //Camera 4
        public static final String kBackRightCamName = "BackRight-OV2311"; 
        public static final Transform3d kBackRightCamTransform = new Transform3d( // >>> Camera's position relative to robot's center
            new Translation3d(
                Units.inchesToMeters(-9.4), // Y: inches right 
                Units.inchesToMeters(-14.1), // Y: inches right 
                Units.inchesToMeters(19.5) // Z: inches above ground 
            ),
            new Rotation3d(
                Units.degreesToRadians(0.0), // Roll: No side tilt 
                Units.degreesToRadians(10.0), // Pitch: No upward tilt 
                Units.degreesToRadians(-108.0) // Yaw: (angled inward/outward) 
            ));
       
    //What does ts do?
        // Tuned by using AdvantageScope data analysis tool(Normal distribution)
        public static final Vector<N3> kSingleStdDevs =
                (RobotBase.isReal()) ? VecBuilder.fill(0.274375, 0.274375, 5.0) : VecBuilder.fill(0.23, 0.23, 5.0);
        public static final Vector<N3> kMultiStdDevs =
                (RobotBase.isReal()) ? VecBuilder.fill(0.23188, 0.23188, 5.0) : VecBuilder.fill(0.23, 0.23, 5.0);

        public static final double kRotStdDevScalar = 20.0;

        public static final double kAmbiguityThreshold = (RobotBase.isReal()) ? 0.2 : 1.0;

        public static final double kMaxTrustDistanceMSingletag =
            3.5; // >>> TODO: Maybe? TUNE ME, you probably wont have to but yk just in case

}
