package frc.robot.systems.shooter.combinedShooter;

import java.util.HashMap;
import java.util.function.Supplier;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.hardware.HardwareRecords.BasicMotorHardware;
import frc.lib.hardware.HardwareRecords.CurrentLimits;
import frc.lib.hardware.HardwareRecords.FollowerMotorHardware;
import frc.lib.hardware.HardwareRecords.MotionMagicConstants;
import frc.lib.hardware.HardwareRecords.MotionMagicFOCControllerFF;
import frc.lib.hardware.HardwareRecords.PDConstants;
import frc.lib.hardware.HardwareRecords.RelativeCANCoderHardware;
import frc.lib.hardware.HardwareRecords.SimpleController;
import frc.lib.tuning.LoggedTunableNumber;
import frc.robot.RobotConstants;
import frc.robot.systems.shooter.combinedShooter.ShooterSS.ShooterStates;

public class ShooterConstants {
        public static final double kMaxFlywheelTestedRPS = 115;
        public static final double kToleranceRPS = 3.0;
        public static final double kBangBangTimeout = 0.25;
        public static final Rotation2d kRPSForShooting = Rotation2d.fromRotations(50);


        public static final RelativeCANCoderHardware kCANCoderConfig = new RelativeCANCoderHardware(
                50,
                1,
                SensorDirectionValue.Clockwise_Positive);

        public static final BasicMotorHardware kFlywheelLeaderConfig = new BasicMotorHardware(
                51,
                RobotConstants.kSubsystemsCANBus,
                1,
                InvertedValue.CounterClockwise_Positive,
                NeutralModeValue.Coast,
                new CurrentLimits(40, 80));

        public static final FollowerMotorHardware kFlywheelFollowerConfig = new FollowerMotorHardware(
                52,
                kFlywheelLeaderConfig,
                MotorAlignmentValue.Opposed);

        public static final MotionMagicFOCControllerFF kFlywheelControlConfig = new MotionMagicFOCControllerFF(
                0,
                new PDConstants(7.5, 0), // Tuned for C3RBERUS!
                new SimpleMotorFeedforward(2.5, 0.22, 1.8), // Tuned for C3RBERUS!
                new MotionMagicConstants(0.0, 60.0, 0.0));

                public static final FollowerMotorHardware kFuelPumpFollower1Config = new FollowerMotorHardware(
                        53,
                        kFlywheelLeaderConfig,
                        MotorAlignmentValue.Aligned
                );

                public static final FollowerMotorHardware kFuelPumpFollower2Config = new FollowerMotorHardware(
                        54,
                        kFlywheelLeaderConfig,
                        MotorAlignmentValue.Opposed
                );

                public static final SimpleController kFuelPumpControlConfig = new SimpleController(
                        0, 
                        new PDConstants(0.22, 0), 
                        new SimpleMotorFeedforward(0.35, 0.0989)
                );    
        
        
                public static final LoggedTunableNumber tLowestHailstormRPS = new LoggedTunableNumber(
                "Shooter/Flywheel/HailstormMinimumRPS", 90
        );


        public static final HashMap<ShooterStates, LoggedTunableNumber> kShooterSetpointToVoltageTuneable = new HashMap<ShooterStates, LoggedTunableNumber>();
        public static final HashMap<ShooterStates, Supplier<Rotation2d>> kShooterSetpointToVelocity = new HashMap<ShooterStates, Supplier<Rotation2d>>();
        
        public static final LoggedTunableNumber tRevVoltage = new LoggedTunableNumber(
                "Shooter/Flywheel/SetpointsVoltage/StandbyVoltage", 0.0); //TODO: set this back to 6.0
        public static final LoggedTunableNumber tStandbyVoltage = new LoggedTunableNumber(
                "Shooter/Flywheel/SetpointsVoltage/StandbyVoltage", 0.0);
        public static final LoggedTunableNumber tTuningVoltage = new LoggedTunableNumber(
                "Shooter/Flywheel/SetpointsVoltage/TuneVoltage", 0.0);
        public static final LoggedTunableNumber tMaxVoltage = new LoggedTunableNumber(
                "Shooter/Flywheel/SetpointsVoltage/MaxVoltage", 0.0);
        public static final LoggedTunableNumber tTuningVelocity = new LoggedTunableNumber(
                "Shooter/Flywheel/TuneVelocityRPS", 0.0);
        public static final LoggedTunableNumber tFeedVelocity = new LoggedTunableNumber("Shooter/Flywheel/FeedVelocity",
                60.0);
        public static final LoggedTunableNumber tOpponentFeedVelocity = new LoggedTunableNumber(
                "Shooter/Flywheel/OpponentFeedVelocity", 90.0);
        public static final LoggedTunableNumber tHailstormVoltage = new LoggedTunableNumber("Shooter/Flywheel/HailstormVoltage", 12.0);
        public static final LoggedTunableNumber tCloseVelocity = new LoggedTunableNumber(
                "Shooter/Flywheel/SetpointRPS/CloseVelocity", 47.5);
        public static final LoggedTunableNumber tMaxVelocity = new LoggedTunableNumber(
                "Shooter/Flywheel/SetpointRPS/MaxVelocity", 0.0);
        public static final LoggedTunableNumber tStandbyVelocity = new LoggedTunableNumber(
                "Shooter/Flywheel/SetpointRPS/StandbyVelocity", 0.0);

        public static final LoggedTunableNumber tTowerVelocity = new LoggedTunableNumber(
                "Shooter/Flywheel/SetpointRPS/TowerVelocity", 57.5);
        public static final LoggedTunableNumber tBumpVelocity = new LoggedTunableNumber(
                "Shooter/Flywheel/SetpointRPS/BumpVelocity", 40.0);
        public static final LoggedTunableNumber tCornerVelocity = new LoggedTunableNumber(
                "Shooter/Flywheel/SetpointRPS/CornerVelocity", 90);
        public static final LoggedTunableNumber tTrenchVelocity = new LoggedTunableNumber(
                "Shooter/Flywheel/SetpointRPS/TrenchVelocity", 80);

        public static final LoggedTunableNumber tTuningAmperage 
                = new LoggedTunableNumber("Shooter/Flywheel/TuningAmperage", 0.0);

        public static final LoggedTunableNumber tIntakeVolts  = new LoggedTunableNumber("Shooter/FuelPump/DesiredVolts/IntakeVolts", 6);
        public static final LoggedTunableNumber tKickbackVolts  = new LoggedTunableNumber("Shooter/FuelPump/DesiredVolts/IntakeVolts", -3);

        public static final LoggedTunableNumber tOuttakeVolts  = new LoggedTunableNumber("Shooter/FuelPump/DesiredVolts/OuttakeVolts", -10.014);
        public static final LoggedTunableNumber tSlightOuttakeVolts = new LoggedTunableNumber("Shooter/FuelPump/DesiredVolts/SlightOuttakeVolts", -2.0);
        public static final LoggedTunableNumber tIntakeVelocity = new LoggedTunableNumber("Shooter/FuelPump/DesiredVelocity/IntakeVelocity", 40.0);
        
        static {
                kShooterSetpointToVoltageTuneable.put(ShooterStates.HAILSTORM_VOLTAGE, tHailstormVoltage);
                kShooterSetpointToVoltageTuneable.put(ShooterStates.STANDBY_VOLTAGE, tStandbyVoltage);
                kShooterSetpointToVoltageTuneable.put(ShooterStates.REV_VOLTAGE, tRevVoltage);
                kShooterSetpointToVoltageTuneable.put(ShooterStates.TUNING_VOLTAGE, tTuningVoltage);
                kShooterSetpointToVoltageTuneable.put(ShooterStates.MAX_VOLTAGE, tMaxVoltage);

                kShooterSetpointToVelocity.put(ShooterStates.TUNING_VELOCITY,
                        () -> Rotation2d.fromRotations(tTuningVelocity.get()));
                // kShooterSetpointToVelocity.put(ShooterStates.FEED_VELOCITY,
                //         () -> Rotation2d.fromRotations(tFeedVelocity.get()));
                kShooterSetpointToVelocity.put(ShooterStates.OPPONENT_FEED_VELOCITY,
                        () -> Rotation2d.fromRotations(tOpponentFeedVelocity.get()));
                kShooterSetpointToVelocity.put(ShooterStates.MAX_VELOCITY,
                        () -> Rotation2d.fromRotations(tMaxVelocity.get()));
                kShooterSetpointToVelocity.put(ShooterStates.STANDBY_VELOCITY,
                        () -> Rotation2d.fromRotations(tStandbyVelocity.get()));

                kShooterSetpointToVelocity.put(ShooterStates.TOWER_VELOCITY,
                        () -> Rotation2d.fromRotations(tTowerVelocity.get()));
                kShooterSetpointToVelocity.put(ShooterStates.BUMP_VELOCITY,
                        () -> Rotation2d.fromRotations(tBumpVelocity.get()));

                kShooterSetpointToVelocity.put(ShooterStates.CORNER_VELOCITY,
                        () -> Rotation2d.fromRotations(tCornerVelocity.get()));
                kShooterSetpointToVelocity.put(ShooterStates.TRENCH_VELOCITY,
                        () -> Rotation2d.fromRotations(tTrenchVelocity.get()));
                
                kShooterSetpointToVoltageTuneable.put(ShooterStates.INTAKE_VOLT, tIntakeVolts);
                kShooterSetpointToVoltageTuneable.put(ShooterStates.KICKBACK_VOLT, tKickbackVolts);
                kShooterSetpointToVoltageTuneable.put(ShooterStates.OUTTAKE_VOLT, tOuttakeVolts);
                kShooterSetpointToVoltageTuneable.put(ShooterStates.SLIGHT_OUTTAKE_VOLT, tSlightOuttakeVolts);
                kShooterSetpointToVelocity.put(ShooterStates.INTAKE_VELOCITY, () -> Rotation2d.fromRotations(tIntakeVelocity.get()));
        
        }
}
