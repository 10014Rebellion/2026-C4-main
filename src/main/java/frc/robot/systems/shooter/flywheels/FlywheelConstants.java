package frc.robot.systems.shooter.flywheels;

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
import frc.lib.tuning.LoggedTunableNumber;
import frc.robot.RobotConstants;
import frc.robot.systems.shooter.flywheels.FlywheelsSS.FlywheelStates;

public class FlywheelConstants {
    public static final double kMaxFlywheelTestedRPS = 115;
    public static final double kToleranceRPS = 3.0;
    public static final double kBangBangTimeout = 0.25;

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

    public static final LoggedTunableNumber tLowestHailstormRPS = new LoggedTunableNumber(
        "Shooter/Flywheel/HailstormMinimumRPS", 90
    );


    public static final HashMap<FlywheelStates, LoggedTunableNumber> kFlywheelSetpointToVoltageTuneable = new HashMap<FlywheelStates, LoggedTunableNumber>();
    public static final HashMap<FlywheelStates, Supplier<Rotation2d>> kFlywheelSetpointToVelocity = new HashMap<FlywheelStates, Supplier<Rotation2d>>();

    public static final LoggedTunableNumber tRevVoltage = new LoggedTunableNumber(
            "Shooter/Flywheel/SetpointsVoltage/StandbyVoltage", 6.0);
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
            "Shooter/Flywheel/SetpointRPS/StandbyVelocity", 17.5);

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

    static {
        kFlywheelSetpointToVoltageTuneable.put(FlywheelStates.HAILSTORM_VOLTAGE, tHailstormVoltage);
        kFlywheelSetpointToVoltageTuneable.put(FlywheelStates.STANDBY_VOLTAGE, tStandbyVoltage);
        kFlywheelSetpointToVoltageTuneable.put(FlywheelStates.REV_VOLTAGE, tRevVoltage);
        kFlywheelSetpointToVoltageTuneable.put(FlywheelStates.TUNING_VOLTAGE, tTuningVoltage);
        kFlywheelSetpointToVoltageTuneable.put(FlywheelStates.MAX_VOLTAGE, tMaxVoltage);

        kFlywheelSetpointToVelocity.put(FlywheelStates.TUNING_VELOCITY,
                () -> Rotation2d.fromRotations(tTuningVelocity.get()));
        kFlywheelSetpointToVelocity.put(FlywheelStates.FEED_VELOCITY,
                () -> Rotation2d.fromRotations(tFeedVelocity.get()));
        kFlywheelSetpointToVelocity.put(FlywheelStates.OPPONENT_FEED_VELOCITY,
                () -> Rotation2d.fromRotations(tOpponentFeedVelocity.get()));
        kFlywheelSetpointToVelocity.put(FlywheelStates.MAX_VELOCITY,
                () -> Rotation2d.fromRotations(tMaxVelocity.get()));
        kFlywheelSetpointToVelocity.put(FlywheelStates.STANDBY_VELOCITY,
                () -> Rotation2d.fromRotations(tStandbyVelocity.get()));

        kFlywheelSetpointToVelocity.put(FlywheelStates.TOWER_VELOCITY,
                () -> Rotation2d.fromRotations(tTowerVelocity.get()));
        kFlywheelSetpointToVelocity.put(FlywheelStates.BUMP_VELOCITY,
                () -> Rotation2d.fromRotations(tBumpVelocity.get()));

        kFlywheelSetpointToVelocity.put(FlywheelStates.CORNER_VELOCITY,
                () -> Rotation2d.fromRotations(tCornerVelocity.get()));
        kFlywheelSetpointToVelocity.put(FlywheelStates.TRENCH_VELOCITY,
                () -> Rotation2d.fromRotations(tTrenchVelocity.get()));
    }
}
