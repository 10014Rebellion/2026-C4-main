package frc.lib.hardware;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;

public class HardwareRecords {
    public static record CurrentLimits(
        double supplyCurrentLimit,
        double statorCurrentLimit
    ) {}

    public static record PositionSoftLimits(
        double backwardLimitM,
        double forwardLimitM
    ) {}

    public static record RotationSoftLimits(
        Rotation2d backwardLimit,
        Rotation2d forwardLimit
    ) {}

    public static record MotionConstraints(
        double maxVelocity,
        double maxAcceleration,
        double maxJerk
    ) {}

    public static record CANRangeConfiguration(
        int canRangeID,
        double fuelDetectionCutoff,
        double positionTolerance
    ) {}

    public static record BasicMotorHardware(
        int motorID, 
        CANBus canBus, 
        double rotorToMechanismRatio,
        InvertedValue direction,
        NeutralModeValue neutralMode,
        CurrentLimits currentLimit
    ) {}

    public static record FollowerMotorHardware(
        int motorID,
        BasicMotorHardware leaderConfig,
        MotorAlignmentValue alignmentValue
    ) {}

    public static record RelativeCANCoderHardware(
        int cancoderID,
        double cancoderToMechanismRatio,
        SensorDirectionValue direction
    ) {}

    public static record CANdiEncoder(
        int canID,
        FeedbackSensorSourceValue feedbackPort,
        double sensorToMechanismRatio,
        Rotation2d offset
    ){}

    public static record ElevatorController(
        int slot,
        PDConstants pdController,
        ElevatorFeedforward feedforward,
        double tolerance
    ) {}

    public static record ArmController(
        int slot,
        PDConstants pdController,
        ArmFeedforward feedforward
    ) {}

    public static record ArmControllerMotionMagic(
        int slot,
        PDConstants pdController,
        MotionMagicConstants motionMagicConstants,
        ArmFeedforward feedforward
    ) {}

    public static record SimpleController(
        int slot,
        PDConstants pdController,
        SimpleMotorFeedforward feedforward
    ) {}

    public static record MotionMagicFOCElevatorFF(
        int slot,
        PDConstants pdController,
        ElevatorFeedforward feedforward,
        MotionMagicConstants motionMagicConstants   
    ) {}

    public static record MotionMagicFOCController(
        int slot,
        PDConstants pdController,
        MotionMagicConstants motionMagicConstants
    ) {}

    public static record MotionMagicFOCControllerFF(
        int slot,
        PDConstants pdController,
        SimpleMotorFeedforward feedforward,
        MotionMagicConstants motionMagicConstants
    ) {}

    public static record MotionMagicConstants(
        double maxVelocity,
        double maxAcceleration,
        double maxJerk
    ) {}

    public static record PDConstants(
        double kP,
        double kD
    ) {} 
}
