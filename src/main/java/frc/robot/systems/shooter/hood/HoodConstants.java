package frc.robot.systems.shooter.hood;

import java.util.HashMap;
import java.util.function.Supplier;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.lib.hardware.HardwareRecords.ArmControllerMotionMagic;
import frc.lib.hardware.HardwareRecords.BasicMotorHardware;
import frc.lib.hardware.HardwareRecords.CurrentLimits;
import frc.lib.hardware.HardwareRecords.MotionMagicConstants;
import frc.lib.hardware.HardwareRecords.PDConstants;
import frc.lib.hardware.HardwareRecords.RotationSoftLimits;
import frc.lib.tuning.LoggedTunableNumber;
import frc.robot.RobotConstants;
import frc.robot.systems.shooter.hood.HoodSS.HoodStates;

public class HoodConstants {
    public static final BasicMotorHardware kHoodConfig = new BasicMotorHardware(
            55, // Motor CAN ID
            RobotConstants.kSubsystemsCANBus, // CANBus
            (133.0 / 9.0) * (20.0 / 12.0), // Rotor to Mechanism Gear Ratio
            InvertedValue.CounterClockwise_Positive, // Direction
            NeutralModeValue.Brake, // Neutral Mode
            new CurrentLimits(
                    40, // Supply
                    50 // Stator
            ));

    public static final ArmControllerMotionMagic kHoodControlConfig = new ArmControllerMotionMagic(
            0, // not currently used
            new PDConstants(300, 5), // Tuned for C3RBERUS!
            new MotionMagicConstants(8000.0 / 360.0, 8000. / 360.0, 0), // Tuned for C3RBERUS!
            new ArmFeedforward(0.7, 0.1, 0, 0) // Tuned for C3RBERUS!
    );

    public static final Rotation2d kRestingTrueAngle = Rotation2d.fromDegrees(7.5);

    public static final RotationSoftLimits kHoodLimits = new RotationSoftLimits(
            Rotation2d.fromDegrees(0.02),
            Rotation2d.fromDegrees(40.0) // Tuned for C3RBERUS!
    );

    public static final Rotation2d kAdjustStepAmount = Rotation2d.fromDegrees(2);
    public static final Rotation2d kTolerance = Rotation2d.fromRotations(0.03);
    public static final double kHoodLength = Units.feetToMeters(8.061);
    public static final double kHoodMass = Units.inchesToMeters(1.258);

    // Only add states that have constant setpoint throughout a real match.
    public static final HashMap<HoodStates, Supplier<Rotation2d>> kStateToSetpointMapHood = new HashMap<HoodStates, Supplier<Rotation2d>>();

    public static final LoggedTunableNumber tTuningVoltage = new LoggedTunableNumber("Hood/TuneVoltage", 0.0);
    public static final LoggedTunableNumber tTuningAmp = new LoggedTunableNumber("Hood/TuneAmperage", 0.0);
    public static final LoggedTunableNumber tTuningShotSetpointDeg = new LoggedTunableNumber(
            "Hood/Setpoint/TuningShotSetpointDegrees", 0.0);
    public static final LoggedTunableNumber tMaxSetpointDeg = new LoggedTunableNumber(
            "Hood/Setpoint/MaxSetpointDegrees", kHoodLimits.forwardLimit().getDegrees());
    public static final LoggedTunableNumber tMidSetpointDeg = new LoggedTunableNumber(
            "Hood/Setpoint/MidSetpointDegrees", kHoodLimits.forwardLimit().getDegrees() / 2.0);
    public static final LoggedTunableNumber tMinSetpointDeg = new LoggedTunableNumber(
            "Hood/Setpoint/MinSetpointDegrees", kHoodLimits.backwardLimit().getDegrees());
    public static final LoggedTunableNumber tIncrementSpeedDPS = new LoggedTunableNumber("Hood/IncrementSpeedDPS", 1.0);

    public static final LoggedTunableNumber tTowerShotSetpointDeg = new LoggedTunableNumber(
            "Hood/Setpoint/TowerShotSetpointDegrees", 13.0);
    public static final LoggedTunableNumber tBumpShotSetpointDeg = new LoggedTunableNumber(
            "Hood/Setpoint/BumpShotSetpointDegrees", 5.0);
    public static final LoggedTunableNumber tTrenchShotSetpointDeg = new LoggedTunableNumber(
            "Hood/Setpoint/TrenchShotSetpointDegrees", 15.0);
    public static final LoggedTunableNumber tCornerShotSetpointDeg = new LoggedTunableNumber(
            "Hood/Setpoint/CornerShotSetpointDegrees", 20.0);

    public static final LoggedTunableNumber tOpponentFeedShotSetpointDeg = new LoggedTunableNumber(
        "Hood/Setpoint/OpponentFeedShot", Rotation2d.fromDegrees(27.5+5).minus(kRestingTrueAngle).getDegrees());


    static {
        kStateToSetpointMapHood.put(HoodStates.MAX, () -> Rotation2d.fromDegrees(tMaxSetpointDeg.get()));
        kStateToSetpointMapHood.put(HoodStates.MID, () -> Rotation2d.fromDegrees(tMidSetpointDeg.get()));
        kStateToSetpointMapHood.put(HoodStates.MIN, () -> Rotation2d.fromDegrees(tMinSetpointDeg.get()));

        kStateToSetpointMapHood.put(HoodStates.OPPONENT_FEED_ANGLE, () -> Rotation2d.fromDegrees(tOpponentFeedShotSetpointDeg.get()));

        kStateToSetpointMapHood.put(HoodStates.TOWER_ANGLE, () -> Rotation2d.fromDegrees(tTowerShotSetpointDeg.get()));
        kStateToSetpointMapHood.put(HoodStates.BUMP_ANGLE, () -> Rotation2d.fromDegrees(tBumpShotSetpointDeg.get()));
        kStateToSetpointMapHood.put(HoodStates.CORNER_ANGLE,
                () -> Rotation2d.fromDegrees(tCornerShotSetpointDeg.get()));
        kStateToSetpointMapHood.put(HoodStates.TRENCH_ANGLE,
                () -> Rotation2d.fromDegrees(tTrenchShotSetpointDeg.get()));

        kStateToSetpointMapHood.put(HoodStates.TUNING_SETPOINT,
                () -> Rotation2d.fromDegrees(tTuningShotSetpointDeg.get()));
    }

}
