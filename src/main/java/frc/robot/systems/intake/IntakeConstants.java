package frc.robot.systems.intake;

import java.util.HashMap;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotConstants;
import frc.robot.systems.intake.rack.IntakeRackSS.IntakeRackState;
import frc.robot.systems.intake.roller.IntakeRollerSS.IntakeRollerState;
import frc.lib.hardware.HardwareRecords.BasicMotorHardware;
import frc.lib.hardware.HardwareRecords.CurrentLimits;
import frc.lib.hardware.HardwareRecords.MotionMagicConstants;
import frc.lib.hardware.HardwareRecords.MotionMagicFOCElevatorFF;
import frc.lib.hardware.HardwareRecords.PDConstants;
import frc.lib.hardware.HardwareRecords.PositionSoftLimits;
import frc.lib.simulation.SimulationRecords.SimulatedElevator;
import frc.lib.tuning.LoggedTunableNumber;

public class IntakeConstants {
        public static class RackConstants {
                public record IntakeMotionConfig(
                                double kMaxTimeSec,
                                double kFarthestExtensionM,
                                double kClosestExtensionM,
                                int kMainPeaks,
                                double kHoldFreq,
                                double kHoldMinimum,
                                double kDropRatio // 0 = to farthest in every minima, 1 = linear. 0.5 = reduces minima
                                                  // by 0.5
                                                  // every period
                ) {

                        public static final IntakeMotionConfig kIntakeFunc = new IntakeMotionConfig(
                                        1.8, // kMaxTimeSec
                                        tIntakingSetpointMeters.get(), // kFarthestExtensionM
                                        0.21, // kClosestExtensionM
                                        3, // kMainPeaks
                                        2, // kHoldFreq
                                        0.136, // kHoldMinimum
                                        0.5 // kDropRatio
                        );
                }

                public static double kRackToleranceMeters = Units.inchesToMeters(0.5);
                public static double kRollerUsageCutoffMeters = 0.072;

                public static final BasicMotorHardware kRackMotorConfig = new BasicMotorHardware(
                                44, // TODO: TUNE ME;
                                RobotConstants.kSubsystemsCANBus,
                                (32.0 / 30.0) * (10.0 / ((9.0 / 15.0) * (16.0 / 24.0))),
                                InvertedValue.CounterClockwise_Positive,
                                NeutralModeValue.Coast,
                                new CurrentLimits(60, 90));

                public static final SimulatedElevator kRackElevator = new SimulatedElevator(
                                DCMotor.getKrakenX60(1),
                                9,
                                0.05,
                                // Pertend is perfectly horizontal //
                                false,
                                0.0,
                                0.001);

                // WITH POSITION VOLTAGE
                public static final MotionMagicFOCElevatorFF kRackController = (!RobotConstants.isSim())
                                ? new MotionMagicFOCElevatorFF( // REAL
                                                0,
                                                new PDConstants(1500.0, 75.0),
                                                new ElevatorFeedforward(0.3, 2.0, 0, 0),
                                                new MotionMagicConstants(60.0, 120.0, 0))
                                : new MotionMagicFOCElevatorFF( // SIM
                                                0,
                                                new PDConstants(40, 6.0),
                                                new ElevatorFeedforward(0.0, 0.0, 0.1, 0),
                                                new MotionMagicConstants(300.0, 300.0, 0));

                public static final PositionSoftLimits kRackLimitsMeters = new PositionSoftLimits(
                                -0.01, // Negative voltage limit
                                0.3 // Positive voltage limit
                );

                public static final double kArmLengthMeters = Units.inchesToMeters(12.0);
                public static final double kArmMassKg = 5.0;

                public static final LoggedTunableNumber tCompactTuningStowStepM = new LoggedTunableNumber(
                                "Intake/Compact/StowStepMeters");
                public static final LoggedTunableNumber tCompactTuningOutStepM = new LoggedTunableNumber(
                                "Intake/Compact/OutStepMeters");

                public static final LoggedTunableNumber tRackTuningVoltage = new LoggedTunableNumber(
                                "Intake/Tuning/TuneVoltage", 0.0);
                public static final LoggedTunableNumber tRackTuningAmp = new LoggedTunableNumber(
                                "Intake/Tuning/TuneAmperage",
                                0.0);
                public static final LoggedTunableNumber tConstantCompactAmps = new LoggedTunableNumber(
                                "Intake/Setpoint/CompactConstantAmps", 0.0);

                public static final LoggedTunableNumber tIncrementSpeedMPS = new LoggedTunableNumber(
                                "Intake/IncrementMPS",
                                0.04);

                public static final LoggedTunableNumber tStowSetpointMeters = new LoggedTunableNumber(
                                "Intake/Setpoint/StowSetpointMeters", 0.3);

                public static final LoggedTunableNumber tSafeStowSetpointMeters = new LoggedTunableNumber(
                                "Intake/Setpoint/SafeStowSetpointMeters", 0.22);

                public static final LoggedTunableNumber tCompactStartPositionMeters = new LoggedTunableNumber(
                                "Intake/Setpoint/CompactStartPositionMeters", 0.12);

                public static final LoggedTunableNumber tIntakingSetpointMeters = new LoggedTunableNumber(
                                "Intake/Setpoint/IntakeSetpointMeters", 0.0);

                public static final LoggedTunableNumber tTuningShotSetpointMeters = new LoggedTunableNumber(
                                "Intake/Setpoint/TuningShotSetpointMeters", 0);

                public static final LoggedTunableNumber tCompactHighSetpointMeters = new LoggedTunableNumber(
                                "Intake/Setpoint/CompactHighSetpointMeters", 0.1);

                public static final LoggedTunableNumber tCompactLowSetpointMeters = new LoggedTunableNumber(
                                "Intake/Setpoint/CompactLowSetpointMeters", 0.2);

                public static final HashMap<IntakeRackState, LoggedTunableNumber> kStateToSetpointMapIntake = new HashMap<>();

                static {
                        kStateToSetpointMapIntake.put(IntakeRackState.STOW, tStowSetpointMeters);
                        kStateToSetpointMapIntake.put(IntakeRackState.SAFESTOW, tSafeStowSetpointMeters);
                        kStateToSetpointMapIntake.put(IntakeRackState.INTAKE, tIntakingSetpointMeters);
                        kStateToSetpointMapIntake.put(IntakeRackState.TUNING_SETPOINT, tTuningShotSetpointMeters);
                        kStateToSetpointMapIntake.put(IntakeRackState.COMPACT_HIGH, tCompactHighSetpointMeters);
                        kStateToSetpointMapIntake.put(IntakeRackState.COMPACT_LOW, tCompactLowSetpointMeters);
                }
        }

        public static class RollerConstants {
                public final static BasicMotorHardware kRollerMotorConfig = new BasicMotorHardware(
                                42,
                                RobotConstants.kSubsystemsCANBus,
                                1,
                                InvertedValue.Clockwise_Positive,
                                NeutralModeValue.Coast,
                                new CurrentLimits(40, 80));

                public static final LoggedTunableNumber tIdleTuningVoltage = new LoggedTunableNumber(
                                "Intake/Voltage/IDLE",
                                0.0);
                public static final LoggedTunableNumber tIntakeTuningVoltage = new LoggedTunableNumber(
                                "Intake/Voltage/INTAKE",
                                11.014);
                public static final LoggedTunableNumber tOuttakeTuningVoltage = new LoggedTunableNumber(
                                "Intake/Voltage/OUTTAKE", -10.014);
                public static final LoggedTunableNumber tRollerTuningVoltage = new LoggedTunableNumber(
                                "Intake/Voltage/TUNING",
                                0.0);
                public static final HashMap<IntakeRollerState, LoggedTunableNumber> kStateToIntakeVoltage = new HashMap<>();

                static {
                        kStateToIntakeVoltage.put(IntakeRollerState.IDLE, tIdleTuningVoltage);
                        kStateToIntakeVoltage.put(IntakeRollerState.INTAKE, tIntakeTuningVoltage);
                        kStateToIntakeVoltage.put(IntakeRollerState.OUTTAKE, tOuttakeTuningVoltage);
                        kStateToIntakeVoltage.put(IntakeRollerState.TUNING, tRollerTuningVoltage);
                }
        }
}
