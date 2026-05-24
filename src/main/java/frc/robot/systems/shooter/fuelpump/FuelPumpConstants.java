package frc.robot.systems.shooter.fuelpump;

import java.util.HashMap;
import java.util.function.Supplier;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.hardware.HardwareRecords.BasicMotorHardware;
import frc.lib.hardware.HardwareRecords.CurrentLimits;
import frc.lib.hardware.HardwareRecords.FollowerMotorHardware;
import frc.lib.hardware.HardwareRecords.PDConstants;
import frc.lib.hardware.HardwareRecords.SimpleController;
import frc.lib.tuning.LoggedTunableNumber;
import frc.robot.RobotConstants;
import frc.robot.systems.shooter.fuelpump.FuelPumpSS.FuelPumpState;

public class FuelPumpConstants {
    public static final double kToleranceRPS = 3.0;
    public static final Rotation2d kRPSForShooting = Rotation2d.fromRotations(50);

    public static final BasicMotorHardware kFuelPumpLeaderConfig = new BasicMotorHardware(
        53,
        RobotConstants.kSubsystemsCANBus,
        1,
        InvertedValue.CounterClockwise_Positive,
        NeutralModeValue.Coast,
        new CurrentLimits(40, 80)
    );

    public static final FollowerMotorHardware kFuelPumpFollowerConfig = new FollowerMotorHardware(
        54,
        kFuelPumpLeaderConfig,
        MotorAlignmentValue.Opposed
    );

    public static final SimpleController kFuelPumpControlConfig = new SimpleController(
        0, 
        new PDConstants(0.22, 0), 
        new SimpleMotorFeedforward(0.35, 0.0989)
    );

    public static final HashMap<FuelPumpState, LoggedTunableNumber> kStateToTuneableFuelPumpVolts = new HashMap<FuelPumpState, LoggedTunableNumber>();
    public static final HashMap<FuelPumpState, Supplier<Rotation2d>> kStateToTuneableFuelPumpVelocity = new HashMap<FuelPumpState, Supplier<Rotation2d>>();

    public static final LoggedTunableNumber tTuningVoltage = new LoggedTunableNumber("Shooter/FuelPump/TuneVoltage", 0.0);
    public static final LoggedTunableNumber tIntakeVolts  = new LoggedTunableNumber("Shooter/FuelPump/DesiredVolts/IntakeVolts", 6);
    public static final LoggedTunableNumber tKickbackVolts  = new LoggedTunableNumber("Shooter/FuelPump/DesiredVolts/IntakeVolts", -3);

    public static final LoggedTunableNumber tOuttakeVolts  = new LoggedTunableNumber("Shooter/FuelPump/DesiredVolts/OuttakeVolts", -10.014);
    public static final LoggedTunableNumber tSlightOuttakeVolts = new LoggedTunableNumber("Shooter/FuelPump/DesiredVolts/SlightOuttakeVolts", -2.0);
    public static final LoggedTunableNumber tIntakeVelocity = new LoggedTunableNumber("Shooter/FuelPump/DesiredVelocity/IntakeVelocity", 40.0);

    static {
        kStateToTuneableFuelPumpVolts.put(FuelPumpState.INTAKE_VOLT, tIntakeVolts);
        kStateToTuneableFuelPumpVolts.put(FuelPumpState.KICKBACK_VOLT, tKickbackVolts);
        kStateToTuneableFuelPumpVolts.put(FuelPumpState.OUTTAKE_VOLT, tOuttakeVolts);
        kStateToTuneableFuelPumpVolts.put(FuelPumpState.SLIGHT_OUTTAKE_VOLT, tSlightOuttakeVolts);
        kStateToTuneableFuelPumpVelocity.put(FuelPumpState.INTAKE_VELOCITY, () -> Rotation2d.fromRotations(tIntakeVelocity.get()));
    }
}
