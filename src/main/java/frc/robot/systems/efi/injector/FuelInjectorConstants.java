package frc.robot.systems.efi.injector;

import java.util.HashMap;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.lib.hardware.HardwareRecords.BasicMotorHardware;
import frc.lib.hardware.HardwareRecords.CurrentLimits;
import frc.lib.tuning.LoggedTunableNumber;
import frc.robot.RobotConstants;
import frc.robot.systems.efi.FuelInjectorSS;
import frc.robot.systems.efi.FuelInjectorSS.FuelInjectorState;


public class FuelInjectorConstants {

    public static final BasicMotorHardware kFuelInjectorConfig = new BasicMotorHardware(
        43, 
        RobotConstants.kSubsystemsCANBus,
        1, 
        InvertedValue.Clockwise_Positive, 
        NeutralModeValue.Brake, 
        new CurrentLimits(40, 80));

    public static final LoggedTunableNumber tIdleTuningVoltage = new LoggedTunableNumber("FuelInjector/Voltage/IDLE", 0.0);
    public static final LoggedTunableNumber tOuttakeTuningVoltage = new LoggedTunableNumber("FuelInjector/Voltage/OUTTAKE", -10);
    public static final LoggedTunableNumber tIntakeTuningVoltage = new LoggedTunableNumber("FuelInjector/Voltage/INTAKE", 7);
    public static final LoggedTunableNumber tFuelInjectorTuningVoltage = new LoggedTunableNumber("FuelInjector/Voltage/TUNING", 0.0);
    public static final LoggedTunableNumber tFuelInjectorKickbackVoltage = new LoggedTunableNumber("FuelInjector/Voltage/KICKBACK", -2.0);
    public static final HashMap<FuelInjectorState, LoggedTunableNumber> kStateToInjectorVoltage = new HashMap<>();

        static {
            kStateToInjectorVoltage.put(FuelInjectorState.IDLE, tIdleTuningVoltage);
            kStateToInjectorVoltage.put(FuelInjectorState.KICKBACK, tFuelInjectorKickbackVoltage);
            kStateToInjectorVoltage.put(FuelInjectorState.INTAKE, tIntakeTuningVoltage);
            kStateToInjectorVoltage.put(FuelInjectorState.OUTTAKE, tOuttakeTuningVoltage);
            kStateToInjectorVoltage.put(FuelInjectorState.TUNING, tFuelInjectorTuningVoltage);
        }
    
}
