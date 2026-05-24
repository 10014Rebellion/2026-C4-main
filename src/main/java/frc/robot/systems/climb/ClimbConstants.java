package frc.robot.systems.climb;

import java.util.HashMap;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.RobotConstants;
import frc.robot.systems.climb.ClimbSS.ClimbState;
import frc.lib.hardware.HardwareRecords.BasicMotorHardware;
import frc.lib.hardware.HardwareRecords.CurrentLimits;
import frc.lib.hardware.HardwareRecords.PositionSoftLimits;
import frc.lib.simulation.SimulationRecords.SimulatedElevator;
import frc.lib.tuning.LoggedTunableNumber;

public class ClimbConstants {
    public static final int kHookPort = 4;

    public static final Rotation2d kHookOutPosition = Rotation2d.kCCW_Pi_2;
    public static final Rotation2d kHookInPosition = Rotation2d.kZero;    

    public static final LoggedTunableNumber kClimbUp = new LoggedTunableNumber("Climb/UpVoltage", 5.0);
    public static final LoggedTunableNumber kClimbDown = new LoggedTunableNumber("Climb/DownVoltage", -5.0);
    public static final LoggedTunableNumber kClimbStay = new LoggedTunableNumber("Climb/StayVoltage", 0.0);
    public static final LoggedTunableNumber kClimbStayRobot = new LoggedTunableNumber("Climb/StayRobotVoltage", -0.2);
    public static final LoggedTunableNumber kClimbIdle = new LoggedTunableNumber("Climb/IdleVoltage", 0.0);

    public static final HashMap<ClimbState, LoggedTunableNumber> kStateToVoltage = new HashMap<>();
    static {
        kStateToVoltage.put(ClimbState.UP, kClimbUp);
        kStateToVoltage.put(ClimbState.DOWN, kClimbDown);
        kStateToVoltage.put(ClimbState.STAY, kClimbStay);
        kStateToVoltage.put(ClimbState.STAY_ROBOT, kClimbStayRobot);
        kStateToVoltage.put(ClimbState.IDLE, kClimbIdle);
    }

    public static final double kClimbHeight = 3.0;//changes values to fix encoder readings: Arya
    public static final double kClimbedHeight = 0.13; //changed values to fix encoder readings: Arya
    
    public static final BasicMotorHardware kClimbMotorConstants = new BasicMotorHardware(
        60, // Motor ID // TODO: TUNE ME!
        RobotConstants.kSubsystemsCANBus, 
        16.0, // Rotor to Mechanism Ratio // TODO: TUNE ME!
        InvertedValue.Clockwise_Positive,
        NeutralModeValue.Brake,
        new CurrentLimits(30, 40)
    );
    
    public static final PositionSoftLimits kSoftLimits = new PositionSoftLimits(
        0.00, 
        3.0
    );

    public static final SimulatedElevator kSimElevator = new SimulatedElevator(
        DCMotor.getKrakenX60(1), 
        9, 
        0.05, 
        true, 
        0.0, 
        0.001
    );
}