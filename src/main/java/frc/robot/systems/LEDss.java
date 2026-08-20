package frc.robot.systems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.game.TransitionTracker;

import java.util.Optional;

public class LEDss extends SubsystemBase {

    public static record LEDHardware(
        int canRangeID,
        int candleID,
        CANBus canBus
    ) {}

    private final CANrange canRange;
    private final CANdle candle;
    private final TransitionTracker mTracker = new TransitionTracker();

    private static final int LED_START = 0;
    private static final int LED_END = 80;

    private static final RGBWColor RED_COLOR = new RGBWColor(255, 0, 0);
    private static final RGBWColor BLUE_COLOR = new RGBWColor(0, 0, 255);
    private static final RGBWColor YELLOW_COLOR = new RGBWColor(255, 255, 0);

    public LEDss(LEDHardware hardware) {
        canRange = new CANrange(hardware.canRangeID(), hardware.canBus());
        candle = new CANdle(hardware.candleID(), hardware.canBus());

        CANdleConfiguration config = new CANdleConfiguration();
        config.LED.StripType = StripTypeValue.GRB; // strip is wired GRB, not RGB
        config.LED.BrightnessScalar = 1.0;
        candle.getConfigurator().apply(config);
    }

    private RGBWColor getAllianceColor() {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
            return BLUE_COLOR;
        }else if(alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red){
            return RED_COLOR;
        }
        else{
            return YELLOW_COLOR;
        }
        
    }

    @Override
    public void periodic() {
        boolean isDetected = canRange.getIsDetected().getValue();
        double distanceMeters = canRange.getDistance().getValueAsDouble();

        boolean hubActive = false; //mTracker.isHubActive()

        SmartDashboard.putBoolean("CANrange IsDetected", isDetected);
        SmartDashboard.putNumber("CANrange Distance", distanceMeters);
        SmartDashboard.putBoolean("LED Hub Active", hubActive);

        RGBWColor finalColor = hubActive ? YELLOW_COLOR : getAllianceColor();

        candle.setControl(new SolidColor(LED_START, LED_END).withColor(finalColor));
    }
}