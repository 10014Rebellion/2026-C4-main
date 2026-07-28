package frc.robot.systems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;

public class LEDss extends SubsystemBase {

    private final CANrange canRange;
    private final CANdle candle;

    private static final double MIN_DISTANCE = 0.05;
    private static final double MAX_DISTANCE = 0.50;

    private static final int LED_START = 0;
    private static final int LED_END = 80;

    private static final RGBWColor RED_COLOR = new RGBWColor(255, 0, 0);
    private static final RGBWColor BLUE_COLOR = new RGBWColor(0, 0, 255);
    private static final RGBWColor YELLOW_COLOR = new RGBWColor(255, 255, 0);

    public LEDss(CANBus canivore) {
        canRange = new CANrange(48, canivore);
        candle = new CANdle(30, canivore);

        CANdleConfiguration config = new CANdleConfiguration();
        config.LED.StripType = StripTypeValue.GRB;
        config.LED.BrightnessScalar = 1.0;
        candle.getConfigurator().apply(config);
    }

    private RGBWColor getAllianceColor() {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
            return BLUE_COLOR;
        }
        return RED_COLOR;
    }

    private RGBWColor lerpColor(RGBWColor a, RGBWColor b, double t) {
        t = MathUtil.clamp(t, 0.0, 1.0);
        int r = (int) (a.Red + (b.Red - a.Red) * t);
        int g = (int) (a.Green + (b.Green - a.Green) * t);
        int bl = (int) (a.Blue + (b.Blue - a.Blue) * t);
        return new RGBWColor(r, g, bl);
    }

    private boolean isHubActive() {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) return false;

        double matchTime = DriverStation.getMatchTime();
        if (matchTime < 0) return false;

        String gameData = DriverStation.getGameSpecificMessage();
        if (gameData.isEmpty()) return true;

        boolean redInactiveFirst = gameData.charAt(0) == 'R';
        boolean ownAllianceInactiveFirst =
                (alliance.get() == DriverStation.Alliance.Red) == redInactiveFirst;

        if (matchTime > 130) return true;
        else if (matchTime > 105) return !ownAllianceInactiveFirst;
        else if (matchTime > 80)  return ownAllianceInactiveFirst;
        else if (matchTime > 55)  return !ownAllianceInactiveFirst;
        else if (matchTime > 30)  return ownAllianceInactiveFirst;
        else return true;
    }

    @Override
    public void periodic() {
        boolean isDetected = canRange.getIsDetected().getValue();
        double distanceMeters = canRange.getDistance().getValueAsDouble();

        boolean hubActive = isHubActive(); //change isHubActive to true to make the led lights yellow

        SmartDashboard.putBoolean("CANrange IsDetected", isDetected);
        SmartDashboard.putNumber("CANrange Distance", distanceMeters);
        SmartDashboard.putBoolean("Hub Active", hubActive);

        RGBWColor baseColor = hubActive ? YELLOW_COLOR : getAllianceColor();
        RGBWColor finalColor;

        if (!isDetected) {
            finalColor = baseColor;
        } else {
            double clamped = MathUtil.clamp(distanceMeters, MIN_DISTANCE, MAX_DISTANCE);
            double blendFactor = 1.0 - ((clamped - MIN_DISTANCE) / (MAX_DISTANCE - MIN_DISTANCE));
            finalColor = lerpColor(baseColor, YELLOW_COLOR, blendFactor);
        }

        candle.setControl(new SolidColor(LED_START, LED_END).withColor(finalColor));
    }
}
