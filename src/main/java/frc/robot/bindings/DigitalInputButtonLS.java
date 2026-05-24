package frc.robot.bindings;
import edu.wpi.first.wpilibj.DigitalInput;

public class DigitalInputButtonLS implements DigitalInputButtonIO {
    DigitalInput mClimbNeutralModeButton = new DigitalInput(0);

    public DigitalInputButtonLS() {}
    
    @Override
    public void updateInputs(DigitalInputButtonIOInputs pInputs) {
        pInputs.iPressed = mClimbNeutralModeButton.get();
    }
}
