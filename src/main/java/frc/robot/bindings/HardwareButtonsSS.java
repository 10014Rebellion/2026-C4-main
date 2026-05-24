package frc.robot.bindings;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;

public class HardwareButtonsSS extends SubsystemBase{
    DigitalInputButtonIO climbButtonHardwareIO = (!RobotConstants.isSim()) ? new DigitalInputButtonLS() : new DigitalInputButtonIO() {};

    DigitalInputButtonIOInputsAutoLogged inputs = new DigitalInputButtonIOInputsAutoLogged();

    public HardwareButtonsSS() {
    }

    @Override
    public void periodic() {
        climbButtonHardwareIO.updateInputs(inputs);
        Logger.processInputs("HardwareButtonsSS", inputs);
    }

    public DigitalInputButtonIOInputsAutoLogged getClimbButtonUpdateInputs() {
        return inputs;
    }
}
