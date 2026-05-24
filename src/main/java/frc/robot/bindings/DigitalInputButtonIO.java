package frc.robot.bindings;

import org.littletonrobotics.junction.AutoLog;

public interface DigitalInputButtonIO {
    @AutoLog
    public static class DigitalInputButtonIOInputs {
        public boolean iPressed = false;
    }

    public default void updateInputs(DigitalInputButtonIOInputs pInputs) {}
}
