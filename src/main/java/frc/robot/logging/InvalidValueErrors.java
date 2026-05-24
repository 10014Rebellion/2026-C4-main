package frc.robot.logging;

import frc.lib.telemetry.TelemetryConstants.Severity;
import frc.lib.telemetry.TelemetryError;

public class InvalidValueErrors {
    public record UnaccountedEnum(String pInvalidEnumName) implements TelemetryError {
        @Override
        public String message() {
            return "UNACCOUNTED ENUM: \"" + pInvalidEnumName + "\". ACCOUNT FOR THIS ENUM PROPERLY TO PREVENT UNINTENDED CONSEQUENCES!";
        }

        @Override
        public Severity severity() {
            return Severity.ERROR;
        }
    }
}
