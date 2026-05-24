package frc.robot.logging;

import frc.lib.telemetry.TelemetryConstants.Severity;
import frc.lib.telemetry.TelemetryError;

public class MotorErrors {
    public record SettingControlToFollower(Object pMotorClass) implements TelemetryError {
        @Override
        public String message() {
            return "A FOLLOWER MOTOR IS BEING REQUESTED TO ROTATE IN " + pMotorClass.getClass().getSimpleName() + ". BAD! SET CONTROL TO LEADER INSTEAD!";
        }

        @Override
        public Severity severity() {
            return Severity.WARNING;
        }
    }

    public record EnforcingLeaderAsFollower(Object pMotorClass) implements TelemetryError {
        @Override
        public String message() {
            return "A LEADER MOTOR IS BEING REQUESTED TO ENFORCE FOLLOWING " + pMotorClass.getClass().getSimpleName() + ". THIS MOTOR IS NOT A FOLLOWER! STOP ENFORCING FOLLOWER MODE!";
        }

        @Override
        public Severity severity() {
            return Severity.WARNING;
        }
    }
}
