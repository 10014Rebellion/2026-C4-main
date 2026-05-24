// REBELLION 10014

package frc.robot.logging;

import frc.lib.telemetry.TelemetryConstants.Severity;
import frc.lib.telemetry.TelemetryError;

public class DriveErrors {
    public record ProfileExponentZero(int pDefaultExponent) implements TelemetryError {
        @Override
        public String message() {
            return "DRIVE EXPONENT SET TO ZERO, FIX THE PROFILE, DEFAULTING TO input^" + pDefaultExponent;
        }

        @Override
        public Severity severity() {
            return Severity.ERROR;
        }
    }

    public record PPGuiLoadFailed() implements TelemetryError {
        @Override
        public String message() {
            return "FAILED TO LOAD ROBOT CONFIG FROM PATHPLANNER GUI";
        }

        @Override
        public Severity severity() {
            return Severity.ERROR;
        }
    }

    public record PPCacheSaveFailed() implements TelemetryError {
        @Override
        public String message() {
            return "FAILED TO SAVE PATHPLANNER GUI SETTINGS TO CACHE";
        }

        @Override
        public Severity severity() {
            return Severity.WARNING;
        }
    }

    public record PPConfigLoadedFromCache() implements TelemetryError {
        @Override
        public String message() {
            return "LOADED ROBOT CONFIG FROM LOCAL CACHE (MAY BE OUT OF DATE)";
        }

        @Override
        public Severity severity() {
            return Severity.WARNING;
        }
    }

    public record PPConfigCacheLoadFailed() implements TelemetryError {
        @Override
        public String message() {
            return "FAILED TO LOAD ROBOT CONFIG FROM LOCAL CACHE";
        }

        @Override
        public Severity severity() {
            return Severity.ERROR;
        }
    }

    public record PPConfigLoadedFromDefault() implements TelemetryError {
        @Override
        public String message() {
            return "LOADED ROBOT CONFIG FROM DEFAULT DEPLOY FILE";
        }

        @Override
        public Severity severity() {
            return Severity.WARNING;
        }
    }

    public record PPConfigDefaultLoadFailed() implements TelemetryError {
        @Override
        public String message() {
            return "FAILED TO LOAD ROBOT CONFIG FROM DEFAULT DEPLOY FILE";
        }

        @Override
        public Severity severity() {
            return Severity.FATAL;
        }
    }

    public record WrongHeadingState() implements TelemetryError {
        @Override
        public String message() {
            return "HEADING STATE NOT APPLIED, WILL NOT HEADING ALIGN";
        }

        @Override
        public Severity severity() {
            return Severity.ERROR;
        }
    }
}
