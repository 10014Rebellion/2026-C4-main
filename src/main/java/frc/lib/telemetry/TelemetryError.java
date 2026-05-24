// REBELLION 10014

package frc.lib.telemetry;

import frc.lib.telemetry.TelemetryConstants.Severity;

public interface TelemetryError {
    String message();

    Severity severity();
}
