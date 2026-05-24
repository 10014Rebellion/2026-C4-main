// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.lib;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;

import frc.lib.telemetry.Telemetry;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class PhoenixUtil {
    public static enum CanivoreBus {
        UNDERWORLD,
        OVERWORLD,
        RIO
    }

    /** Attempts to run the command until no error is produced. */
    public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
        for (int i = 0; i < maxAttempts; i++) {
            var error = command.get();
            if (error.isOK()) break;
        }
    }

    /** Signals for synchronized refresh. */
    private static BaseStatusSignal[] underworldSignals = new BaseStatusSignal[0];
    private static BaseStatusSignal[] overworldSignals = new BaseStatusSignal[0];

    private static BaseStatusSignal[] rioSignals = new BaseStatusSignal[0];

    /** Registers a set of signals for synchronized refresh. */
    public static void registerSignals(CanivoreBus bus, BaseStatusSignal... signals) {
        switch (bus) {
            case OVERWORLD:
                BaseStatusSignal[] newSignals1 = new BaseStatusSignal[overworldSignals.length + signals.length];
                System.arraycopy(overworldSignals, 0, newSignals1, 0, overworldSignals.length);
                System.arraycopy(signals, 0, newSignals1, overworldSignals.length, signals.length);
                overworldSignals = newSignals1;
                break;

            case UNDERWORLD:
                BaseStatusSignal[] newSignals2 = new BaseStatusSignal[underworldSignals.length + signals.length];
                System.arraycopy(underworldSignals, 0, newSignals2, 0, underworldSignals.length);
                System.arraycopy(signals, 0, newSignals2, underworldSignals.length, signals.length);
                underworldSignals = newSignals2;
                break;

            case RIO:
                BaseStatusSignal[] newSignals3 = new BaseStatusSignal[rioSignals.length + signals.length];
                System.arraycopy(rioSignals, 0, newSignals3, 0, rioSignals.length);
                System.arraycopy(signals, 0, newSignals3, rioSignals.length, signals.length);
                rioSignals = newSignals3;
                break;
        
            default:
                Telemetry.reportIssue(null);
                break;
        }
    }

    /** Refresh all registered signals. */
    public static void refreshAll() {
        if (underworldSignals.length > 0) {
            BaseStatusSignal.refreshAll(underworldSignals);
        }

        if (overworldSignals.length > 0) {
            BaseStatusSignal.refreshAll(overworldSignals);
        }

        if (rioSignals.length > 0) {
            StatusCode code = BaseStatusSignal.refreshAll(rioSignals);
            Logger.recordOutput("Rio/CAN/Error", code.getName());
        }
    }

}
