package frc.robot.bindings;

import java.util.function.DoubleSupplier;

import frc.robot.RobotConstants;

public class DriverTransition {
    private final double kSwitchTimeSec = 3;

    private final double kStepAmount = 1.0 / ((int) Math.ceil(kSwitchTimeSec / RobotConstants.kPeriodicSec));
    private double kDefenderFactor = 0.0;
    private DriverType mActiveDriver = DriverType.SCORER;
    private DriverType mDesiredDriver = DriverType.SCORER;

    public enum DriverType {
        DEFENDER,
        SCORER
    } 

    public DriverTransition() {}

    public void hardSetDriverAs(DriverType pDriverToSwitchTo) {
        if(pDriverToSwitchTo == DriverType.DEFENDER) {
            kDefenderFactor = 1;
            mActiveDriver = DriverType.DEFENDER;
            mDesiredDriver = DriverType.DEFENDER;
        } else {
            kDefenderFactor = 0;
            mActiveDriver = DriverType.SCORER;
            mDesiredDriver = DriverType.SCORER;
        }
    }

    public void switchTo(DriverType pDriverToSwitchTo) {
        if(mActiveDriver != pDriverToSwitchTo) {
            mDesiredDriver = pDriverToSwitchTo;
        }
    }

    /*
     * Should be fetched periodically during trigger (supplier)
     */
    public DoubleSupplier getScorerFactorSupplier() {
        return () -> 1.0 - kDefenderFactor;
    }

    /*
     * Should be fetched periodically during trigger (supplier)
     */
    public DoubleSupplier getDefenderFactorSupplier() {
        return () -> kDefenderFactor;
    }

    public void periodic() {
        if(mActiveDriver != mDesiredDriver) {
            if(mDesiredDriver == DriverType.DEFENDER) {
                kDefenderFactor = kDefenderFactor + kStepAmount;
                if(kDefenderFactor >= 1) {
                    kDefenderFactor = 1;
                    mActiveDriver = DriverType.DEFENDER;
                }
            }  else { // If desired is scorer
                kDefenderFactor = kDefenderFactor - kStepAmount;

                if(kDefenderFactor <= 0) {
                    kDefenderFactor = 0;
                    mActiveDriver = DriverType.SCORER;
                }
            }
        }
    }
}
