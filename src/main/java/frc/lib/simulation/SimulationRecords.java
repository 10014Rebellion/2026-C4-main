package frc.lib.simulation;

import edu.wpi.first.math.system.plant.DCMotor;

public class SimulationRecords {

    public static record SimulatedElevator(
        DCMotor kMotor,
        double kCarriageMassKg,
        double kDrumRadiusMeters,
        boolean kSimulateGravity,
        double kStartingHeightMeters,
        double kMeasurementStdDevs
    ) {}
}
