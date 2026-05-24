package frc.robot.systems.efi.sensors;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.MeasurementHealthValue;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.lib.PhoenixUtil;
import frc.lib.PhoenixUtil.CanivoreBus;
import frc.lib.hardware.HardwareRecords.CANRangeConfiguration;

public class SensorIOCANRange implements SensorIO{

    private final CANrange mCANRange;

    private final StatusSignal<Distance> distanceFromFuel;
    private final StatusSignal<Boolean> isDetected;
    private final StatusSignal<Distance> distanceFromFuelStdev;
    private final StatusSignal<Double> ambience;
    private final StatusSignal<Double> signalStrength;
    private final StatusSignal<MeasurementHealthValue> measurementHealth;
    private final StatusSignal<Time> measurementTime;

    public SensorIOCANRange(CANRangeConfiguration configuration){
        mCANRange = new CANrange(configuration.canRangeID());

        CANrangeConfiguration canRangeConfig = new CANrangeConfiguration();

        canRangeConfig.ProximityParams.MinSignalStrengthForValidMeasurement = 5500;
        canRangeConfig.ProximityParams.ProximityHysteresis = configuration.positionTolerance();
        canRangeConfig.ProximityParams.ProximityThreshold = configuration.fuelDetectionCutoff();

        mCANRange.getConfigurator().apply(canRangeConfig);

        // Intialize Status Signals
        distanceFromFuel = mCANRange.getDistance();
        isDetected = mCANRange.getIsDetected();
        distanceFromFuelStdev = mCANRange.getDistanceStdDev();
        ambience = mCANRange.getAmbientSignal();
        signalStrength = mCANRange.getSignalStrength();
        measurementHealth = mCANRange.getMeasurementHealth();
        measurementTime = mCANRange.getMeasurementTime();

        mCANRange.optimizeBusUtilization(0.0);

        PhoenixUtil.registerSignals(
            CanivoreBus.RIO, 
            distanceFromFuel,
            isDetected, 
            distanceFromFuelStdev,
            ambience,
            signalStrength,
            measurementHealth,
            measurementTime);
    }

    @Override
    public void updateInputs(SensorInputs inputs){
        inputs.distanceFromSensor = distanceFromFuel.getValueAsDouble();
        inputs.distanceFromSensorStdDev = distanceFromFuelStdev.getValueAsDouble();
        inputs.hasObject = isDetected.getValue();
        inputs.ambience = ambience.getValueAsDouble();
        inputs.signalStrength = signalStrength.getValueAsDouble();
        inputs.isMeasurementHealthGood = measurementHealth.equals(MeasurementHealthValue.Good);
        inputs.measurementTime = measurementTime.getValueAsDouble();
        
        inputs.isSensorConnected = BaseStatusSignal.refreshAll(
            distanceFromFuel,
            distanceFromFuelStdev,
            isDetected,
            ambience,
            signalStrength,
            measurementHealth,
            measurementTime).isOK();
    }

    @Override
    public void setSensorConfigs(double pTolerance, double pCutoff) {
        CANrangeConfiguration canRangeConfig = new CANrangeConfiguration();
        canRangeConfig.ProximityParams.MinSignalStrengthForValidMeasurement = 3000;
        canRangeConfig.ProximityParams.ProximityHysteresis = pTolerance;
        canRangeConfig.ProximityParams.ProximityThreshold = pCutoff;
        mCANRange.getConfigurator().apply(canRangeConfig);
    }

    @Override
    public boolean getCANRangeValue(){
        return isDetected.getValue();
    }
    
}