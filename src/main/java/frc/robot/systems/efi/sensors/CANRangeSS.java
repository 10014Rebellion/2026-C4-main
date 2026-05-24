package frc.robot.systems.efi.sensors;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.tuning.LoggedTunableNumber;

public class CANRangeSS extends SubsystemBase {
    private final SensorIO mLeftCANRange;
    private final SensorIO mCenterCANRange;
    private final SensorIO mRightCANRange;

    private final SensorInputsAutoLogged mLeftCANRangeInputs = new SensorInputsAutoLogged();
    private final SensorInputsAutoLogged mCenterCANRangeInputs = new SensorInputsAutoLogged();
    private final SensorInputsAutoLogged mRightCANRangeInputs = new SensorInputsAutoLogged();

    private final LoggedTunableNumber tRightPositionTolerance = new LoggedTunableNumber("GeneralSensors/CANRange/Right/Tolerance", 0.0);
    private final LoggedTunableNumber tRightPositionCutoff = new LoggedTunableNumber("GeneralSensors/CANRange/Right/Cutoff", 0.0);

    private final LoggedTunableNumber tCenterPositionTolerance = new LoggedTunableNumber("GeneralSensors/CANRange/Center/Tolerance", 0.0);
    private final LoggedTunableNumber tCenterPositionCutoff = new LoggedTunableNumber("GeneralSensors/CANRange/Center/Cutoff", 0.0);

    private final LoggedTunableNumber tLeftPositionTolerance = new LoggedTunableNumber("GeneralSensors/CANRange/Left/Tolerance", 0.0);
    private final LoggedTunableNumber tLeftPositionCutoff = new LoggedTunableNumber("GeneralSensors/CANRange/Left/Cutoff", 0.0);

    public CANRangeSS(SensorIO pLeftCANRange, SensorIO pCenterCANRange, SensorIO pRightCANRange) {
        this.mLeftCANRange = pLeftCANRange;
        this.mCenterCANRange = pCenterCANRange;
        this.mRightCANRange = pRightCANRange;
    }

    public boolean allHasFuel() {
        return leftHasFuel() && centerHasFuel() && rightHasFuel();
    }

    public boolean anyHasFuel() {
        return leftHasFuel() || centerHasFuel() || rightHasFuel();
    }

    public boolean leftHasFuel(){
        return hasFuel(mLeftCANRangeInputs);
    }

    public boolean rightHasFuel(){
        return hasFuel(mRightCANRangeInputs);
    }

    public boolean centerHasFuel(){
        return hasFuel(mCenterCANRangeInputs);
    }

    private boolean hasFuel(SensorInputsAutoLogged inputs) {
        return inputs.hasObject;
    }

    public void setSensorConfigs(SensorIO pCANRange, double pCutoff, double pTolerance){
        pCANRange.setSensorConfigs(pCutoff, pTolerance);
    }

    @Override
    public void periodic(){ 
        mLeftCANRange.updateInputs(mLeftCANRangeInputs);
        mCenterCANRange.updateInputs(mCenterCANRangeInputs);
        mRightCANRange.updateInputs(mRightCANRangeInputs);

        Logger.processInputs("GeneralSensors/CANRange/Left", mLeftCANRangeInputs);
        Logger.processInputs("GeneralSensors/CANRange/Mid", mCenterCANRangeInputs);
        Logger.processInputs("GeneralSensors/CANRange/Right", mRightCANRangeInputs);

        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> setSensorConfigs(mLeftCANRange, tLeftPositionCutoff.get(), tLeftPositionTolerance.get()),
            tLeftPositionCutoff, tLeftPositionTolerance);

        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> setSensorConfigs(mCenterCANRange, tCenterPositionCutoff.get(), tCenterPositionTolerance.get()),
            tCenterPositionCutoff, tCenterPositionTolerance);

        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> setSensorConfigs(mRightCANRange, tRightPositionCutoff.get(), tRightPositionTolerance.get()),
            tRightPositionCutoff, tRightPositionTolerance);
    }
}
