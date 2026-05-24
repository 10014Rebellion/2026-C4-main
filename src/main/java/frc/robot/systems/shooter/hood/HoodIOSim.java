package frc.robot.systems.shooter.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.hardware.HardwareRecords.ArmControllerMotionMagic;
import frc.lib.hardware.HardwareRecords.BasicMotorHardware;

public class HoodIOSim implements HoodIO {

    private final SingleJointedArmSim mHoodSim;
    private final ProfiledPIDController mHoodController;
    private double mAppliedVolts = 0.0;


    public HoodIOSim(BasicMotorHardware pHardware, ArmControllerMotionMagic pController) {
        mHoodSim = new SingleJointedArmSim(
            DCMotor.getKrakenX44Foc(1), 
            pHardware.rotorToMechanismRatio(), 
            SingleJointedArmSim.estimateMOI(HoodConstants.kHoodLength, HoodConstants.kHoodMass), 
            HoodConstants.kHoodLength, 
            HoodConstants.kHoodLimits.backwardLimit().getRadians(), 
            HoodConstants.kHoodLimits.forwardLimit().getRadians(), 
            true, 
            0.0, 
            0.0001, 0.0001);

        mHoodController = 
            new ProfiledPIDController(
                HoodConstants.kHoodControlConfig.pdController().kP(), 
                0.0, 
                HoodConstants.kHoodControlConfig.pdController().kD(), 
                new TrapezoidProfile.Constraints(
                    pController.motionMagicConstants().maxVelocity(), 
                    pController.motionMagicConstants().maxAcceleration()));
    }

    public Rotation2d getPos(){
        return Rotation2d.fromRadians(mHoodSim.getAngleRads());
    }

    public void updateInputs(HoodInputs pInputs) {
        mHoodSim.update(0.02);
        pInputs.iIsHoodConnected = true;
        pInputs.iHoodAngle = getPos();
        pInputs.iHoodVelocityRotPS = Rotation2d.fromRotations(mHoodSim.getVelocityRadPerSec() / (2 * Math.PI));
        pInputs.iHoodAccelerationRPSS = Rotation2d.kZero;
        pInputs.iHoodMotorVolts = mAppliedVolts;
        pInputs.iHoodSupplyCurrentAmps = 0.0;
        pInputs.iHoodStatorCurrentAmps = mHoodSim.getCurrentDrawAmps();
        pInputs.iHoodTempCelsius = 0.0;
        pInputs.iHoodReferenceValue = Rotation2d.fromRotations(mHoodController.getSetpoint().position);
        pInputs.iHoodReferenceValueSlope = Rotation2d.fromRotations(mHoodController.getSetpoint().velocity);
    }

    public void setPDConstants(double pKP, double pKD) {
        mHoodController.setPID(pKP, 0.0, pKD);
    }

    @Override
    public void setMotionMagicConstants(double pCruiseVelRPS, double pMaxAccelRPS2, double pMaxJerkRPS3) {
        mHoodController.setConstraints(new TrapezoidProfile.Constraints(pCruiseVelRPS, pMaxAccelRPS2));
    }

    public void resetPPID() {
        mHoodController.reset(getPos().getRotations());
    }

    public void setMotorPosition(Rotation2d pRotationSP, double pArbFF) {
        setMotorVolts(mHoodController.calculate(getPos().getRotations(), pRotationSP.getRotations()) + pArbFF);
    }

    public void setMotorVolts(double pVolts) {
        mAppliedVolts = MathUtil.clamp(pVolts, -12, 12);
        mHoodSim.setInputVoltage(pVolts);
    }

    public void setMotorAmps(double pAmps) {}

    public void stopMotor() {
        setMotorVolts(0.0);
    }

    
}
