// REBELLION 10014

package frc.robot.systems.drive.controllers;

import frc.lib.tuning.LoggedTunableNumber;
import frc.robot.systems.drive.controllers.ManualTeleopController.DriverProfiles;

public class TuneableDriverProfile {
    private LoggedTunableNumber mLinearScalar;
    private LoggedTunableNumber mLinearDeadBand;
    private LoggedTunableNumber mLinearInputsExponent;
    private LoggedTunableNumber mRotationScalar;
    private LoggedTunableNumber mRotationInputsExponent;
    private LoggedTunableNumber mRotationDeadband;
    private LoggedTunableNumber mSniperControl;

    public TuneableDriverProfile(
            String pKey,
            double pDefaultLinearScalar,
            double pDefaultLinearDeadband,
            double pDefaultLinearInputsExponent,
            double pDefaultRotationScalar,
            double pDefaultRotationInputExponent,
            double pDefaultRotationDeadband,
            double pDefaultSniperControl) {
        mLinearScalar = new LoggedTunableNumber("Drive/Teleop/" + pKey + "/LinearScalar", pDefaultLinearScalar);
        mLinearDeadBand = new LoggedTunableNumber("Drive/Teleop/" + pKey + "/LinearDeadband", pDefaultLinearDeadband);
        mLinearInputsExponent =
            new LoggedTunableNumber("Drive/Teleop/" + pKey + "/LinearInputsExponent", pDefaultLinearInputsExponent);
        mRotationScalar = new LoggedTunableNumber("Drive/Teleop/" + pKey + "/RotationScalar", pDefaultRotationScalar);
        mRotationInputsExponent = new LoggedTunableNumber(
            "Drive/Teleop/" + pKey + "/RotationInputsExponent", pDefaultRotationInputExponent);
        mRotationDeadband =
            new LoggedTunableNumber("Drive/Teleop/" + pKey + "/RotationDeadband", pDefaultRotationDeadband);
        mSniperControl = new LoggedTunableNumber("Drive/Teleop/" + pKey + "/SniperControl", pDefaultSniperControl);
    }

    public TuneableDriverProfile(DriverProfiles pDefaults) {
        this(
            pDefaults.key(),
            pDefaults.linearScalar(),
            pDefaults.linearDeadband(),
            pDefaults.linearExponent(),
            pDefaults.rotationalScalar(),
            pDefaults.rotationalScalar(),
            pDefaults.rotationDeadband(),
            pDefaults.sniperScalar()
        );
    }

    public LoggedTunableNumber linearScalar() {
        return mLinearScalar;
    }

    public LoggedTunableNumber linearDeadBand() {
        return mLinearDeadBand;
    }

    public LoggedTunableNumber linearInputsExponent() {
        return mLinearInputsExponent;
    }

    public LoggedTunableNumber rotationScalar() {
        return mRotationScalar;
    }

    public LoggedTunableNumber rotationInputsExponent() {
        return mRotationInputsExponent;
    }

    public LoggedTunableNumber rotationDeadband() {
        return mRotationDeadband;
    }

    public LoggedTunableNumber sniperControl() {
        return mSniperControl;
    }
}
