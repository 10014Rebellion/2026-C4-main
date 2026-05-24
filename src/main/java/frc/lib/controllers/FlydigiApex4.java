package frc.lib.controllers;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class FlydigiApex4 extends CommandXboxController {
    private final int mPort;

    public FlydigiApex4(int pDriverStationPort) {
        super(pDriverStationPort);
        mPort = pDriverStationPort;
    }

    public Trigger selectButton() {
        return super.button(7);
    }

    public Trigger startButton() {
        return super.button(8);
    }

    public Rotation2d getPOVAngle() {
        double povAngle = super.getHID().getPOV();
        if(povAngle == -1) DriverStation.reportError("<<< FATAL: NO POV WAS DETECTED ON CONTROLLER PORT: " + mPort + " >>>", true);
        return Rotation2d.fromDegrees(povAngle);
    }
}
