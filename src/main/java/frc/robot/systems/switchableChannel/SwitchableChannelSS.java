package frc.robot.systems.switchableChannel;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwitchableChannelSS extends SubsystemBase{
    private final PowerDistribution mPD;

    public SwitchableChannelSS() {
        mPD = new PowerDistribution();
    }

    public void enableSwitchableChannel() {
        mPD.setSwitchableChannel(true);
    }

}
