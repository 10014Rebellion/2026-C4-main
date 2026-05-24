package frc.robot.bindings.profilebindings.drivers;

import frc.robot.bindings.BindingCommands;
import frc.robot.bindings.profilebindings.ProfileBindings;

public class TahaBindings implements ProfileBindings {
    private BindingCommands mCommands;

    public TahaBindings(){}

    public TahaBindings(BindingCommands pCommands) {
        this.mCommands = pCommands;
    }

    @Override
    public void initAllBindings() {}

    @Override
    public void initDriveBindings() {}

    @Override
    public void initShooterBindings() {}

    @Override
    public void initIntakeBindings() {}

    @Override
    public void initClimbBindings() {}
}
