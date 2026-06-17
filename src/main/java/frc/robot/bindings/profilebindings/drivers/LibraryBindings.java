package frc.robot.bindings.profilebindings.drivers;

import frc.robot.bindings.BindingCommands;
import frc.robot.bindings.profilebindings.ProfileBindings;

public class LibraryBindings implements ProfileBindings 
{
    private BindingCommands mCommands;

    public LibraryBindings(){}

    public LibraryBindings(BindingCommands pCommands) {
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