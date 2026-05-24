// REBELLION 10014

package frc.robot.bindings;

import frc.robot.bindings.profilebindings.drivers.*;
import frc.robot.systems.drive.controllers.ManualTeleopController.DriverProfiles;

public class BindingsConstants {
    public static final int kPilotControllerPort = 0;
    public static final int kGunnerControllerPort = 3;


    public static final DriverProfiles kDefaultProfile =
        new DriverProfiles(
            "Default", // Name
            1.0, // Linear Scalar
            3,  // Linear Exponent
            0.05, // Left Joystick Deadband
            1.0, // Rotational Scalar
            3.0, // Rotational Exponent
            0.05, // Right Joystick Deadband
            0.2, // Sniper Scalar
            new DefaultBindings()
        );

    public static DriverProfiles[] kProfiles = {
        new DriverProfiles(
            "Bosco", // Name
            0.8, // Linear Scalar
            4.0, // Linear Exponent
            0.05, // Left Joystick Deadband
            1.0, // Rotational Scalar
            4.0, // Rotational Exponent
            0.05, //     Right Joystick Deadband
            0.2, // Sniper Scalar
            new BoscoBindings()
        ),

        new DriverProfiles(
            "Eli", // Name
            1.0, // Linear Scalar
            3,  // Linear Exponent
            0.075, // Left Joystick Deadband
            1.0, // Rotational Scalar
            3.0, // Rotational Exponent
            0.1, // Right Joystick Deadband
            0.2, // Sniper Scalar,
            new EliBindings()
        ),

        new DriverProfiles(
            "Taha", // Name
            1.0, // Linear Scalar
            3,  // Linear Exponent
            0.05, // Left Joystick Deadband
            1.0, // Rotational Scalar
            3.0, // Rotational Exponent
            0.1, // Right Joystick Deadband
            0.2, // Sniper Scalar
            new TahaBindings()
        ),
    };
}
