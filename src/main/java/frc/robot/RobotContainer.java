package frc.robot;

import static frc.robot.systems.drive.DriveConstants.*;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.bindings.BindingsConstants;
import frc.robot.bindings.ButtonBindings;
import frc.robot.systems.drive.Drive;
import frc.robot.systems.drive.GyroIO;
import frc.robot.systems.drive.GyroIOPigeon2;
import frc.robot.systems.drive.ModuleIO;
import frc.robot.systems.drive.ModuleIOSim;
import frc.robot.systems.drive.ModuleIOTalonFX;
import frc.robot.systems.drive.TunerConstants;
import frc.robot.systems.drive.controllers.ManualTeleopController.DriverProfiles;
import frc.robot.systems.efi.FuelInjectorSS;
import frc.robot.systems.efi.injector.FuelInjectorConstants;
import frc.robot.systems.efi.injector.FuelInjectorIO;
import frc.robot.systems.efi.injector.FuelInjectorIOKrakenX60;
import frc.robot.systems.efi.injector.FuelInjectorIOSim;
import frc.robot.systems.efi.sensors.CANRangeSS;
import frc.robot.systems.efi.sensors.SensorIO;
import frc.robot.systems.intake.Intake;
import frc.robot.systems.intake.IntakeConstants;
import frc.robot.systems.intake.rack.IntakeRackIO;
import frc.robot.systems.intake.rack.IntakeRackIOKrakenX60;
import frc.robot.systems.intake.rack.IntakeRackIOSim;
import frc.robot.systems.intake.rack.IntakeRackSS;
import frc.robot.systems.intake.roller.IntakeRollerIO;
import frc.robot.systems.intake.roller.IntakeRollerIOKrakenX60;
import frc.robot.systems.intake.roller.IntakeRollerIOSim;
import frc.robot.systems.intake.roller.IntakeRollerSS;
import frc.robot.systems.shooter.combinedShooter.ShooterConstants;
import frc.robot.systems.shooter.combinedShooter.ShooterIO;
import frc.robot.systems.shooter.combinedShooter.ShooterIOKrakenX44;
import frc.robot.systems.shooter.combinedShooter.ShooterIOSim;
import frc.robot.systems.shooter.combinedShooter.ShooterSS;
import frc.robot.systems.shooter.encoder.EncoderIO;
import frc.robot.systems.shooter.hood.HoodSS;
import frc.robot.systems.shooter.shotMap.FeedMap;
import frc.robot.systems.shooter.shotMap.ShotMap;
import frc.robot.systems.switchableChannel.SwitchableChannelSS;
import frc.robot.systems.shooter.hood.HoodConstants;
import frc.robot.systems.shooter.hood.HoodIO;
import frc.robot.systems.shooter.hood.HoodIOKrakenX44;
import frc.robot.systems.shooter.hood.HoodIOSim;
import frc.robot.systems.apriltag.ATagCameraIO;
import frc.robot.systems.apriltag.ATagCameraIOPV;
import frc.robot.systems.apriltag.ATagVision;
import frc.robot.systems.apriltag.ATagVisionConstants;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import frc.robot.systems.climb.ClimbSS;
import frc.robot.systems.climb.ClimbIOKrakenx44;
import frc.robot.systems.climb.ClimbIOSim;
import frc.robot.systems.climb.ClimbIO;
import frc.robot.systems.climb.ClimbConstants;

public class RobotContainer {
    private final Drive mDriveSS;
    private final HoodSS mHoodSS;
    private final ShooterSS mShooterSS;
    private final Intake mIntakeSS;
    private final FuelInjectorSS mFuelInjectorSS;
    private final ClimbSS mClimbSS;
    private final CANRangeSS mCANRangesSS;
    private final SwitchableChannelSS mSwitchableChannelSS;

    private final LoggedDashboardChooser<Command> mDriverProfileChooser = new LoggedDashboardChooser<>("DriverProfile");
    private final ButtonBindings mButtonBindings;

    public RobotContainer() {
        switch (RobotConstants.kCurrentMode) {
            case REAL: {
                mDriveSS = new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFX(TunerConstants.FrontLeft),
                        new ModuleIOTalonFX(TunerConstants.FrontRight),
                        new ModuleIOTalonFX(TunerConstants.BackLeft),
                        new ModuleIOTalonFX(TunerConstants.BackRight));


                mCANRangesSS = new CANRangeSS(
                        new SensorIO() {
                        },
                        new SensorIO() {
                        },
                        new SensorIO() {
                        });

                mHoodSS = new HoodSS(new HoodIOKrakenX44(HoodConstants.kHoodConfig, HoodConstants.kHoodControlConfig),
                        mCANRangesSS);

                mShooterSS = new ShooterSS(
                        new ShooterIOKrakenX44(ShooterConstants.kFlywheelLeaderConfig),
                        new ShooterIOKrakenX44(ShooterConstants.kFlywheelFollowerConfig),
                        new ShooterIOKrakenX44(ShooterConstants.kFuelPumpFollower1Config),
                        new ShooterIOKrakenX44(ShooterConstants.kFuelPumpFollower2Config), 
                        mCANRangesSS,
                        new EncoderIO() {
                        });

                mIntakeSS = new Intake(
                        new IntakeRackSS(new IntakeRackIOKrakenX60(
                                IntakeConstants.RackConstants.kRackMotorConfig)),
                        new IntakeRollerSS(
                                new IntakeRollerIOKrakenX60(IntakeConstants.RollerConstants.kRollerMotorLeaderConfig),
                                new IntakeRollerIOKrakenX60(IntakeConstants.RollerConstants.kRollerFollowerConfig))
                        );

                mClimbSS = new ClimbSS(
                        new ClimbIOKrakenx44(ClimbConstants.kClimbMotorConstants));

                mFuelInjectorSS = new FuelInjectorSS(
                        new FuelInjectorIOKrakenX60(FuelInjectorConstants.kFuelInjectorConfig));

                mSwitchableChannelSS = new SwitchableChannelSS();
                break;
            }
            case SIM: {
                mDriveSS =
                        new Drive(
                                new GyroIO() {},
                                new ModuleIOSim(TunerConstants.FrontLeft),
                                new ModuleIOSim(TunerConstants.FrontRight),
                                new ModuleIOSim(TunerConstants.BackLeft),
                                new ModuleIOSim(TunerConstants.BackRight));

                ShooterIOSim leaderSim = new ShooterIOSim(ShooterConstants.kFlywheelLeaderConfig);
                ShooterIOSim follower1Sim = new ShooterIOSim(ShooterConstants.kFlywheelLeaderConfig);
                ShooterIOSim follower2Sim = new ShooterIOSim(ShooterConstants.kFuelPumpFollower1Config);
                ShooterIOSim follower3Sim = new ShooterIOSim(ShooterConstants.kFuelPumpFollower2Config);

                IntakeRollerIOSim intakeRollerLeaderSim = new IntakeRollerIOSim(IntakeConstants.RollerConstants.kRollerMotorLeaderConfig);
                IntakeRollerIOSim intakeRollerFollowerSim = new IntakeRollerIOSim(IntakeConstants.RollerConstants.kRollerMotorLeaderConfig);

                mCANRangesSS = new CANRangeSS(
                        new SensorIO() {
                        },
                        new SensorIO() {
                        },
                        new SensorIO() {
                        });

                mHoodSS = new HoodSS(new HoodIOSim(HoodConstants.kHoodConfig, HoodConstants.kHoodControlConfig),
                        mCANRangesSS);

                mShooterSS = new ShooterSS(
                        leaderSim,
                        follower1Sim,
                        follower2Sim,
                        follower3Sim,
                        mCANRangesSS,
                        new EncoderIO() {
                        });

                mIntakeSS = new Intake(
                        new IntakeRackSS(new IntakeRackIOSim(
                                IntakeConstants.RackConstants.kRackElevator,
                                IntakeConstants.RackConstants.kRackMotorConfig)),
                        new IntakeRollerSS(intakeRollerLeaderSim,intakeRollerFollowerSim));

                mFuelInjectorSS = new FuelInjectorSS(new FuelInjectorIOSim());

                mClimbSS = new ClimbSS(new ClimbIOSim(
                        ClimbConstants.kSimElevator,
                        ClimbConstants.kClimbMotorConstants,
                        ClimbConstants.kSoftLimits));

                mSwitchableChannelSS = new SwitchableChannelSS();

                break;
            }

            default: {
                mDriveSS = 
                        new Drive(
                                new GyroIO() {},
                                new ModuleIO() {},
                                new ModuleIO() {},
                                new ModuleIO() {},
                                new ModuleIO() {}
                        );

                mCANRangesSS = new CANRangeSS(
                        new SensorIO() {
                        },
                        new SensorIO() {
                        },
                        new SensorIO() {
                        });

                mHoodSS = new HoodSS(new HoodIO() {
                }, mCANRangesSS);

                mShooterSS = new ShooterSS(
                        new ShooterIO() {
                        },
                        new ShooterIO() {
                        },
                        new ShooterIO() {
                        },
                        new ShooterIO() {
                        },
                        mCANRangesSS,
                        new EncoderIO() {
                        });

                mIntakeSS = new Intake(
                        new IntakeRackSS(new IntakeRackIO() {
                        }),
                        new IntakeRollerSS(new IntakeRollerIO() {
                        },
                        new IntakeRollerIO() {
                        }));

                mClimbSS = new ClimbSS(new ClimbIO() {
                });

                mFuelInjectorSS = new FuelInjectorSS(new FuelInjectorIO() {
                });

                mSwitchableChannelSS = new SwitchableChannelSS();

                break;
            }
        }

        ShotMap.getInstance().setPoseSupplier(() -> mDriveSS.getPose());
        FeedMap.getInstance().setPoseSupplier(() -> mDriveSS.getPose());

        mButtonBindings = new ButtonBindings(mDriveSS, mHoodSS, mShooterSS, mIntakeSS, mFuelInjectorSS,
                mClimbSS, mCANRangesSS);

        initBindings();

        mDriverProfileChooser.addDefaultOption(
                BindingsConstants.kDefaultProfile.key(),
                mDriveSS.setDriveProfile(BindingsConstants.kDefaultProfile));
        for (DriverProfiles profile : BindingsConstants.kProfiles)
            mDriverProfileChooser.addOption(profile.key(), mDriveSS.setDriveProfile(profile));

        // autos = new AutonCommands(mDriveSS, mIntakeSS, mHoodSS, mShooterSS, mClimbSS, mFuelInjectorSS);
    }

    public Drive getDrivetrain() {
        return mDriveSS;
    }

    private void initBindings() {
        mButtonBindings.initBindings();
    }

//     public Supplier<Command> getAutonomousCommand() {
//         return autos.getAuto();
//     }

    public Command getDriverProfileCommand() {
        return mDriverProfileChooser.get();
    }
}
