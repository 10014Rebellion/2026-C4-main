package frc.robot;

import static frc.robot.systems.drive.DriveConstants.*;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.bindings.BindingsConstants;
import frc.robot.bindings.ButtonBindings;
import frc.robot.systems.drive.Drive;
import frc.robot.systems.drive.controllers.ManualTeleopController.DriverProfiles;
import frc.robot.systems.drive.gyro.GyroIO;
import frc.robot.systems.drive.gyro.GyroIOPigeon2;
import frc.robot.systems.drive.modules.Module;
import frc.robot.systems.drive.modules.ModuleIO;
import frc.robot.systems.drive.modules.ModuleIOKraken;
import frc.robot.systems.drive.modules.ModuleIOSim;
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
import frc.robot.systems.intake.roller.IntakeRollerIOKrakenX44;
import frc.robot.systems.intake.roller.IntakeRollerIOSim;
import frc.robot.systems.intake.roller.IntakeRollerSS;
import frc.robot.systems.shooter.ShotMap;
import frc.robot.systems.shooter.flywheels.FlywheelConstants;
import frc.robot.systems.shooter.flywheels.FlywheelIO;
import frc.robot.systems.shooter.flywheels.FlywheelIOKrakenX44;
import frc.robot.systems.shooter.flywheels.FlywheelIOSim;
import frc.robot.systems.shooter.flywheels.FlywheelsSS;
import frc.robot.systems.shooter.flywheels.encoder.EncoderIO;
import frc.robot.systems.shooter.fuelpump.FuelPumpConstants;
import frc.robot.systems.shooter.fuelpump.FuelPumpIO;
import frc.robot.systems.shooter.fuelpump.FuelPumpIOKrakenX44;
import frc.robot.systems.shooter.fuelpump.FuelPumpIOSim;
import frc.robot.systems.shooter.fuelpump.FuelPumpSS;
import frc.robot.systems.shooter.hood.HoodSS;
import frc.robot.systems.shooter.hood.HoodConstants;
import frc.robot.systems.shooter.hood.HoodIO;
import frc.robot.systems.shooter.hood.HoodIOKrakenX44;
import frc.robot.systems.shooter.hood.HoodIOSim;
import frc.robot.systems.apriltag.ATagCameraIO;
import frc.robot.systems.apriltag.ATagCameraIOPV;
import frc.robot.systems.apriltag.ATagVision;
import frc.robot.systems.apriltag.ATagVisionConstants;
import frc.robot.systems.auton.AutonCommands;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import frc.robot.systems.climb.ClimbSS;
import frc.robot.systems.climb.ClimbIOKrakenx44;
import frc.robot.systems.climb.ClimbIOSim;
import frc.robot.systems.climb.ClimbIO;
import frc.robot.systems.climb.ClimbConstants;

public class RobotContainer {
    private final Drive mDriveSS;
    private final FuelPumpSS mFuelPumpSS;
    private final HoodSS mHoodSS;
    private final FlywheelsSS mFlywheelsSS;
    private final Intake mIntakeSS;
    private final FuelInjectorSS mFuelInjectorSS;
    private final ClimbSS mClimbSS;
    private final CANRangeSS mCANRangesSS;

    private final LoggedDashboardChooser<Command> mDriverProfileChooser = new LoggedDashboardChooser<>("DriverProfile");
    private final ButtonBindings mButtonBindings;
    private final AutonCommands autos;

    public RobotContainer() {

        switch (RobotConstants.kCurrentMode) {
            case REAL: {
                mDriveSS = new Drive(
                        new Module[] {
                                new Module("FL", new ModuleIOKraken(kFrontLeftHardware)),
                                new Module("FR", new ModuleIOKraken(kFrontRightHardware)),
                                new Module("BL", new ModuleIOKraken(kBackLeftHardware)),
                                new Module("BR", new ModuleIOKraken(kBackRightHardware))
                        },
                        new GyroIOPigeon2(),
                        new ATagVision(new ATagCameraIOPV[] {
                                new ATagCameraIOPV(ATagVisionConstants.kFLATagCamHardware),
                                new ATagCameraIOPV(ATagVisionConstants.kFRATagCamHardware),
                                new ATagCameraIOPV(ATagVisionConstants.kBLATagCamHardware),
                                new ATagCameraIOPV(ATagVisionConstants.kBRATagCamHardware)
                        }));

                mFuelPumpSS = new FuelPumpSS(
                        new FuelPumpIOKrakenX44(FuelPumpConstants.kFuelPumpLeaderConfig),
                        new FuelPumpIOKrakenX44(FuelPumpConstants.kFuelPumpFollowerConfig));

                mCANRangesSS = new CANRangeSS(
                        new SensorIO() {
                        },
                        new SensorIO() {
                        },
                        new SensorIO() {
                        });

                mHoodSS = new HoodSS(new HoodIOKrakenX44(HoodConstants.kHoodConfig, HoodConstants.kHoodControlConfig),
                        mCANRangesSS);

                mFlywheelsSS = new FlywheelsSS(
                        new FlywheelIOKrakenX44(FlywheelConstants.kFlywheelLeaderConfig),
                        new FlywheelIOKrakenX44(FlywheelConstants.kFlywheelFollowerConfig),
                        mCANRangesSS,
                        new EncoderIO() {
                        });

                mIntakeSS = new Intake(
                        new IntakeRackSS(new IntakeRackIOKrakenX60(
                                IntakeConstants.RackConstants.kRackMotorConfig)),
                        new IntakeRollerSS(
                                new IntakeRollerIOKrakenX44(IntakeConstants.RollerConstants.kRollerMotorConfig)));

                mClimbSS = new ClimbSS(
                        new ClimbIOKrakenx44(ClimbConstants.kClimbMotorConstants));

                mFuelInjectorSS = new FuelInjectorSS(
                        new FuelInjectorIOKrakenX60(FuelInjectorConstants.kFuelInjectorConfig));
                break;
            }
            case SIM: {
                mDriveSS = new Drive(
                        new Module[] {
                                new Module("FL", new ModuleIOSim()),
                                new Module("FR", new ModuleIOSim()),
                                new Module("BL", new ModuleIOSim()),
                                new Module("BR", new ModuleIOSim())
                        },
                        new GyroIO() {
                        },
                        new ATagVision(new ATagCameraIO[] {
                                new ATagCameraIOPV(ATagVisionConstants.kFLATagCamHardware),
                                new ATagCameraIOPV(ATagVisionConstants.kFRATagCamHardware),
                                new ATagCameraIOPV(ATagVisionConstants.kBLATagCamHardware),
                                new ATagCameraIOPV(ATagVisionConstants.kBRATagCamHardware)
                        }));

                FlywheelIOSim leaderSim = new FlywheelIOSim(FlywheelConstants.kFlywheelLeaderConfig);
                FlywheelIOSim followerSim = new FlywheelIOSim(FlywheelConstants.kFlywheelLeaderConfig);
                ;

                mFuelPumpSS = new FuelPumpSS(
                        new FuelPumpIOSim(FuelPumpConstants.kFuelPumpLeaderConfig),
                        new FuelPumpIOSim(FuelPumpConstants.kFuelPumpFollowerConfig));

                mCANRangesSS = new CANRangeSS(
                        new SensorIO() {
                        },
                        new SensorIO() {
                        },
                        new SensorIO() {
                        });

                mHoodSS = new HoodSS(new HoodIOSim(HoodConstants.kHoodConfig, HoodConstants.kHoodControlConfig),
                        mCANRangesSS);

                mFlywheelsSS = new FlywheelsSS(
                        leaderSim,
                        followerSim,
                        mCANRangesSS,
                        new EncoderIO() {
                        });

                mIntakeSS = new Intake(
                        new IntakeRackSS(new IntakeRackIOSim(
                                IntakeConstants.RackConstants.kRackElevator,
                                IntakeConstants.RackConstants.kRackMotorConfig)),
                        new IntakeRollerSS(new IntakeRollerIOSim()));

                mFuelInjectorSS = new FuelInjectorSS(new FuelInjectorIOSim());

                mClimbSS = new ClimbSS(new ClimbIOSim(
                        ClimbConstants.kSimElevator,
                        ClimbConstants.kClimbMotorConstants,
                        ClimbConstants.kSoftLimits));
                break;
            }

            default: {
                mDriveSS = new Drive(
                        new Module[] {
                                new Module("FL", new ModuleIO() {
                                }),
                                new Module("FR", new ModuleIO() {
                                }),
                                new Module("BL", new ModuleIO() {
                                }),
                                new Module("BR", new ModuleIO() {
                                })
                        },
                        new GyroIO() {
                        },
                        new ATagVision(new ATagCameraIO[] {
                                new ATagCameraIO() {
                                },
                                new ATagCameraIO() {
                                },
                                new ATagCameraIO() {
                                },
                                new ATagCameraIO() {
                                }
                        }));

                mFuelPumpSS = new FuelPumpSS(
                        new FuelPumpIO() {
                        },
                        new FuelPumpIO() {
                        });

                mCANRangesSS = new CANRangeSS(
                        new SensorIO() {
                        },
                        new SensorIO() {
                        },
                        new SensorIO() {
                        });

                mHoodSS = new HoodSS(new HoodIO() {
                }, mCANRangesSS);

                mFlywheelsSS = new FlywheelsSS(
                        new FlywheelIO() {
                        },
                        new FlywheelIO() {
                        },
                        mCANRangesSS,
                        new EncoderIO() {
                        });

                mIntakeSS = new Intake(
                        new IntakeRackSS(new IntakeRackIO() {
                        }),
                        new IntakeRollerSS(new IntakeRollerIO() {
                        }));

                mClimbSS = new ClimbSS(new ClimbIO() {
                });

                mFuelInjectorSS = new FuelInjectorSS(new FuelInjectorIO() {
                });

                break;
            }
        }

        ShotMap.getInstance().setPoseSupplier(() -> mDriveSS.getPoseEstimate());

        mButtonBindings = new ButtonBindings(mDriveSS, mFuelPumpSS, mHoodSS, mFlywheelsSS, mIntakeSS, mFuelInjectorSS,
                mClimbSS, mCANRangesSS);

        initBindings();

        mDriverProfileChooser.addDefaultOption(
                BindingsConstants.kDefaultProfile.key(),
                mDriveSS.getDriveManager().setDriveProfile(BindingsConstants.kDefaultProfile));
        for (DriverProfiles profile : BindingsConstants.kProfiles)
            mDriverProfileChooser.addOption(profile.key(), mDriveSS.getDriveManager().setDriveProfile(profile));

        autos = new AutonCommands(mDriveSS, mIntakeSS, mFuelPumpSS, mHoodSS, mFlywheelsSS, mClimbSS, mFuelInjectorSS);
    }

    public Drive getDrivetrain() {
        return mDriveSS;
    }

    private void initBindings() {
        mButtonBindings.initBindings();
    }

    public Supplier<Command> getAutonomousCommand() {
        return autos.getAuto();
    }

    public Command getDriverProfileCommand() {
        return mDriverProfileChooser.get();
    }
}
