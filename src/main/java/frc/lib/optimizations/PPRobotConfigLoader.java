package frc.lib.optimizations;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardCopyOption;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Filesystem;
import frc.lib.telemetry.Telemetry;
import frc.robot.RobotConstants;
import frc.robot.logging.DriveErrors.PPCacheSaveFailed;
import frc.robot.logging.DriveErrors.PPConfigCacheLoadFailed;
import frc.robot.logging.DriveErrors.PPConfigDefaultLoadFailed;
import frc.robot.logging.DriveErrors.PPConfigLoadedFromCache;
import frc.robot.logging.DriveErrors.PPConfigLoadedFromDefault;
import frc.robot.logging.DriveErrors.PPGuiLoadFailed;

import org.json.simple.parser.ParseException;

public class PPRobotConfigLoader {
  private static final Path kGUIConfig = Filesystem.getDeployDirectory().toPath().resolve("pathplanner/settings.json");

  private static final Path kDeployCacheConfig = Filesystem.getDeployDirectory().toPath().resolve("cache/robotconfig-cache.json");

  private static final Path kRioCacheConfig = Paths.get("/home/lvuser/robotconfig-cache.json");

  public static RobotConfig load() {
    // First try loading from GUI.
    try {
      RobotConfig config = RobotConfig.fromGUISettings();
      validate(config);

      try {
        cacheGuiToDeployConfig();
        if(!RobotConstants.isSim()) {
          try {
            cacheGuiToRioConfig();
          } catch (Exception pFailedtoSavetoCache) {
            Telemetry.reportIssue(new PPCacheSaveFailed());
            Telemetry.reportException(pFailedtoSavetoCache);
          }
        } else {
          Telemetry.log("Currently in sim, can't save GUI to rio cache unless deployed on a real robot");
        }
      } catch(Exception pFailedtoSavetoCache) {
        Telemetry.reportIssue(new PPCacheSaveFailed());
        Telemetry.reportException(pFailedtoSavetoCache);
      }

      // Could do reread of the saved data to ensure that its correct
      // RobotConfig reread = loadFromDisk(kRioCacheConfig);
      // validate(reread);

      Telemetry.log("Drive Config Successfully Loaded from GUI");

      return config;
    } catch (Exception pGUIFail) {
      Telemetry.reportIssue(new PPGuiLoadFailed());
      Telemetry.reportException(pGUIFail);
    }

    if(!RobotConstants.isSim()) {
      // If GUI loading failed, load the cache
      try {
        RobotConfig config = fromSettingsFile(kRioCacheConfig);
        validate(config);
  
        Telemetry.reportIssue(new PPConfigLoadedFromCache());
  
        return config;
      } catch (Exception pCacheFail) {
        Telemetry.reportIssue(new PPConfigCacheLoadFailed());
        Telemetry.reportException(pCacheFail);
      }
    } else {
      Telemetry.log("Currently in sim, skipping caching from rio and instead attempting to use deploy folder cache");
    }

    // If cache failed, try defaulting to deploy config
    try {
      RobotConfig config = fromSettingsFile(kDeployCacheConfig);
      validate(config);

      Telemetry.reportIssue(new PPConfigLoadedFromDefault());

      return config;
    } catch (Exception pDefaultFail) {
      Telemetry.reportIssue(new PPConfigDefaultLoadFailed());
      Telemetry.reportException(pDefaultFail);
    }

    // If it all failed, you're cooked.
    throw new IllegalStateException(
        "<<< NO VALID DRIVE CONFIG AVAILABLE. ROBOT MUST NOT ENABLE");
  }

  private static void cacheGuiToDeployConfig() throws IOException {
    Files.copy(
        kGUIConfig,
        kDeployCacheConfig,
        StandardCopyOption.REPLACE_EXISTING,
        StandardCopyOption.COPY_ATTRIBUTES);
  }

  private static void cacheGuiToRioConfig() throws IOException {
    Files.copy(
        kGUIConfig,
        kDeployCacheConfig,
        StandardCopyOption.REPLACE_EXISTING,
        StandardCopyOption.COPY_ATTRIBUTES);
  }

  // Stole this from pathplanner
  private static RobotConfig fromSettingsFile(Path settingsPath) throws IOException, ParseException {
    BufferedReader br = new BufferedReader(new FileReader(settingsPath.toFile()));
    StringBuilder fileContentBuilder = new StringBuilder();

    String line;
    while ((line = br.readLine()) != null)
      fileContentBuilder.append(line);
    br.close();

    JSONObject json = (JSONObject) new JSONParser()
        .parse(fileContentBuilder.toString());

    boolean isHolonomic = (Boolean) json.get("holonomicMode");
    double massKG = ((Number) json.get("robotMass")).doubleValue();
    double MOI = ((Number) json.get("robotMOI")).doubleValue();
    double wheelRadius = ((Number) json.get("driveWheelRadius")).doubleValue();
    double gearing = ((Number) json.get("driveGearing")).doubleValue();
    double maxDriveSpeed = ((Number) json.get("maxDriveSpeed")).doubleValue();
    double wheelCOF = ((Number) json.get("wheelCOF")).doubleValue();
    String driveMotor = (String) json.get("driveMotorType");
    double driveCurrentLimit = ((Number) json.get("driveCurrentLimit")).doubleValue();

    int numMotors = isHolonomic ? 1 : 2;

    DCMotor gearbox = switch (driveMotor) {
      case "krakenX60" -> DCMotor.getKrakenX60(numMotors);
      case "krakenX60FOC" -> DCMotor.getKrakenX60Foc(numMotors);
      case "falcon500" -> DCMotor.getFalcon500(numMotors);
      case "falcon500FOC" -> DCMotor.getFalcon500Foc(numMotors);
      case "vortex" -> DCMotor.getNeoVortex(numMotors);
      case "NEO" -> DCMotor.getNEO(numMotors);
      case "CIM" -> DCMotor.getCIM(numMotors);
      case "miniCIM" -> DCMotor.getMiniCIM(numMotors);
      default -> throw new IllegalArgumentException("Invalid motor type: " + driveMotor);
    };

    gearbox = gearbox.withReduction(gearing);

    ModuleConfig moduleConfig = new ModuleConfig(wheelRadius, maxDriveSpeed, wheelCOF, gearbox,
        driveCurrentLimit, numMotors);

    if (isHolonomic) {
      Translation2d[] moduleOffsets = {
          new Translation2d(
              ((Number) json.get("flModuleX")).doubleValue(),
              ((Number) json.get("flModuleY")).doubleValue()),
          new Translation2d(
              ((Number) json.get("frModuleX")).doubleValue(),
              ((Number) json.get("frModuleY")).doubleValue()),
          new Translation2d(
              ((Number) json.get("blModuleX")).doubleValue(),
              ((Number) json.get("blModuleY")).doubleValue()),
          new Translation2d(
              ((Number) json.get("brModuleX")).doubleValue(),
              ((Number) json.get("brModuleY")).doubleValue())
      };

      return new RobotConfig(massKG, MOI, moduleConfig, moduleOffsets);
    } else {
      double trackwidth = ((Number) json.get("robotTrackwidth")).doubleValue();
      return new RobotConfig(massKG, MOI, moduleConfig, trackwidth);
    }
  }

  private static void validate(RobotConfig pConfig) {
    if (pConfig == null)
      throw new IllegalStateException("<<< ROBOT CONFIG IS NULL >>>");
    if (pConfig.moduleConfig == null)
      throw new IllegalStateException("<<< MODULE CONFIG IS NULL >>>");
    if (pConfig.moduleLocations == null || pConfig.moduleLocations.length != pConfig.numModules)
      throw new IllegalStateException("<<< INVALID MODULE LOCATIONS >>>");
    if (pConfig.moduleConfig.maxDriveVelocityMPS <= 0)
      throw new IllegalStateException("<<< INVALID MAX DRIVE VELOCITY \"" + pConfig.moduleConfig.maxDriveVelocityMPS + "MPS\" >>>");
    if (pConfig.moduleConfig.maxDriveVelocityRadPerSec <= 0)
      throw new IllegalStateException("<<< INVALID MAX DRIVE ROTATION VELOCITY \"" + pConfig.moduleConfig.maxDriveVelocityRadPerSec + "rad/s\" >>>");
    if (pConfig.numModules != 4)
      throw new IllegalStateException("<<< INVALID SWERVE MODULE COUNT \"" + pConfig.numModules + "\" >>>");
  }
}
