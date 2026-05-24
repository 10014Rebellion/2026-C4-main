package frc.lib.telemetry;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.LongSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.util.WPISerializable;
import edu.wpi.first.util.protobuf.Protobuf;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import frc.robot.RobotConstants;
import us.hebi.quickbuf.ProtoMessage;

public class TelemetryRecordOutput {
  private static boolean shouldLogDebugValue() {
    return RobotConstants.kTuningMode;
  }
  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void log(String key, byte[] value) {
    Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void log(String key, byte[][] value) {
    Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void log(String key, boolean value) {
    Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void log(String key, BooleanSupplier value) {
    Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void log(String key, boolean[] value) {
    Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void log(String key, boolean[][] value) {
    Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void log(String key, int value) {
    Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void log(String key, IntSupplier value) {
    Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void log(String key, int[] value) {
    Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void log(String key, int[][] value) {
    Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void log(String key, long value) {
    Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void log(String key, LongSupplier value) {
    Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void log(String key, long[] value) {
    Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void log(String key, long[][] value) {
    Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void log(String key, float value) {
    Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void log(String key, float[] value) {
    Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void log(String key, float[][] value) {
    Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void log(String key, double value) {
    Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void log(String key, DoubleSupplier value) {
    Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void log(String key, double[] value) {
    Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void log(String key, double[][] value) {
    Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void log(String key, String value) {
    Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void log(String key, String[] value) {
    Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void log(String key, String[][] value) {
    Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static <E extends Enum<E>> void log(String key, E value) {
    Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static <E extends Enum<E>> void log(String key, E[] value) {
    Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static <E extends Enum<E>> void log(String key, E[][] value) {
    Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static <U extends Unit> void log(String key, Measure<U> value) {
    Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * <p>
   * This method serializes a single object as a struct. Example usage:
   * {@code recordOutput("MyPose", Pose2d.struct, new Pose2d())}
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static <T> void log(String key, Struct<T> struct, T value) {
    Logger.recordOutput(key, struct, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method serializes an array of objects as a struct. Example usage:
   * {@code
   * recordOutput("MyPoses", Pose2d.struct, new Pose2d(), new Pose2d());
   * recordOutput("MyPoses", Pose2d.struct, new Pose2d[] {new Pose2d(), new
   * Pose2d()});
   * }
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  @SuppressWarnings("unchecked")
  public static <T> void log(String key, Struct<T> struct, T... value) {
    Logger.recordOutput(key, struct, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static <T> void log(String key, Struct<T> struct, T[][] value) {
    Logger.recordOutput(key, struct, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method serializes a single object as a protobuf. Protobuf should only be
   * used for objects that do not support struct serialization. Example usage:
   * {@code recordOutput("MyPose", Pose2d.proto, new Pose2d())}
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static <T, MessageType extends ProtoMessage<?>> void log(String key, Protobuf<T, MessageType> proto,
      T value) {
    Logger.recordOutput(key, proto, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method serializes a single object as a struct or protobuf automatically.
   * Struct is preferred if both methods are supported.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param <T>   The type
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static <T extends WPISerializable> void log(String key, T value) {
    Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method serializes an array of objects as a struct automatically.
   * Top-level protobuf arrays are not supported.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param <T>   The type
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  @SuppressWarnings("unchecked")
  public static <T extends StructSerializable> void log(String key, T... value) {
    Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method serializes an array of objects as a struct automatically.
   * Top-level protobuf arrays are not supported.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param <T>   The type
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static <T extends StructSerializable> void log(String key, T[][] value) {
    Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method serializes a single object as a struct or protobuf automatically.
   * Struct is preferred if both methods are supported.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param <R>   The type
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static <R extends Record> void log(String key, R value) {
    Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method serializes an array of objects as a struct automatically.
   * Top-level protobuf arrays are not supported.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param <R>   The type
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  @SuppressWarnings("unchecked")
  public static <R extends Record> void log(String key, R... value) {
    Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method serializes an array of objects as a struct automatically.
   * Top-level protobuf arrays are not supported.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param <R>   The type
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static <R extends Record> void log(String key, R[][] value) {
    Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * The current position of the Mechanism2d is logged once as a set of nested
   * fields. If the position is updated, this method must be called again.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void log(String key, LoggedMechanism2d value) {
    Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void logDebug(String key, byte[] value) {
    if(shouldLogDebugValue()) Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void logDebug(String key, byte[][] value) {
    if(shouldLogDebugValue()) Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void logDebug(String key, boolean value) {
    if(shouldLogDebugValue()) Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void logDebug(String key, BooleanSupplier value) {
    if(shouldLogDebugValue()) Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void logDebug(String key, boolean[] value) {
    if(shouldLogDebugValue()) Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void logDebug(String key, boolean[][] value) {
    if(shouldLogDebugValue()) Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void logDebug(String key, int value) {
    if(shouldLogDebugValue()) Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void logDebug(String key, IntSupplier value) {
    if(shouldLogDebugValue()) Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void logDebug(String key, int[] value) {
    if(shouldLogDebugValue()) Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void logDebug(String key, int[][] value) {
    if(shouldLogDebugValue()) Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void logDebug(String key, long value) {
    if(shouldLogDebugValue()) Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void logDebug(String key, LongSupplier value) {
    if(shouldLogDebugValue()) Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void logDebug(String key, long[] value) {
    if(shouldLogDebugValue()) Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void logDebug(String key, long[][] value) {
    if(shouldLogDebugValue()) Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void logDebug(String key, float value) {
    if(shouldLogDebugValue()) Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void logDebug(String key, float[] value) {
    if(shouldLogDebugValue()) Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void logDebug(String key, float[][] value) {
    if(shouldLogDebugValue()) Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void logDebug(String key, double value) {
    if(shouldLogDebugValue()) Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void logDebug(String key, DoubleSupplier value) {
    if(shouldLogDebugValue()) Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void logDebug(String key, double[] value) {
    if(shouldLogDebugValue()) Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void logDebug(String key, double[][] value) {
    if(shouldLogDebugValue()) Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void logDebug(String key, String value) {
    if(shouldLogDebugValue()) Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void logDebug(String key, String[] value) {
    if(shouldLogDebugValue()) Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void logDebug(String key, String[][] value) {
    if(shouldLogDebugValue()) Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static <E extends Enum<E>> void logDebug(String key, E value) {
    if(shouldLogDebugValue()) Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static <E extends Enum<E>> void logDebug(String key, E[] value) {
    if(shouldLogDebugValue()) Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static <E extends Enum<E>> void logDebug(String key, E[][] value) {
    if(shouldLogDebugValue()) Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static <U extends Unit> void logDebug(String key, Measure<U> value) {
    if(shouldLogDebugValue()) Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * <p>
   * This method serializes a single object as a struct. Example usage:
   * {@code recordOutput("MyPose", Pose2d.struct, new Pose2d())}
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static <T> void logDebug(String key, Struct<T> struct, T value) {
    if(shouldLogDebugValue()) Logger.recordOutput(key, struct, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method serializes an array of objects as a struct. Example usage:
   * {@code
   * recordOutput("MyPoses", Pose2d.struct, new Pose2d(), new Pose2d());
   * recordOutput("MyPoses", Pose2d.struct, new Pose2d[] {new Pose2d(), new
   * Pose2d()});
   * }
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  @SuppressWarnings("unchecked")
  public static <T> void logDebug(String key, Struct<T> struct, T... value) {
    if(shouldLogDebugValue()) Logger.recordOutput(key, struct, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static <T> void logDebug(String key, Struct<T> struct, T[][] value) {
    if(shouldLogDebugValue()) Logger.recordOutput(key, struct, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method serializes a single object as a protobuf. Protobuf should only be
   * used for objects that do not support struct serialization. Example usage:
   * {@code recordOutput("MyPose", Pose2d.proto, new Pose2d())}
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static <T, MessageType extends ProtoMessage<?>> void logDebug(String key, Protobuf<T, MessageType> proto,
      T value) {
    if(shouldLogDebugValue()) Logger.recordOutput(key, proto, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method serializes a single object as a struct or protobuf automatically.
   * Struct is preferred if both methods are supported.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param <T>   The type
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static <T extends WPISerializable> void logDebug(String key, T value) {
    if(shouldLogDebugValue()) Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method serializes an array of objects as a struct automatically.
   * Top-level protobuf arrays are not supported.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param <T>   The type
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  @SuppressWarnings("unchecked")
  public static <T extends StructSerializable> void logDebug(String key, T... value) {
    if(shouldLogDebugValue()) Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method serializes an array of objects as a struct automatically.
   * Top-level protobuf arrays are not supported.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param <T>   The type
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static <T extends StructSerializable> void logDebug(String key, T[][] value) {
    if(shouldLogDebugValue()) Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method serializes a single object as a struct or protobuf automatically.
   * Struct is preferred if both methods are supported.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param <R>   The type
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static <R extends Record> void logDebug(String key, R value) {
    if(shouldLogDebugValue()) Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method serializes an array of objects as a struct automatically.
   * Top-level protobuf arrays are not supported.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param <R>   The type
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  @SuppressWarnings("unchecked")
  public static <R extends Record> void logDebug(String key, R... value) {
    if(shouldLogDebugValue()) Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * This method serializes an array of objects as a struct automatically.
   * Top-level protobuf arrays are not supported.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param <R>   The type
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static <R extends Record> void logDebug(String key, R[][] value) {
    if(shouldLogDebugValue()) Logger.recordOutput(key, value);
  }

  /**
   * Records a single output field for easy access when viewing the log. On the
   * simulator, use this method to record extra data based on the original inputs.
   * 
   * <p>
   * The current position of the Mechanism2d is logged once as a set of nested
   * fields. If the position is updated, this method must be called again.
   * 
   * <p>
   * This method is <b>not thread-safe</b> and should only be called from the
   * main thread. See the "Common Issues" page in the documentation for more
   * details.
   * 
   * @param key   The name of the field to record. It will be stored under
   *              "/RealOutputs" or "/ReplayOutputs"
   * @param value The value of the field.
   */
  public static void logDebug(String key, LoggedMechanism2d value) {
    if(shouldLogDebugValue()) Logger.recordOutput(key, value);
  }
}
