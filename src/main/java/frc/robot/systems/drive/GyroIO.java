// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.systems.drive;

import edu.wpi.first.math.geometry.Rotation2d;

import static edu.wpi.first.units.Units.Rotation;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
  @AutoLog
  public static class GyroIOInputs {
    public boolean connected = false;
    public Rotation2d yawPosition = Rotation2d.kZero;
    public Rotation2d yawVelocityPerSecDeg = Rotation2d.kZero;
    public double[] odometryYawTimestamps = new double[] {};
    public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
    public double accelXG = 0.0;
    public double accelYG = 0.0;
  }

  public default void updateInputs(GyroIOInputs inputs) {}
  
  public default void resetGyro(Rotation2d rotation) {}

}