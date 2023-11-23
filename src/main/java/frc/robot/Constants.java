// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class robot {
    public static final boolean isSim = true;
  }
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class Swerve
  {
    public static final int maxDriveSpeedMPS = 5;
    public static final double maxRotationSpeedRadPS = 2 * Math.PI;
    public static final double controllerDeadBand = 0.1;
    public static final double maxDriveAccelerationMPSS = 5;
    public static final double maxRotationAccelerationRadPSS = 2 * Math.PI;
    public static final double horizontalBaseM = Units.inchesToMeters(17.5);
    public static final double wheelDiameterM = Units.inchesToMeters(3.5);
    public static final double verticalBaseM = Units.inchesToMeters(17.5);

    public static final SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics(
      new Translation2d(horizontalBaseM / 2, verticalBaseM /2),
      new Translation2d(horizontalBaseM / 2, -verticalBaseM /2),
      new Translation2d(-horizontalBaseM / 2, verticalBaseM /2),
      new Translation2d(-horizontalBaseM / 2, -verticalBaseM /2)
    );
  }
}
