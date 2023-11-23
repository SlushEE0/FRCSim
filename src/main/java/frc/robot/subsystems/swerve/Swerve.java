// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveModuleIO.ModuleData;

public class Swerve extends SubsystemBase {
  private final SwerveModuleIO[] modules = new SwerveModuleIO[4];
  private final ModuleData[] data = new ModuleData[4];

  private static double drivetrainRotationRad;

  public Swerve() {
    if (Constants.Robot.isSim) {
      for (int i = 0; i < 4; i++) {
        modules[i] = new SwerveModuleSim();
        data[i] = new ModuleData();
        data[i].index = i;
      }
    }
  }

  public Rotation2d getRotation2d() {
    return new Rotation2d();
  }

  public void setModuleState(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Swerve.maxSpeedMPS);
    for (int i = 0; i < 4; i++) {
      modules[i].setDesiredState(states[i]);
    }

    for (int i = 0; i < 4; i++) {
      modules[i].updateData(data[i]);
    }
    
    periodic();
  }

  public void setRotationRad(double rotation) {
    drivetrainRotationRad = rotation;
  }

  @Override
  public void periodic() {
    for (int i = 0; i < 4; i++) {
      modules[i].updateData(data[i]);
    }

    double[] realStates = {
        Math.toDegrees(data[0].turnPositionRad),
        data[0].driveVelocityMPerSec,
        Math.toDegrees(data[1].turnPositionRad),
        data[1].driveVelocityMPerSec,
        Math.toDegrees(data[2].turnPositionRad),
        data[2].driveVelocityMPerSec,
        Math.toDegrees(data[3].turnPositionRad),
        data[3].driveVelocityMPerSec
    };

    double[] theoryStates = {
        data[0].theoreticalState.angle.getDegrees(),
        data[0].theoreticalState.speedMetersPerSecond,
        data[1].theoreticalState.angle.getDegrees(),
        data[1].theoreticalState.speedMetersPerSecond,
        data[2].theoreticalState.angle.getDegrees(),
        data[2].theoreticalState.speedMetersPerSecond,
        data[3].theoreticalState.angle.getDegrees(),
        data[3].theoreticalState.speedMetersPerSecond
    };

    SmartDashboard.putNumberArray("Real States", realStates);
    SmartDashboard.putNumberArray("Theoretical States", theoryStates);
    SmartDashboard.putNumber("Drivetrain Rotation", Math.toDegrees(drivetrainRotationRad));
  }

  @Override
  public void simulationPeriodic() {
  }
}
