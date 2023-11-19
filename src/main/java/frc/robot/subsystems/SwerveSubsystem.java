// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveModuleIO;
import frc.robot.subsystems.swerve.SwerveModuleSim;
import frc.robot.subsystems.swerve.SwerveModuleIO.ModuleData;

public class SwerveSubsystem extends SubsystemBase {
  private final SwerveModuleIO[] modules = new SwerveModuleIO[4];
  private final ModuleData[] data = new ModuleData[4];

  public SwerveSubsystem() {
    if (Constants.Robot.isSim) {
      for (int i = 0; i < data.length; i++) {
        data[i] = new ModuleData();
        modules[i] = new SwerveModuleSim();
      }
    }
  }

  public Rotation2d getRotation2d() {
    return new Rotation2d();
  }

  public void setModuleState(SwerveModuleState[] states) {
    for (int i = 0; i < states.length; i++) {
      modules[i].setDesiredState(states[i]);
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i < 4; i++) {
      modules[i].updateData(data[i]);
    }
  }

  @Override
  public void simulationPeriodic() {
  }
}
