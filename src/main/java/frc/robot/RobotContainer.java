// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swerve.SwerveModuleIO;
import frc.robot.commands.swerve.SwerveCMD;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  private final SwerveCMD defaultSubsystem = new SwerveCMD();

  private final CommandXboxController m_driverController =
      new CommandXboxController(0);

  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {

  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new PrintCommand("!Auto");
  }
}