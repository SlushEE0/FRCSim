// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.Swerve;
import frc.robot.commands.swerve.SwerveCMD;
import frc.robot.Constants;

public class RobotContainer {
  private final Swerve defaultSubsystem = new Swerve();

  private final CommandXboxController pilot = new CommandXboxController(0);

  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    Robot.swerve.setDefaultCommand(
        new SwerveCMD(defaultSubsystem,
            new DoubleSupplier[] { () -> pilot.getLeftX(), () -> pilot.getLeftY() },
            new DoubleSupplier[] { () -> pilot.getRightX(), () -> pilot.getRightY() }));
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new PrintCommand("!Auto");
  }
}
