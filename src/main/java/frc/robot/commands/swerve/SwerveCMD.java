// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.Swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class SwerveCMD extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  private Swerve subsystem;
  private DoubleSupplier leftXSupplier;
  private DoubleSupplier leftYSupplier;
  private DoubleSupplier rightXSupplier;

  private final SlewRateLimiter xDriveLimiter = new SlewRateLimiter(Constants.Swerve.maxDriveAccelMPS);
  private final SlewRateLimiter yDriveLimiter = new SlewRateLimiter(Constants.Swerve.maxDriveAccelMPS);
  private final SlewRateLimiter turnLimiter = new SlewRateLimiter(Constants.Swerve.maxTurnAccelRadPs);

  public SwerveCMD(Swerve subsystem, DoubleSupplier[] leftJoystick, DoubleSupplier[] rightJoystick) {
    this.subsystem = subsystem;
    this.leftXSupplier = leftJoystick[0];
    this.leftYSupplier = leftJoystick[1];

    this.rightXSupplier = rightJoystick[0];

    addRequirements(Robot.swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = leftXSupplier.getAsDouble();
    double ySpeed = (leftYSupplier.getAsDouble());
    double turnSpeed = rightXSupplier.getAsDouble();

    xSpeed = Math.abs(xSpeed) > Constants.Swerve.controllerDeadband ? xSpeed : 0;
    ySpeed = Math.abs(ySpeed) > Constants.Swerve.controllerDeadband ? ySpeed : 0;
    turnSpeed = Math.abs(turnSpeed) > Constants.Swerve.controllerDeadband ? turnSpeed : 0;

    xSpeed = xDriveLimiter.calculate(xSpeed * Constants.Swerve.maxSpeedMPS);
    ySpeed = yDriveLimiter.calculate(ySpeed * Constants.Swerve.maxSpeedMPS);
    turnSpeed = turnLimiter.calculate(turnSpeed * Constants.Swerve.maxTurnSpeedRadPS);

    SmartDashboard.putNumber("turnSpeed", turnSpeed);

    // ChassisSpeeds f = new ChassisSpeeds();
    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turnSpeed,
        Robot.swerve.getRotation2d());
    
    SmartDashboard.putNumber("CS vX", chassisSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("CS vY", chassisSpeeds.vyMetersPerSecond);

    SwerveModuleState[] states = Constants.Swerve.driveKinematics.toSwerveModuleStates(chassisSpeeds);
    
    Robot.swerve.setModuleState(states);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
