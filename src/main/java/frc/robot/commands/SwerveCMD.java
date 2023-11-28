package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.Swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class SwerveCMD extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  private DoubleSupplier leftXSupplier;
  private DoubleSupplier leftYSupplier;
  private DoubleSupplier rightXSupplier;

  private final SlewRateLimiter xDriveLimiter = new SlewRateLimiter(Constants.SwerveSim.maxDriveAccelMPS);
  private final SlewRateLimiter yDriveLimiter = new SlewRateLimiter(Constants.SwerveSim.maxDriveAccelMPS);
  private final SlewRateLimiter turnLimiter = new SlewRateLimiter(Constants.SwerveSim.maxRotationAccelRadPS);

  public SwerveCMD(DoubleSupplier[] leftJoystick, DoubleSupplier[] rightJoystick) {
    this.leftXSupplier = leftJoystick[1];
    this.leftYSupplier = leftJoystick[0];

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

    xSpeed = Math.abs(xSpeed) > Constants.SwerveSim.controllerDeadband ? xSpeed : 0;
    ySpeed = Math.abs(ySpeed) > Constants.SwerveSim.controllerDeadband ? ySpeed : 0;
    turnSpeed = Math.abs(turnSpeed) > Constants.SwerveSim.controllerDeadband ? turnSpeed : 0;

    xSpeed = xDriveLimiter.calculate(xSpeed * Constants.SwerveSim.maxSpeedMPS);
    ySpeed = yDriveLimiter.calculate(ySpeed * Constants.SwerveSim.maxSpeedMPS);
    turnSpeed = turnLimiter.calculate(turnSpeed * Constants.SwerveSim.maxRotationSpeedRadPS);

    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turnSpeed,
        Robot.swerve.getRotation2d());

    SwerveModuleState[] states = Constants.SwerveSim.driveKinematics.toSwerveModuleStates(chassisSpeeds);

    Robot.swerve.setModuleStates(states);
    Robot.swerve.setDrivetrainRotation(chassisSpeeds.omegaRadiansPerSecond);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}