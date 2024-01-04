package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.Constants;
import frc.robot.Robot;

public class MoveToPose extends CommandBase {
  private final Swerve swerve = Robot.swerve;
  private final Pose2d targetPose;

  private final PIDController driveXController = new PIDController(1, 0, 0);
  private final PIDController driveYController = new PIDController(1, 0, 0);
  private final PIDController turnController = new PIDController(1, 0, 0);

  public MoveToPose(Pose2d targetPose) {
    this.targetPose = targetPose;
    addRequirements(swerve);
    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    turnController.setTolerance(Math.PI / 500);
    driveXController.setTolerance(0.02);
    driveYController.setTolerance(0.02);
  }

  @Override
  public void execute() {
    Pose2d currentPose = swerve.getPose();

    double driveXPos = currentPose.getX();
    double driveYPos = currentPose.getY();

    double driveXVeloM = driveXController.calculate(driveXPos, targetPose.getX());
    double driveYVeloM = driveYController.calculate(driveYPos, targetPose.getY());

    double turnVelocity = turnController.calculate(currentPose.getRotation().getRadians(),
        targetPose.getRotation().getRadians());

    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(driveXVeloM, driveYVeloM, turnVelocity, swerve.getRotation2d());

    SwerveModuleState[] moduleStates = Constants.SwerveReal.driveKinematics
        .toSwerveModuleStates(chassisSpeeds);

    swerve.setModuleStates(moduleStates);
    swerve.setDrivetrainRotation(chassisSpeeds.omegaRadiansPerSecond * 0.02);

    SmartDashboard.putNumber("Drive X Pos", driveXPos);
    SmartDashboard.putNumber("Drive Y Pos", driveYPos);
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return atGoal();
  }

  /** Checks if the robot is stopped at the final pose. */
  public boolean atGoal() {
    return driveXController.atSetpoint() && driveYController.atSetpoint() && turnController.atSetpoint();
  }
}