package frc.robot.commands;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.Constants;
import frc.robot.Robot;

public class FollowTrajectory extends CommandBase {

  private Timer timer = new Timer();
  private Trajectory trajectory;
  private final PIDController driveXController = new PIDController(1, 0, 0);
  private final PIDController driveYController = new PIDController(1, 0, 0);
  private final PIDController turnController = new PIDController(1, 0, 0);

  private final Swerve swerve = Robot.swerve;

  public FollowTrajectory() {
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    // swerve.resetOdometry(new Pose2d(0,0,new Rotation2d(0)));
    trajectory = createTrajectory();
    timer.start();
  }

  @Override
  public void execute() {
    double cur_time = timer.get();
    State trajectoryState = trajectory.sample(cur_time);

    Pose2d targetPose = trajectoryState.poseMeters;

    Pose2d currentPose = swerve.getPose();

    double driveXPos = currentPose.getX();
    double driveYPos = currentPose.getY();

    double driveXVeloM = driveXController.calculate(driveXPos, targetPose.getX());
    double driveYVeloM = driveYController.calculate(driveYPos, targetPose.getY());

    double turnVelocity = turnController.calculate(currentPose.getRotation().getRadians(),
        targetPose.getRotation().getRadians());

    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(driveXVeloM, driveYVeloM, turnVelocity,
        swerve.getRotation2d());

    SwerveModuleState[] moduleStates = Constants.SwerveReal.driveKinematics
        .toSwerveModuleStates(chassisSpeeds);

    swerve.setModuleStates(moduleStates);
    swerve.setDrivetrainRotation(chassisSpeeds.omegaRadiansPerSecond * 0.02);

    SmartDashboard.putNumber("Drive X Pos", driveXPos);
    SmartDashboard.putNumber("Drive Y Pos", driveYPos);
    
    swerve.setModuleStates(moduleStates);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private Trajectory createTrajectory() {

    Pose2d startPose = new Pose2d(new Translation2d(0, 0), new Rotation2d(0));
    List<Translation2d> midpoints = new ArrayList<>();
    midpoints.add(new Translation2d(2, 2));
    midpoints.add(new Translation2d(2, 4));

    Pose2d endPose = new Pose2d(new Translation2d(0, 4), new Rotation2d(180));

    // generate trajectory
    return TrajectoryGenerator.generateTrajectory(
        startPose,
        midpoints,
        endPose,
        new TrajectoryConfig(
            0.5,
            0.5));
  }

  private Trajectory createTrajectory(Pose2d[] waypoints, boolean isReversed) {
    if (isReversed) {
      // iterate through waypoints from last to first
      Collections.reverse(Arrays.asList(waypoints));

      // reverse pose for each waypoint (subtract pi)
      for (int i = 0; i < waypoints.length; i++)
        waypoints[i] = waypoints[i].transformBy(
            new Transform2d(
                new Translation2d(0.0, 0.0),
                new Rotation2d(Math.PI)));
    }

    // generate trajectory
    return TrajectoryGenerator.generateTrajectory(
        List.of(waypoints),
        new TrajectoryConfig(
            Constants.SwerveReal.maxSpeedMPS,
            Constants.SwerveReal.maxDriveAccelMPS));
  }
}