package frc.robot.subsystems.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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

  public double drivetrainRotationRad = 0;
  SwerveDrivePoseEstimator swervePose;

  public Swerve() {
    for (int i = 0; i < 4; i++) {
      if (Constants.isSim) {
        modules[i] = new SwerveModuleSim();
      } else {
        modules[i] = new SwerveModuleSpark(i);
      }
      data[i] = new ModuleData();
    }

    SwerveModulePosition[] poseArr = { data[0].position, data[1].position, data[2].position, data[3].position };

    swervePose = new SwerveDrivePoseEstimator(Constants.SwerveReal.driveKinematics,
        getRotation2d(),
        poseArr,
        new Pose2d(new Translation2d(0, 0), new Rotation2d(0, 0)));
  }

  public Rotation2d getRotation2d() {
    return new Rotation2d(drivetrainRotationRad);
  }

  public Pose2d getPose() {
    Pose2d estimatedPose = swervePose.getEstimatedPosition();

    return new Pose2d(estimatedPose.getTranslation(), getRotation2d());
  }

  public void setModuleStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.SwerveSim.maxSpeedMPS);
    for (int i = 0; i < 4; i++) {
      modules[i].setDesiredState(states[i]);
    }

    periodic();
  }

  public void setDrivetrainRotation(double rotationDiff) {
    drivetrainRotationRad = (drivetrainRotationRad + rotationDiff) % (2 * Math.PI);
  }

  public void updatePose() {
    SwerveModulePosition[] newPoseArr = { data[0].position, data[1].position, data[2].position, data[3].position };
    swervePose.update(getRotation2d(), newPoseArr);
  }

  @Override
  public void periodic() {
    for (int i = 0; i < 4; i++) {
      modules[i].updateData(data[i]);
    }
    updatePose();

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
    SmartDashboard.putNumber("Robot Pose X", getPose().getX());
    SmartDashboard.putNumber("Robot Pose Y", getPose().getY());
    SmartDashboard.putNumberArray("Odometry",
        new double[] { getPose().getX(), getPose().getY(), getPose().getRotation().getDegrees() });
  }

  @Override
  public void simulationPeriodic() {
  }
}