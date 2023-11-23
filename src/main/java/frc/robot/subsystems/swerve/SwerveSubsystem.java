package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveModuleIO.ModuleData;

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private final SwerveModuleIO[] modules = new SwerveModuleIO[4];
  private final ModuleData[] moduleData = new ModuleData[4];
  private double realAngle = 0;

  public SwerveSubsystem()
  {
    if(Constants.robot.isSim)
    {
      for(int i=0;i<4;i++)
      {
        modules[i] = new SwerveModuleSim();
        moduleData[i] = new ModuleData();
      }
    }
  }

  public void setModuleStates(SwerveModuleState[] states)
  {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Swerve.maxDriveSpeedMPS);
    for(int i=0;i<4;i++)
    {
      modules[i].setDesiredState(states[i]);
    }
    //System.out.println(modules);
  }

  public Rotation2d getRotation2d()
  {
    return new Rotation2d();
  }

  @Override 
  public void periodic()
  {
    for(int i=0;i<4;i++)
    {
      modules[i].updateData(moduleData[i]);
      SmartDashboard.putNumber("Module " + Integer.toString(i) + "drivevolt", moduleData[i].driveAppliedVolts);
      SmartDashboard.putNumber("Module " + Integer.toString(i) + "drivevelocity", moduleData[i].driveVelocityMPerSec);
    }

   // double[] realStates = new double[8];
    //double[] theoreticalStates = new double[8];

    /*
      get states from module data
      convert to chassis speed via swervedrivekinematic
      useomegaradian per sepcond to find the rotational change per loop: keep track of changes
     */

    /* 

    for(int i=0;i<8;i+=2)
    {
      realStates[i] = moduleData[i/2].turnPositionRad / Math.PI * 180;
      realStates[i+1] = moduleData[i/2].driveVelocityMPerSec;
      theoreticalStates[i] = moduleData[i/2].theoreticalState.angle.getDegrees();
      theoreticalStates[i+1] = moduleData[i/2].theoreticalState.speedMetersPerSecond;
    }
    */


    double[] realStates = {
      moduleData[0].turnPositionRad / Math.PI * 180,
      moduleData[0].driveVelocityMPerSec,
      moduleData[1].turnPositionRad / Math.PI * 180,
      moduleData[1].driveVelocityMPerSec,
      moduleData[2].turnPositionRad / Math.PI * 180,
      moduleData[2].driveVelocityMPerSec,
      moduleData[3].turnPositionRad / Math.PI * 180,
      moduleData[3].driveVelocityMPerSec
    };

    double[] theoreticalStates = {
      moduleData[0].theoreticalState.angle.getDegrees(),
      moduleData[0].theoreticalState.speedMetersPerSecond,
      moduleData[1].theoreticalState.angle.getDegrees(),
      moduleData[1].theoreticalState.speedMetersPerSecond,
      moduleData[2].theoreticalState.angle.getDegrees(),
      moduleData[2].theoreticalState.speedMetersPerSecond,
      moduleData[3].theoreticalState.angle.getDegrees(),
      moduleData[3].theoreticalState.speedMetersPerSecond,
    };
    SmartDashboard.putNumberArray("RealStates", realStates);
    SmartDashboard.putNumberArray("TheoreticalStates", theoreticalStates);
    //01
    //23 (swerve wheel visualization)

    SwerveModuleState frontLeftState = new SwerveModuleState(realStates[1],new Rotation2d(realStates[0] * Math.PI / 180));
    SwerveModuleState frontRightState = new SwerveModuleState(realStates[3],new Rotation2d(realStates[2]* Math.PI / 180));
    SwerveModuleState backLefState = new SwerveModuleState(realStates[5],new Rotation2d(realStates[4]* Math.PI / 180));
    SwerveModuleState backRighState = new SwerveModuleState(realStates[7],new Rotation2d(realStates[6]* Math.PI / 180));
    
    ChassisSpeeds realSpeeds = Constants.Swerve.driveKinematics.toChassisSpeeds(frontLeftState,frontRightState,backLefState,backRighState);
    double turnDiff = realSpeeds.omegaRadiansPerSecond * 0.02;
    realAngle += turnDiff;

    while (realAngle < 0) {
      realAngle += 2.0 * Math.PI;
  }
  while (realAngle > 2.0 * Math.PI) {
      realAngle -= 2.0 * Math.PI;
  }
    SmartDashboard.putNumber("RobotRotation", realAngle /Math.PI * 180);

    // this is the veloicty -> convert to number (position) or an angle SmartDashboard.putNumber("realRotation",realSpeeds.omegaRadiansPerSecond);

  }
  
}
