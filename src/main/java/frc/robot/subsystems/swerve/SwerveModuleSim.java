package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class SwerveModuleSim implements SwerveModuleIO {
  private FlywheelSim driveSim = new FlywheelSim(DCMotor.getNEO(0), 6.75, 0.025);
  private FlywheelSim turnSim = new FlywheelSim(DCMotor.getNEO(0), 150 / 7, 0.004);

  private PIDController drivePID = new PIDController(0, 0, 0);
  private PIDController turnPID = new PIDController(0, 0, 0);

  private SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0, 0);

  private SwerveModulePosition position = new SwerveModulePosition();
  private SwerveModuleState theoreticalState = new SwerveModuleState();
  private double drivePositionM = 0.0;
  private double driveVelocityMPerSec = 0.0;
  private double driveAppliedVolts = 0.0;
  private double driveCurrentAmps = 0.0;

  private double turnPositionRad = 0.0;
  private double turnVelocityRadPerSec = 0.0;
  private double turnAppliedVolts = 0.0;
  private double turnCurrentAmps = 0.0;

  @Override
  public void updateData(ModuleData data) {
    driveSim.update(0.02);
    turnSim.update(0.02);

    data.driveVelocityMPerSec = driveSim.getAngularVelocityRadPerSec() * (Constants.Swerve.wheelDiamM / 2);
    data.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();

    data.driveCurrentAmps = driveSim.getCurrentDrawAmps();
    data.turnCurrentAmps = turnSim.getCurrentDrawAmps();

    drivePositionM += driveSim.getAngularVelocityRadPerSec() * 0.02 * (Constants.Swerve.wheelDiamM / 2);
    data.drivePositionM = drivePositionM;

    turnPositionRad += (turnSim.getAngularVelocityRadPerSec() * 0.02);
    data.turnPositionRad = turnPositionRad;
  }

  @Override
  public void setDesiredState(SwerveModuleState state) {
    double driveFFVolts = driveFF.calculate(driveVelocityMPerSec, driveAppliedVolts);
    double driveVolts = drivePID.calculate(driveVelocityMPerSec, state.speedMetersPerSecond);
    double turnVolts = turnPID.calculate(turnPositionRad, state.angle.getRadians());

    driveSim.setInputVoltage(driveVolts + driveFFVolts);
    turnSim.setInputVoltage(turnVolts);
  }

  @Override
  public void stop() {
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
  }

  @Override
  public void setTurningBrakeMode(boolean enable) {
  }
}
