package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

public class SwerveModuleSim implements SwerveModuleIO {
  private FlywheelSim driveSim = new FlywheelSim(DCMotor.getNEO(1), 6.75, 0.025);
  private FlywheelSim turnSim = new FlywheelSim(DCMotor.getNEO(1), 150 / 7, 0.004);

  private PIDController drivePID = new PIDController(1, 0, 0);
  private PIDController turnPID = new PIDController(10, 0, 0);

  private SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0, 3);

  private SwerveModulePosition position = new SwerveModulePosition();
  private SwerveModuleState theoreticalState = new SwerveModuleState();
  private double drivePositionM = 0.0;
  private double driveAppliedVolts = 0.0;
  private int index = 0;

  private double turnPositionRad = 0.0;
  private double turnAppliedVolts = 0.0;


  public SwerveModuleSim() {
    turnPID.enableContinuousInput(0, 2 * Math.PI);
  }

  private SwerveModuleState getCurrState() {
    return new SwerveModuleState(driveSim.getAngularVelocityRadPerSec() * Constants.SwerveSim.wheelDiamM / 2,
        new Rotation2d(turnPositionRad));
  }

  @Override
  public void updateData(ModuleData data) {
    driveSim.update(0.02);
    turnSim.update(0.02);

    // SmartDashboard.putNumber("Drive vel " + data.index,
    // driveSim.getAngularVelocityRadPerSec() * (Constants.Swerve.wheelDiamM / 2));

    double meterDiff = driveSim.getAngularVelocityRadPerSec() * Constants.SwerveSim.wheelDiamM / 2 * 0.02;
    drivePositionM += meterDiff;
    data.driveVelocityMPerSec = driveSim.getAngularVelocityRadPerSec() * (Constants.SwerveSim.wheelDiamM / 2);
    data.driveCurrentAmps = driveSim.getCurrentDrawAmps();
    data.drivePositionM = drivePositionM;
    data.driveAppliedVolts = driveAppliedVolts;

    double angleDiff = turnSim.getAngularVelocityRadPerSec() * 0.02;
    turnPositionRad += angleDiff;
    data.turnPositionRad = turnPositionRad;
    data.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
    data.turnAppliedVolts = turnAppliedVolts;
    data.turnCurrentAmps = turnSim.getCurrentDrawAmps();
    data.turnAppliedVolts = turnAppliedVolts;

    data.position = new SwerveModulePosition(drivePositionM, new Rotation2d(turnPositionRad));

    while (turnPositionRad < 0) {
      turnPositionRad += 2.0 * Math.PI;
    }

    while (turnPositionRad > 2.0 * Math.PI) {
      turnPositionRad -= 2.0 * Math.PI;
    }

    data.theoreticalState = theoreticalState;
  }

  @Override
  public void setDesiredState(SwerveModuleState state) {
    state = SwerveModuleState.optimize(state, getCurrState().angle);
    theoreticalState = state;

    // SmartDashboard.putNumber("State Speed " + index,
    // theoreticalState.speedMetersPerSecond);

    if (Math.abs(state.speedMetersPerSecond) < 0.01) {
      state.speedMetersPerSecond = 0;
    }

    double driveFFVolts = driveFF.calculate(state.speedMetersPerSecond);
    double driveVolts = drivePID.calculate(driveSim.getAngularVelocityRadPerSec() * Constants.SwerveSim.wheelDiamM / 2,
        state.speedMetersPerSecond);
    double turnVolts = turnPID.calculate(turnPositionRad, state.angle.getRadians());

    // SmartDashboard.putNumber("Drive Volts" + index, driveFFVolts + driveVolts);

    setMotorVolts(driveFFVolts + driveVolts, driveSim);
    setMotorVolts(turnVolts, turnSim);
  }

  private void setMotorVolts(double volts, FlywheelSim flywheel) {
    double setVolts = volts; // MathUtil.clamp(volts, -7.0, 7.0);

    flywheel.setInputVoltage(setVolts);
  }

  @Override
  public void stop() {
    setMotorVolts(0.0, driveSim);
    setMotorVolts(0.0, turnSim);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
  }

  @Override
  public void setTurningBrakeMode(boolean enable) {
  }
}