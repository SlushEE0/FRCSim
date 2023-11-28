package frc.robot.subsystems.swerve;

import javax.swing.text.Position;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

public class SwerveModuleReal implements SwerveModuleIO {
  private CANSparkMax driveMotor;
  private CANSparkMax turnMotor;

  private PIDController drivePID = new PIDController(1, 0, 0);
  private PIDController turnPID = new PIDController(10, 0, 0);

  private SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0, 3);

  private SwerveModulePosition position = new SwerveModulePosition();
  private SwerveModuleState theoreticalState = new SwerveModuleState();
  private double drivePositionM = 0.0;
  private double driveAppliedVolts = 0.0;

  private double turnPositionRad = 0.0;
  private double turnAppliedVolts = 0.0;
  private int index = 0;

  public SwerveModuleReal(int motorIndex) {
    turnPID.enableContinuousInput(0, 2 * Math.PI);

    int driveMotorPort = Constants.SwerveReal.driveMotorPorts[motorIndex];
    int turnMotorPort = Constants.SwerveReal.turnMotorPorts[motorIndex];

    index = motorIndex;
    driveMotor = new CANSparkMax(Math.abs(driveMotorPort), MotorType.kBrushless);
    turnMotor = new CANSparkMax(Math.abs(turnMotorPort), MotorType.kBrushless);

    if (driveMotorPort < 0) {
      driveMotor.setInverted(true);
    } else {
      driveMotor.setInverted(false);
    }

    if (turnMotorPort < 0) {
      turnMotor.setInverted(true);
    } else {
      turnMotor.setInverted(false);
    }
  }

  private SwerveModuleState getCurrState() {
    return new SwerveModuleState(
        (driveMotor.get() * Constants.SwerveReal.maxSpeedMPS) * Constants.SwerveReal.wheelDiamM / 2,
        new Rotation2d(turnPositionRad));
  }

  @Override
  public void updateData(ModuleData data) {
    driveMotor.setControlFramePeriodMs(20);
    turnMotor.setControlFramePeriodMs(20);

    double meterDiff = (driveMotor.get() * Constants.SwerveReal.maxSpeedMPS) * Constants.SwerveReal.wheelDiamM / 2
        * 0.02;
    drivePositionM += meterDiff;
    data.driveVelocityMPerSec = (driveMotor.get() * Constants.SwerveReal.maxSpeedMPS)
        * (Constants.SwerveReal.wheelDiamM / 2);
    data.driveCurrentAmps = driveMotor.getOutputCurrent();
    data.drivePositionM = drivePositionM;
    data.driveAppliedVolts = driveAppliedVolts;

    double angleDiff = (turnMotor.get() * Constants.SwerveReal.maxSpeedMPS) * 0.02;
    turnPositionRad += angleDiff;
    data.turnPositionRad = turnPositionRad;
    data.turnVelocityRadPerSec = (turnMotor.get() * Constants.SwerveReal.maxSpeedMPS);
    data.turnAppliedVolts = turnAppliedVolts;
    data.turnCurrentAmps = turnMotor.getOutputCurrent();
    data.turnAppliedVolts = turnAppliedVolts;

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
    double driveVolts = drivePID.calculate(
        (driveMotor.get() * Constants.SwerveReal.maxSpeedMPS) * Constants.SwerveReal.wheelDiamM / 2,
        state.speedMetersPerSecond);
    double turnVolts = turnPID.calculate(turnPositionRad, state.angle.getRadians());

    // SmartDashboard.putNumber("Drive Volts" + index, driveFFVolts + driveVolts);

    setMotorVolts(driveFFVolts + driveVolts, driveMotor);
    setMotorVolts(turnVolts, turnMotor);
  }

  private void setMotorVolts(double volts, CANSparkMax motor) {
    motor.setVoltage(volts);
  }

  @Override
  public void stop() {
    driveMotor.setVoltage(0.0);
    turnMotor.setVoltage(0.0);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
  }

  @Override
  public void setTurningBrakeMode(boolean enable) {
  }
}