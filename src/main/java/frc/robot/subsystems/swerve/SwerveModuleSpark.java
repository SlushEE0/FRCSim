package frc.robot.subsystems.swerve;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

public class SwerveModuleSpark implements SwerveModuleIO {
  private CANSparkMax driveMotor;
  private CANSparkMax turnMotor;

  private RelativeEncoder driveEncoder;
  private CANCoder absoluteEncoder;

  private PIDController drivePID = new PIDController(1, 0, 0);
  private PIDController turnPID = new PIDController(10, 0, 0);

  private SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0, 3);

  private SwerveModulePosition position = new SwerveModulePosition();
  private SwerveModuleState theoreticalState = new SwerveModuleState();
  private double drivePositionM = 0.0;
  private double driveAppliedVolts = 0.0;

  private double turnPositionRad = 0.0;
  private double turnAppliedVolts = 0.0;
  int index = 0;

  public SwerveModuleSpark(int motorIndex) {
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

    driveEncoder = driveMotor.getEncoder();
    absoluteEncoder = new CANCoder(Constants.SwerveReal.absoluteEncoderPorts[index]);

    driveEncoder.setVelocityConversionFactor(Constants.SwerveReal.driveGearRatio);
    absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
  }

  private SwerveModuleState getCurrState() {
    return new SwerveModuleState(
        (driveEncoder.getVelocity() * 2 * Math.PI / 60) * Constants.SwerveReal.wheelDiamM / 2,
        new Rotation2d(turnPositionRad));
  }

  @Override
  public void updateData(ModuleData data) {
    data.driveVelocityMPerSec = driveEncoder.getVelocity() * Math.PI
        * Constants.SwerveReal.wheelDiamM;
    data.driveCurrentAmps = driveMotor.getOutputCurrent();
    data.drivePositionM = driveEncoder.getPosition() * Constants.SwerveReal.wheelDiamM * Math.PI;
    data.driveAppliedVolts = driveMotor.getBusVoltage();

    data.turnPositionRad = Math.toRadians(absoluteEncoder.getAbsolutePosition());
    data.turnVelocityRadPerSec = Math.toRadians(absoluteEncoder.getVelocity());
    data.turnAppliedVolts = turnMotor.getBusVoltage();
    data.turnCurrentAmps = turnMotor.getOutputCurrent();

    while (turnPositionRad < 0) {
      turnPositionRad += 2.0 * Math.PI;
    }

    while (turnPositionRad > 2.0 * Math.PI) {
      turnPositionRad -= 2.0 * Math.PI;
    }
  }

  @Override
  public void setDesiredState(SwerveModuleState state) {
    state = SwerveModuleState.optimize(state, getCurrState().angle);

    if (Math.abs(state.speedMetersPerSecond) < 0.01) {
      state.speedMetersPerSecond = 0;
    }

    double driveFFVolts = driveFF.calculate(state.speedMetersPerSecond);
    double driveVolts = drivePID.calculate(
        driveEncoder.getPosition(),
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
    driveMotor.set(0.0);
    turnMotor.set(0.0);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    driveMotor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void setTurningBrakeMode(boolean enable) {
    turnMotor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }
}