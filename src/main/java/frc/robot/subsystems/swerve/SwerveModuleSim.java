package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

public class SwerveModuleSim implements SwerveModuleIO {
    private FlywheelSim driveSim = new FlywheelSim(DCMotor.getNEO(1), 6.75, 0.025);
    private FlywheelSim turnSim = new FlywheelSim(DCMotor.getNEO(1), 150.0 / 7.0, 0.004);

    private PIDController drivePID = new PIDController(1.5, 0, 0);
    private SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0, 2.93);
    private PIDController turnPID = new PIDController(6, 0, 0);

    public SwerveModuleSim() {
        
        turnPID.enableContinuousInput(0, 2 * Math.PI);
    }

    private SwerveModuleState theoreticalState = new SwerveModuleState();
    private double drivePositionM = 0.0;
    private double driveAppliedVolts = 0.0;

    private double turnPositionRad = 0.0;
    private double turnAppliedVolts = 0.0;

    // check swervemoduleio for the comments
    @Override
    public void updateData(ModuleData data) {
        driveSim.update(0.02);
        turnSim.update(0.02);

        double meterDiff = driveSim.getAngularVelocityRadPerSec() * Constants.Swerve.wheelDiameterM / 2 * 0.02;
        drivePositionM += meterDiff;
        data.driveVelocityMPerSec = driveSim.getAngularVelocityRadPerSec() * (Constants.Swerve.wheelDiameterM / 2);
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


        while (turnPositionRad < 0) {
            turnPositionRad += 2.0 * Math.PI;
        }
        while (turnPositionRad > 2.0 * Math.PI) {
            turnPositionRad -= 2.0 * Math.PI;
        }
        data.theoreticalState = theoreticalState;
    }

    private SwerveModuleState getState() {
        return new SwerveModuleState(driveSim.getAngularVelocityRadPerSec() * Constants.Swerve.wheelDiameterM / 2,
                new Rotation2d(turnPositionRad));
    }

    @Override
    public void setDesiredState(SwerveModuleState state) {

        state = SwerveModuleState.optimize(state,getState().angle);
        theoreticalState = state;

        if(Math.abs(state.speedMetersPerSecond) < 0.01)
        {
            state.speedMetersPerSecond = 0;
        }
        double driveVolts = driveFF.calculate(state.speedMetersPerSecond) 
            + drivePID.calculate(driveSim.getAngularVelocityRadPerSec() * Constants.Swerve.wheelDiameterM / 2,
            state.speedMetersPerSecond) ;
        double turnVolts = turnPID.calculate(turnPositionRad, state.angle.getRadians());
        setDriveVoltage(driveVolts);
        setTurnVoltage(turnVolts);

    }

    @Override
    public void stop() {
        setDriveVoltage(0);
        setTurnVoltage(0);
    }

    private void setDriveVoltage(double volts) {
        driveAppliedVolts = volts;
        driveSim.setInputVoltage(driveAppliedVolts);
    }

    private void setTurnVoltage(double volts) {
        turnAppliedVolts = volts;
        turnSim.setInputVoltage(turnAppliedVolts);
    }

    @Override
    public void setDriveBrakeMode(boolean enable) {
    }

    @Override
    public void setTurningBrakeMode(boolean enable) {
    }
}
