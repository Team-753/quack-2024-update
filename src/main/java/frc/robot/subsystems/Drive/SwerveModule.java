package frc.robot.subsystems.Drive;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.robot.Config;

public class SwerveModule {

    public final String moduleName;
    private final TalonFX driveMotor;
    private final TalonFX turnMotor;
    private final double absoluteOffset;
    private final AnalogEncoder analogEncoder;
    private final Rotation2d xAngle;
    private SimpleMotorFeedforward staticFrictionFFController;
    private final VelocityVoltage m_request_velocity = new VelocityVoltage(0).withSlot(0);
    private final PositionVoltage m_request_position = new PositionVoltage(0).withSlot(0);


    public SwerveModule(SwerveModuleConfig swerveModuleConfig)
    {

        moduleName = swerveModuleConfig.name;
        driveMotor = new TalonFX(swerveModuleConfig.driveMotorID);
        turnMotor = new TalonFX(swerveModuleConfig.turnMotorID);
        analogEncoder = new AnalogEncoder(swerveModuleConfig.analogID);
        absoluteOffset = swerveModuleConfig.analogOffset;
        xAngle = swerveModuleConfig.xAngle;

        TalonFXConfiguration turnMotorConfig = new TalonFXConfiguration();
        turnMotorConfig.Slot0.kP = 1; //0.1
        turnMotorConfig.Slot0.kI = 0.01;
        turnMotorConfig.Slot0.kD = 0.0001;
        turnMotorConfig.Slot0.kV = 0;
        //turnMotorConfig.Slot0.integralZone = 500;
        //turnMotorConfig.Slot0.allowableClosedloopError = 0;
        turnMotorConfig.CurrentLimits.SupplyCurrentLimit = 35.0;
        turnMotor.getConfigurator().apply(turnMotorConfig, 50);

        TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();
        driveMotorConfig.Slot0.kP = 0.0005;
        driveMotorConfig.Slot0.kI = 0.0005;
        driveMotorConfig.Slot0.kD = 0.0000;
        driveMotorConfig.Slot0.kV = 1;//0.046;
        //driveMotorConfig.Slot0.integralZone = 500;
        //driveMotorConfig.Slot0.allowableClosedloopError = 0;
        driveMotorConfig.CurrentLimits.SupplyCurrentLimit = 37.5;
        driveMotor.getConfigurator().apply(driveMotorConfig, 50);

        driveMotor.setPosition(0, 50);
        turnMotor.setPosition(getAbsolutePosition() * Config.DriveConstants.turningGearRatio / 360, 50);

        staticFrictionFFController = new SimpleMotorFeedforward(Config.DriveConstants.swerveDriveFFkS, Config.DriveConstants.swerveDriveFFkV, Config.DriveConstants.swerveDriveFFkA);


    }

    public double getAbsolutePosition() {
        return (analogEncoder.getAbsolutePosition() * 360) - absoluteOffset;
    }
    public double getRawAbsolutePosition() {
        return analogEncoder.getAbsolutePosition() * 360;
    }
    public double getIntegratedPosition() {
        return ((turnMotor.getPosition().getValueAsDouble() % (Config.DriveConstants.turningGearRatio)) * 360) / (Config.DriveConstants.turningGearRatio);
    }

    private Rotation2d getIntegratedState() {
        return Rotation2d.fromDegrees(((turnMotor.getPosition().getValueAsDouble() % (Config.DriveConstants.turningGearRatio)) * 360) / (Config.DriveConstants.turningGearRatio));
    }

    public void setState(SwerveModuleState state) {
        SwerveModuleState desiredState = SwerveModuleState.optimize(state, getIntegratedState());
        double velocity = desiredState.speedMetersPerSecond;
        if (Math.abs(velocity) < 0.1) { // lowest speed is 0.1 m/s
            driveMotor.set(0);
        }
        else {
            velocity = (velocity + staticFrictionFFController.calculate(velocity)) * Config.DriveConstants.drivingGearRatio / (Config.DriveConstants.wheelDiameter * Math.PI * 10); // converting from m/s to ticks / 100ms
            driveMotor.setControl(m_request_velocity.withVelocity(velocity));
        }
        double angleDegrees = desiredState.angle.getDegrees();
        if (angleDegrees < 0) {
            angleDegrees += 360;
        }
        double turnPositionTicks = turnMotor.getPosition().getValueAsDouble();
        turnMotor.setControl(m_request_position.withPosition((turnPositionTicks - (turnPositionTicks % (Config.DriveConstants.turningGearRatio))) + (angleDegrees * Config.DriveConstants.turningGearRatio / 360)));
    }

    public void Coast() {
        turnMotor.setNeutralMode(NeutralModeValue.Coast);
        driveMotor.setNeutralMode(NeutralModeValue.Coast);
        Stop();
    }

    public void Brake() {
        turnMotor.setNeutralMode(NeutralModeValue.Brake);
        driveMotor.setNeutralMode(NeutralModeValue.Brake);
        Stop();
    }

    public void XMode() {
        driveMotor.setNeutralMode(NeutralModeValue.Coast);
        setState(new SwerveModuleState(0.0, xAngle));
    }

    private void Stop() {
        turnMotor.set(0);
        driveMotor.set(0);
    }

    public SwerveModulePosition getSwerveModulePosition() {
        double distanceMeters = driveMotor.getPosition().getValueAsDouble() * Config.DriveConstants.wheelDiameter * Math.PI / (Config.DriveConstants.drivingGearRatio);
        Rotation2d angle = getIntegratedState();
        return new SwerveModulePosition(distanceMeters, angle);
    }

    public SwerveModuleState getSwerveModuleState() {
        double velocity = (driveMotor.getVelocity().getValueAsDouble() * 10 * Config.DriveConstants.wheelDiameter * Math.PI) / (Config.DriveConstants.drivingGearRatio);
        Rotation2d angle = getIntegratedState();
        return new SwerveModuleState(velocity, angle);
    }
}
