package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.Constants.DriveConstants;
import frc.robot.utils.Conversions;

public class SwerveModule{

    private final String moduleName;

    private final TalonFX driveMotor;
    private final TalonFX turnMotor;

    private final PIDController turningPidController;

    private final CANCoder turnCanCoder;

    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorId, int turnMotorId, boolean driveMotorReversed, boolean turnMotorReversed, int turnCanCoderId, double absoluteEncoderOffsetRad, String name){
        
        this.moduleName = name;

        this.absoluteEncoderOffsetRad = absoluteEncoderOffsetRad;

        /* Drive Motor Config */
        driveMotor = new TalonFX(driveMotorId, "CANivore");
        driveMotor.configFactoryDefault();
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            true, 35, 60, 0.1);
        
        driveMotor.config_kP(0, 0.5);
        driveMotor.config_kI(0, 0.0);
        driveMotor.config_kD(0, 0.0);
        driveMotor.config_kF(0, 0.0);
        driveMotor.configSupplyCurrentLimit(driveSupplyLimit);
        driveMotor.configOpenloopRamp(0.25);
        driveMotor.setInverted(driveMotorReversed);
        driveMotor.setNeutralMode(NeutralMode.Brake);

        /* Turn Motor Config */
        turnMotor = new TalonFX(turnMotorId, "CANivore");
        turnMotor.configFactoryDefault();
        SupplyCurrentLimitConfiguration turnSupplyLimit = new SupplyCurrentLimitConfiguration(
            true, 35, 60, 0.1);
        
        turnMotor.config_kP(0, 0.5);
        turnMotor.config_kI(0, 0.0);
        turnMotor.config_kD(0, 0.0);
        turnMotor.config_kF(0, 0.0);
        turnMotor.configSupplyCurrentLimit(turnSupplyLimit);
        turnMotor.configOpenloopRamp(0.25);
        turnMotor.setInverted(turnMotorReversed);
        turnMotor.setNeutralMode(NeutralMode.Brake);

        /* Turn CANCoder Config */
        turnCanCoder = new CANCoder(turnCanCoderId, "CANivore");
        turnCanCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        turnCanCoder.configSensorDirection(false);
        turnCanCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        turnCanCoder.configGetFeedbackTimeBase();

        /* PID Controller for Turning */
        turningPidController = new PIDController(0.5, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        /* Timer so stuff can initialize before reset */
        Timer.delay(1);
        resetEncoders();

    }

    public double getDrivePosition(){
        return driveMotor.getSelectedSensorPosition();
    }

    public double getTurningPosition(){
        return turnCanCoder.getPosition();
    }

    public double getDriveVelocity(){
        return driveMotor.getSelectedSensorVelocity();
    }

    public double getTurnVelocity(){
        return turnCanCoder.getVelocity();
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state){
        
        // Remove unwanted movement commands
        if (Math.abs(state.speedMetersPerSecond) < 0.001){
            stop();
            return;
        }

        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(TalonFXControlMode.PercentOutput, state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turnMotor.set(TalonFXControlMode.PercentOutput, turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        SmartDashboard.putString("Swerve["+moduleName+"] state", state.toString());
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()));
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(turnCanCoder.getAbsolutePosition());
    }
  
    public void update(){
        SmartDashboard.putNumber(moduleName + "Absolute-Position", turnCanCoder.getAbsolutePosition());
    }

    public void resetEncoders(){
        driveMotor.setSelectedSensorPosition(0);

        // Reseting turn motor sensor to CanCoder Sensor
        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - (absoluteEncoderOffsetRad * (180.0 / Math.PI)), 12.8);
        turnMotor.setSelectedSensorPosition(absolutePosition);
    }
    
    public void stop(){
        driveMotor.set(TalonFXControlMode.PercentOutput, 0);
        turnMotor.set(TalonFXControlMode.PercentOutput, 0);
    }
}