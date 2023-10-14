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
import frc.robot.utils.Constants.ModuleConstants;

public class SwerveModule{

    private final String moduleName;

    private final TalonFX driveMotor;
    private final TalonFX turnMotor;

    private final PIDController turningPidController;

    private final CANCoder turnCanCoder;

    private final double moduleWheelOffset;

    private final double absoluteEncoderOffsetRad;
    private final boolean absoluteEncoderReversed;

<<<<<<< HEAD
    public SwerveModule(int driveMotorId, int turnMotorId, boolean driveMotorReversed, boolean turnMotorReversed, double moduleWheelOffset, int turnCanCoderId, double absoluteEncoderOffsetRad, boolean absoluteEncoderReversed, String name){
        
        this.moduleName = name;

        this.moduleWheelOffset = moduleWheelOffset;
=======
    private final double moduleChassisOffset;

    public SwerveModule(int driveMotorId, int turnMotorId, boolean driveMotorReversed, boolean turnMotorReversed, double moduleChassisOffset, int turnCanCoderId, double absoluteEncoderOffsetRad, boolean absoluteEncoderReversed, String name){
        
        this.moduleName = name;

        this.moduleChassisOffset = moduleChassisOffset;
>>>>>>> 54714e9a3bf8a9c32020924cee1bb8e04f1a1abb
        this.absoluteEncoderOffsetRad = absoluteEncoderOffsetRad;
        this.absoluteEncoderReversed = absoluteEncoderReversed;

        /* Drive Motor Config */
        driveMotor = new TalonFX(driveMotorId, "Canivore");
        driveMotor.configFactoryDefault();
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            true, 35, 60, 0.1);
        
        driveMotor.configSupplyCurrentLimit(driveSupplyLimit);
        driveMotor.configOpenloopRamp(0.25);
        driveMotor.setInverted(driveMotorReversed);
        driveMotor.setNeutralMode(NeutralMode.Brake);

        /* Turn Motor Config */
        turnMotor = new TalonFX(turnMotorId, "Canivore");
        turnMotor.configFactoryDefault();
        SupplyCurrentLimitConfiguration turnSupplyLimit = new SupplyCurrentLimitConfiguration(
            true, 35, 60, 0.1);

        turnMotor.configSupplyCurrentLimit(turnSupplyLimit);
        turnMotor.configOpenloopRamp(0.25);
        turnMotor.setInverted(turnMotorReversed);
        turnMotor.setNeutralMode(NeutralMode.Brake);

        /* Turn CANCoder Config */
        turnCanCoder = new CANCoder(turnCanCoderId, "Canivore");
        turnCanCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        turnCanCoder.configSensorDirection(false);
        turnCanCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        turnCanCoder.configGetFeedbackTimeBase();

        /* PID Controller for Turning */
        turningPidController = new PIDController(0.5, 0, 0.0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        /* Timer so stuff can initialize before reset */
        Timer.delay(1);
        resetEncoders();

    }

    public double getDrivePosition(){
        return driveMotor.getSelectedSensorPosition() * ModuleConstants.kDriveMotorRot2Meter;
    }

    public double getTurningPosition(){
<<<<<<< HEAD
        return turnMotor.getSelectedSensorPosition() * ModuleConstants.kTurningMotorRot2Rad + moduleWheelOffset;
=======
        return turnMotor.getSelectedSensorPosition() * ModuleConstants.kTurningMotorRot2Rad + moduleChassisOffset;
>>>>>>> 54714e9a3bf8a9c32020924cee1bb8e04f1a1abb
    }

    public double getDriveVelocity(){
        return driveMotor.getSelectedSensorVelocity() * ModuleConstants.kDriveVelocity2MeterPerSec;
    }

    public double getTurnVelocity(){
        return turnCanCoder.getVelocity() * ModuleConstants.kTurningVelocity2RadPerSec + moduleWheelOffset;
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition() + moduleChassisOffset));
    }

    public void setDesiredState(SwerveModuleState desiredState){
        
        // Remove unwanted movement commands
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.001){
            stop();
            return;
        }

<<<<<<< HEAD
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
        driveMotor.set(TalonFXControlMode.PercentOutput, desiredState.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turnMotor.set(TalonFXControlMode.PercentOutput, turningPidController.calculate(getTurningPosition(), desiredState.angle.getRadians() + moduleWheelOffset));
        SmartDashboard.putString("Swerve["+moduleName+"] state", desiredState.toString());
=======
        SwerveModuleState correctedState = new SwerveModuleState();
        correctedState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedState.angle = desiredState.angle.plus(Rotation2d.fromRadians(moduleChassisOffset));

        correctedState = SwerveModuleState.optimize(correctedState, getPosition().angle);
        driveMotor.set(TalonFXControlMode.PercentOutput, correctedState.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turnMotor.set(TalonFXControlMode.PercentOutput, turningPidController.calculate(getTurningPosition(), correctedState.angle.getRadians()));
        SmartDashboard.putString("Swerve["+moduleName+"] state", correctedState.toString());
>>>>>>> 54714e9a3bf8a9c32020924cee1bb8e04f1a1abb
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition() + moduleChassisOffset));
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(turnCanCoder.getAbsolutePosition());
    }

    public void update(){
        SmartDashboard.putNumber(moduleName + "Absolute-Position", turnCanCoder.getAbsolutePosition() - moduleChassisOffset);
    }

    public double getAbsoluteEncoderRad(){
        double angle = turnCanCoder.getAbsolutePosition() * Math.PI / 180.0;
        return angle * (absoluteEncoderReversed ? -1 : 1);
    }

    public void resetEncoders(){
        driveMotor.setSelectedSensorPosition(0);
        turnMotor.setSelectedSensorPosition(((turnCanCoder.getAbsolutePosition() * Math.PI / 180.0) - absoluteEncoderOffsetRad) / 2048.0 + moduleWheelOffset);
    }
    
    public void stop(){
        driveMotor.set(TalonFXControlMode.PercentOutput, 0);
        turnMotor.set(TalonFXControlMode.PercentOutput, 0);
    }
}