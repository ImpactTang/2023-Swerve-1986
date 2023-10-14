/*
 * Swerve Module Subsystem
 */

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

    private final TalonFX driveMotor;
    private final TalonFX turnMotor;
    private final CANCoder turnCanCoder;
    private final PIDController turningPidController;

    private final int driveMotorId;
    private final int turnMotorId;
    private final boolean driveMotorReversed;
    private final boolean turnMotorReversed;
    private final double moduleWheelOffset;
    private final int turnCanCoderId;
    private final double turnCanCoderOffsetRad;
    private final boolean turnCanCoderReversed;
    private final String moduleName;
    
    /* --------------------> Swerve Module Constructor <-------------------- */

    public SwerveModule(int driveMotorId, int turnMotorId, boolean driveMotorReversed, boolean turnMotorReversed, double moduleWheelOffset, int turnCanCoderId, double turnCanCoderOffsetRad, boolean turnCanCoderReversed, String name){
        
        /* --------------------> Assigning Variables <-------------------- */
        this.driveMotorId = driveMotorId;
        this.turnMotorId = turnMotorId;
        this.driveMotorReversed = driveMotorReversed;
        this.turnMotorReversed = turnMotorReversed;
        this.moduleWheelOffset = moduleWheelOffset;
        this.turnCanCoderId = turnCanCoderId;
        this.turnCanCoderOffsetRad = turnCanCoderOffsetRad;
        this.turnCanCoderReversed = turnCanCoderReversed;
        this.moduleName = name;

        /* --------------------> Configurations <-------------------- */

        /* Drive Motor Config */
        driveMotor = new TalonFX(driveMotorId, "Canivore");
        driveMotor.configFactoryDefault();
        driveMotorConfig(driveMotor);

        /* Turn Motor Config */
        turnMotor = new TalonFX(turnMotorId, "Canivore");
        turnMotor.configFactoryDefault();
        turnMotorConfig(turnMotor);

        /* Turn CANCoder Config */
        turnCanCoder = new CANCoder(turnCanCoderId, "Canivore");
        turnCanCoder.configFactoryDefault();
        turnCanCoderConfig(turnCanCoder);

        /* PID Controller for Turning */
        turningPidController = new PIDController(ModuleConstants.kModuleP, ModuleConstants.kModuleI, ModuleConstants.kModuleD);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        /* Timer so stuff can initialize before reset */
        Timer.delay(0.5);
        resetEncoders();

    }

    /* --------------------> Setting Module States <-------------------- */

    public void setDesiredState(SwerveModuleState desiredState){
        
        // Remove unwanted movement commands
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.001){
            stop();
            return;
        }

        desiredState = SwerveModuleState.optimize(desiredState, getState().angle); // Optimize the desired state to remove unnecessary turning

        // Set the drive and turn motor speeds
        driveMotor.set(TalonFXControlMode.PercentOutput, desiredState.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turnMotor.set(TalonFXControlMode.PercentOutput, turningPidController.calculate(getTurningPosition(), desiredState.angle.getRadians() + moduleWheelOffset));

        // Output Module States to Smart Dashboard
        SmartDashboard.putString("Swerve["+moduleName+"] state", desiredState.toString());
    }

    /* --------------------> Getting Module Positions and Velocities <-------------------- */

    public double getDrivePosition(){
        return driveMotor.getSelectedSensorPosition() * ModuleConstants.kDriveMotorRot2Meter;
    }

    public double getTurningPosition(){
        return turnMotor.getSelectedSensorPosition() * ModuleConstants.kTurningMotorRot2Rad + moduleWheelOffset;
    }

    public double getDriveVelocity(){
        return driveMotor.getSelectedSensorVelocity() * ModuleConstants.kDriveVelocity2MeterPerSec;
    }

    public double getTurnVelocity(){
        return turnCanCoder.getVelocity() * ModuleConstants.kTurningVelocity2RadPerSec + moduleWheelOffset;
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()));
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(turnCanCoder.getAbsolutePosition());
    }

    /* --------------------> Adjusting Falcon Encoders to CANCoder Values <-------------------- */

    public void resetEncoders(){
        driveMotor.setSelectedSensorPosition(0);
        turnMotor.setSelectedSensorPosition(((turnCanCoder.getAbsolutePosition() * Math.PI / 180.0) + turnCanCoderOffsetRad) / 2048.0);
    }
    
    /* --------------------> Stop Swerve Module <-------------------- */

    public void stop(){
        driveMotor.set(TalonFXControlMode.PercentOutput, 0);
        turnMotor.set(TalonFXControlMode.PercentOutput, 0);
    }
    
    /* --------------------> Updating Module CANCoder Values to Smart Dashboard <-------------------- */
    public void update(){ 
        SmartDashboard.putNumber(moduleName + "Absolute-Position", turnCanCoder.getAbsolutePosition()); 
    }

    
    /* --------------------> Configuring Drive Motor <-------------------- */

    private void driveMotorConfig(TalonFX driveMotor){
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            true, 35, 60, 0.1);
        
        driveMotor.configSupplyCurrentLimit(driveSupplyLimit);
        driveMotor.configOpenloopRamp(0.25);
        driveMotor.setInverted(driveMotorReversed);
        driveMotor.setNeutralMode(NeutralMode.Brake);
    }

    /* --------------------> Configuring Turn Motor <-------------------- */
    private void turnMotorConfig(TalonFX turnMotor){
        turnMotor.configFactoryDefault();
        SupplyCurrentLimitConfiguration turnSupplyLimit = new SupplyCurrentLimitConfiguration(
            true, 35, 60, 0.1);

        turnMotor.configSupplyCurrentLimit(turnSupplyLimit);
        turnMotor.configOpenloopRamp(0.25);
        turnMotor.setInverted(turnMotorReversed);
        turnMotor.setNeutralMode(NeutralMode.Brake);
    }

    /* --------------------> Configuring Turn CANCoder <-------------------- */
    private void turnCanCoderConfig(CANCoder turnCanCoder){
        turnCanCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        turnCanCoder.configMagnetOffset(turnCanCoderOffsetRad);
        turnCanCoder.configSensorDirection(turnCanCoderReversed);
        turnCanCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        turnCanCoder.configGetFeedbackTimeBase();
    }
}