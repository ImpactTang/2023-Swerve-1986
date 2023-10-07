package frc.robot.subsystems;


import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.math.util.Units.radiansToRotations;

import frc.robot.utils.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {

    private CANSparkMax rotateMotor;
    private CANSparkMax extensionMotor;

    private SparkMaxAbsoluteEncoder extensionMotorEncoder;
    private SparkMaxAbsoluteEncoder rotateMotorEncoder;
    private CANCoder rotateCanCoder;

    private SparkMaxPIDController rotatePidController;
    private SparkMaxPIDController extensionPidController;

    private double armRotationSetpoint;
    private final double armMaxExtension = Units.inchesToMeters(35.0);
    private final double armMinExtension = 0.0;
    private double armExtensionSetpoint = armMinExtension;

    public ArmSubsystem() {

        extensionMotorConfig();
        rotateMotorConfig();

        armExtensionSetpoint = 0.0;

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Extension Motor Speed", extensionMotor.get());
        SmartDashboard.putNumber("Arm Rotation Motor Speed", rotateMotor.get());
        SmartDashboard.putNumber("Arm Rotation Raw", rotateCanCoder.getPosition());
        SmartDashboard.putNumber("Arm Rotation Radians", getArmRotationRadians());
        SmartDashboard.putNumber("Arm setpoint", armRotationSetpoint);
        SmartDashboard.putNumber("Arm Extension setpoint", armExtensionSetpoint);
    }

    public void rotateArm(double speed) {
        rotateMotor.set(speed);
    }

    public void extendArm(double speed) {
        extensionMotor.set(speed);
    }

    public void setArmRotation(double setpoint) {
        this.armRotationSetpoint = setpoint;
        // Put setpoint in radians, 
        // Calculate feed forward based on angle to compensate for gravity
        double cosineScalar = Math.cos(getArmRotationRadians());
        double feedForward = ArmConstants.gravityFF * cosineScalar;
        rotatePidController.setReference(rotateRadiansToRotations(setpoint),
        ControlType.kSmartMotion, 0, feedForward, ArbFFUnits.kPercentOut);
    }

    public void setArmExtension(double armExtensionSetpoint) {
        // Put setpoint in meters
        extensionPidController.setReference(setExtensionSetpoint(armExtensionSetpoint), ControlType.kSmartMotion);
    }

    public double setExtensionSetpoint(double armExtensionSetpoint){
        if (armExtensionSetpoint > armMaxExtension){
            return armExtensionSetpoint = armMaxExtension;
        } else if (armExtensionSetpoint < armMinExtension){
            return armExtensionSetpoint = armMinExtension;
        } else {
            return armExtensionSetpoint;
        }
    }

    public double getArmRotationRadians(){
        return Units.degreesToRadians(rotateCanCoder.getPosition() + ArmConstants.rotateCanCoderOffset);
    }

    private double rotateRadiansToRotations(double rotateRadians) {
        // Convert input radians to rotations, [0, 1]
        return radiansToRotations(rotateRadians + Units.degreesToRadians(ArmConstants.rotateCanCoderOffset));
    }

    public double getArmExtension(){
        return extensionMotorEncoder.getPosition();
    }

    public void extensionMotorConfig(){

        extensionMotor = new CANSparkMax(ArmConstants.extensionMotorId, MotorType.kBrushless);
        extensionMotor.restoreFactoryDefaults();

        extensionMotorEncoder = extensionMotor.getAbsoluteEncoder(Type.kDutyCycle);
        extensionMotor.setInverted(ArmConstants.extensionMotorReversed);
        extensionMotorEncoder.setPositionConversionFactor(ArmConstants.extensionEncoderConversionFactor);
        extensionPidController = extensionMotor.getPIDController();
        extensionPidController.setFeedbackDevice(extensionMotorEncoder);

        extensionPidController.setP(ArmConstants.extensionkP);
        extensionPidController.setI(ArmConstants.extensionkI);
        extensionPidController.setD(ArmConstants.extensionkD);
        extensionPidController.setIZone(ArmConstants.extensionkIz);
        extensionPidController.setFF(ArmConstants.extensionkFF);
        extensionPidController.setOutputRange(ArmConstants.extensionMinOutput, ArmConstants.extensionMaxOutput);

        extensionPidController.setSmartMotionMaxVelocity(ArmConstants.extensionMaxVel, 0);
        extensionPidController.setSmartMotionMinOutputVelocity(ArmConstants.extensionMinVel, 0);
        extensionPidController.setSmartMotionMaxAccel(ArmConstants.extensionMaxAcc, 0);
        extensionPidController.setSmartMotionAllowedClosedLoopError(ArmConstants.allowedErr, 0);

        extensionMotor.setSmartCurrentLimit(50);
        extensionMotor.setIdleMode(IdleMode.kBrake);
        extensionMotor.burnFlash();

    }

    public void rotateMotorConfig(){
        
        rotateMotor = new CANSparkMax(ArmConstants.rotateMotorId, MotorType.kBrushless);
        rotateCanCoder = new CANCoder(ArmConstants.rotateCanCoderId, "rio");

        rotateMotor.restoreFactoryDefaults();
        rotateCanCoder.configFactoryDefault();

        rotateCanCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        rotateCanCoder.configSensorDirection(ArmConstants.rotateCanCoderReversed);
        rotateCanCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        rotateCanCoder.configGetFeedbackTimeBase();

        rotateMotor.setInverted(ArmConstants.rotateMotorReversed);

        rotateMotorEncoder = rotateMotor.getAbsoluteEncoder(Type.kDutyCycle);
        rotatePidController = rotateMotor.getPIDController();
        rotatePidController.setFeedbackDevice(rotateMotorEncoder);

        rotatePidController.setP(ArmConstants.rotatekP);
        rotatePidController.setI(ArmConstants.rotatekI);
        rotatePidController.setD(ArmConstants.rotatekD);
        rotatePidController.setIZone(ArmConstants.rotatekIz);
        rotatePidController.setFF(ArmConstants.rotatekFF);
        rotatePidController.setOutputRange(ArmConstants.rotateMinOutput, ArmConstants.rotateMaxOutput);

        rotatePidController.setSmartMotionMaxVelocity(ArmConstants.rotateMaxVel, 0);
        rotatePidController.setSmartMotionMinOutputVelocity(ArmConstants.rotateMinVel, 0);
        rotatePidController.setSmartMotionMaxAccel(ArmConstants.rotateMaxAcc, 0);
        rotatePidController.setSmartMotionAllowedClosedLoopError(ArmConstants.allowedErr, 0);

        rotateMotor.setSmartCurrentLimit(50);
        rotateMotor.setIdleMode(IdleMode.kBrake);
        rotateMotor.burnFlash();

    }

    public void stopArm(){
        rotateMotor.set(0);
        extensionMotor.set(0);
    }
}
