package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.utils.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {

    private CANSparkMax rotateMotor;
    private CANSparkMax extensionMotor;

    private RelativeEncoder extensionMotorEncoder;
    private CANCoder rotateCanCoder;

    private PIDController rotatePidController;
    private PIDController extensionPidController;

    private double rotationSetpointRadians;
    private double extensionSetpointMeters;

    private SlewRateLimiter rotateSlewRateLimiter;
    private SlewRateLimiter extensionSlewRateLimiter;

    public ArmSubsystem() {

        rotateMotor = new CANSparkMax(ArmConstants.rotateMotorId, MotorType.kBrushless);
        extensionMotor = new CANSparkMax(ArmConstants.extensionMotorId, MotorType.kBrushless);

        rotateMotor.setIdleMode(IdleMode.kBrake);
        extensionMotor.setIdleMode(IdleMode.kBrake);

        rotateMotor.setSmartCurrentLimit(ArmConstants.rotateMotorCurrentLimit);
        extensionMotor.setSmartCurrentLimit(ArmConstants.extensionMotorCurrentLimit);

        rotateMotor.setInverted(ArmConstants.rotateMotorInverted);
        extensionMotor.setInverted(ArmConstants.extensionMotorInverted);

        rotateMotor.setOpenLoopRampRate(ArmConstants.rotateMotorOpenLoopRampRate);
        extensionMotor.setOpenLoopRampRate(ArmConstants.extensionMotorOpenLoopRampRate);

        extensionMotorEncoder = extensionMotor.getEncoder();
        // Converting to meters
        extensionMotorEncoder.setPositionConversionFactor(0.02513274);
        
        rotateMotor.burnFlash();
        extensionMotor.burnFlash();

        rotateCanCoder = new CANCoder(ArmConstants.rotateCanCoderId);
        rotateCanCoder.configFactoryDefault();
        rotateCanCoder.configMagnetOffset(ArmConstants.rotateCanCoderOffset); // TODO: CONFIG OFFSET
        rotateCanCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        rotateCanCoder.configSensorDirection(ArmConstants.rotateCanCoderReversed);
        rotateCanCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        rotateCanCoder.configGetFeedbackTimeBase();

        rotatePidController = new PIDController(ArmConstants.rotatekP, ArmConstants.rotatekI, ArmConstants.rotatekD);
        extensionPidController = new PIDController(ArmConstants.extensionkP, ArmConstants.extensionkI, ArmConstants.extensionkD);

        rotateSlewRateLimiter = new SlewRateLimiter(5, -5, 0);
        extensionSlewRateLimiter = new SlewRateLimiter(5, -5, 0);

        rotationSetpointRadians = 0;
        extensionSetpointMeters = 0;
    }

    @Override
    public void periodic() {
        updateArmExtensionMeters();
        updateArmRotationRadians();
        SmartDashboard.putNumber("Arm Extension Setpoint", extensionSetpointMeters);
        SmartDashboard.putNumber("Arm Rotation Setpoint", rotationSetpointRadians);
        SmartDashboard.putNumber("Arm Extension Motor Speed", extensionMotor.get());
        SmartDashboard.putNumber("Arm Rotation Motor Speed", rotateMotor.get());
    }

    public void updateArmExtensionMeters(){

        if (extensionMotorEncoder.getPosition() < 0 || extensionSetpointMeters < 0 || extensionMotorEncoder.getPosition() > 1.4) {
            extensionMotor.set(0);
            System.out.println("Arm extension out of bounds");
            DriverStation.reportWarning("Arm extension out of bounds", false);
            return;
        }

        // Takes in elevator position in meters and setpoint in meters and outputs PID change
        double calculated = extensionPidController.calculate(extensionMotorEncoder.getPosition(), extensionSetpointMeters);

        // extensionMotor.set(calculated);

        calculated = extensionSlewRateLimiter.calculate(calculated);

        extensionMotor.set(calculated);
    }

    public void updateArmRotationRadians(){

        if (getArmRotateCanCoderRad() < -Math.PI / 2 || getArmRotateCanCoderRad() > Math.PI / 2 || rotationSetpointRadians < -Math.PI / 2 || rotationSetpointRadians > Math.PI / 2){
            rotateMotor.set(0);
            System.out.println("Arm rotation out of bounds");
            DriverStation.reportWarning("Arm rotation out of bounds", false);
            return;
        }

        double canCoderValueRad = rotateCanCoder.getPosition() * (2 * Math.PI / 4096.0);

        double calculated = rotatePidController.calculate(canCoderValueRad, rotationSetpointRadians);

        // rotateMotor.set(calculated);

        calculated = rotateSlewRateLimiter.calculate(calculated);

        rotateMotor.set(calculated);
    }

    public void setArmExtensionSetpoint(double input){
        
        extensionSetpointMeters = input;

        // Leave commented until further understanding
        // TODO: FIGURE THIS OUT
        // extensionSetpointMeters = input * 1.4;
    }

    public void setArmRotationRadians(double input){
        rotationSetpointRadians = input * Math.PI;
    }

    public void addArmRotationSetpoint(double input){
        rotationSetpointRadians += input;
    }

    public void subtractArmRotationSetpoint(double input){
        rotationSetpointRadians -= input;
    }

    public void setArmExtensionMeters(double meters){
        extensionSetpointMeters = meters;
    }

    public void addArmExtensionSetpoint(double input){
        extensionSetpointMeters += input;
    }

    public void subrtactArmExtensionSetpoint(double input){
        extensionSetpointMeters -= input;
    }

    public void setArmExtensionMotors(double input){
        extensionMotor.set(input);
    }

    public void setArmRotationMotors(double input){
        rotateMotor.set(input);
    }

    private double getArmRotateCanCoderRad(){
        return rotateCanCoder.getPosition() * (2 * Math.PI / 4096.0);
    }

    public double getArmExtensionMeters(){
        return extensionMotorEncoder.getPosition();
    }

    // Sets the arm rotation to stow 0 radians
    public boolean zeroArmRotation(){
        if (getArmRotateCanCoderRad() < -0.39){
            setArmRotationRadians(0.25);
        } else if (getArmRotateCanCoderRad() > 0.39){
            setArmRotationRadians(-0.25);
        } else {
            if (getArmRotateCanCoderRad() < -0.196){
                setArmRotationRadians(0.1);
            } else if (getArmRotateCanCoderRad() > 0.196){
                setArmRotationRadians(-0.1);
            } else {
                setArmRotationRadians(0);
                return true;
            }
        }
        return true;
    }

    public void stopArmExtension(){
        setArmExtensionMotors(0);
    }

    public void stopArmRotation(){
        setArmRotationMotors(0);
    }

    public void stopArm(){
        stopArmExtension();
        stopArmRotation();
    }

    public void updateSmartDashboard(){
        SmartDashboard.putNumber("Arm Extension Encoder", extensionMotorEncoder.getPosition());
        SmartDashboard.putNumber("Arm Rotation Encoder", getArmRotateCanCoderRad());
    }
}