package frc.robot.subsystems;


import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.math.util.Units.radiansToRotations;

import frc.robot.utils.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {

    private CANSparkMax rotateMotor;
    private CANSparkMax extensionMotor;

    // private RelativeEncoder extensionMotorEncoder;
    private SparkMaxAbsoluteEncoder rotateMotorEncoder;
    private CANCoder rotateCanCoder;

    private SparkMaxPIDController rotatePidController;
    // private PIDController extensionPidController;

    // private double rotationSetpointRadians;
    // private double extensionSetpointMeters;

    // private SlewRateLimiter rotateSlewRateLimiter;
    // private SlewRateLimiter extensionSlewRateLimiter;

    private Double rotateSetpoint = null;

    public ArmSubsystem() {

        rotateMotor = new CANSparkMax(ArmConstants.rotateMotorId, MotorType.kBrushless);
        rotateCanCoder = new CANCoder(ArmConstants.rotateCanCoderId);

        rotateMotor.restoreFactoryDefaults();
        rotateCanCoder.configFactoryDefault();

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

    @Override
    public void periodic() {

        if (rotateSetpoint != null) {
            // Calculate feed forward based on angle to compensate for gravity
            double cosineScalar = Math.cos(getArmRotation());
            double feedForward = ArmConstants.gravityFF * cosineScalar;
            rotatePidController.setReference(armRadiansToEncoderRotations(rotateSetpoint),
                ControlType.kSmartMotion, 0, feedForward, ArbFFUnits.kPercentOut);

        }

        SmartDashboard.putNumber("Arm Extension Motor Speed", extensionMotor.get());
        SmartDashboard.putNumber("Arm Rotation Motor Speed", rotateMotor.get());
    }

    public void rotateArm(double speed) {
        rotateSetpoint = null;
        rotateMotor.set(speed);
    }

    public void setRotateSetpoint(double setpoint) {
        rotateSetpoint = setpoint; // SETPOINT IN RADIANS
    }

    public double getArmRotation(){
        return Units.rotationsToRadians(rotateCanCoder.getPosition() + ArmConstants.rotateCanCoderOffset);
    }

    static double armRadiansToEncoderRotations(double rotateRadians) {
        return radiansToRotations(rotateRadians) - ArmConstants.rotateCanCoderOffset;
    }

    public void stop(){
        rotateSetpoint = null;
        rotateMotor.set(0);
    }
}
