package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.ArmConstants;

public class ArmRotationSubsystem extends SubsystemBase {

    private CANSparkMax rotateMotor;
    private SparkMaxAbsoluteEncoder rotateMotorEncoder;
    private CANCoder rotateCanCoder;

    private SparkMaxPIDController rotatePidController;

    private Solenoid armBrakeSolenoid;

    private final double armMaxRotation = Math.PI;
    private final double armMinRotation = 0.0;
    private double armRotationSetpoint = armMinRotation;

    public ArmRotationSubsystem() {

        rotateMotorConfig();

        armBrakeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 1);

        armRotationSetpoint = getArmRotationRadians();

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Rotation Motor Speed", rotateMotor.get());
        SmartDashboard.putNumber("Arm Rotation Degrees", rotateCanCoder.getPosition() + ArmConstants.rotateCanCoderOffset);
        SmartDashboard.putNumber("Arm Rotation Radians", getArmRotationRadians());
        SmartDashboard.putNumber("Arm Rotation Setpoint", armRotationSetpoint);
    }

    public void rotateArm(double speed) {
        rotateMotor.set(speed);
    }

    public void setArmRotation(double setpoint) {
        this.armRotationSetpoint = setpoint;
        // Put setpoint in radians

        if (setpoint > armMaxRotation){
            setpoint = armMaxRotation;
        } else if (setpoint < armMinRotation){
            setpoint = armMinRotation;
        }

        // Calculate feed forward based on angle to compensate for gravity
        double cosineScalar = Math.cos(getArmRotationRadians());
        double feedForward = ArmConstants.gravityFF * cosineScalar;
        rotatePidController.setReference(armRotationRadiansToRotations(setpoint),
        ControlType.kSmartMotion, 0, feedForward, ArbFFUnits.kPercentOut);
    }

    public double getArmRotationRadians(){
        return Units.degreesToRadians(rotateCanCoder.getPosition() + ArmConstants.rotateCanCoderOffset);
    }

    private double armRotationRadiansToRotations(double rotateRadians) {
        // Convert input radians to rotations, [0, 1]
        return Units.radiansToRotations(rotateRadians + Units.degreesToRadians(ArmConstants.rotateCanCoderOffset));
    }

    public void toggleArmBrake(){
        armBrakeSolenoid.toggle();
    }

    public void rotateMotorConfig(){
        
        rotateMotor = new CANSparkMax(ArmConstants.rotateMotorId, MotorType.kBrushless);
        rotateCanCoder = new CANCoder(ArmConstants.rotateCanCoderId, "rio");

        rotateMotor.restoreFactoryDefaults();
        rotateCanCoder.configFactoryDefault();

        rotateCanCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
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
    }
}
