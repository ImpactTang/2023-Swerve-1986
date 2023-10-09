package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.utils.Constants.ArmConstants;

public class ArmExtensionSubsystem extends SubsystemBase {

    private CANSparkMax extensionMotor;

    private SparkMaxAbsoluteEncoder extensionMotorEncoder;

    private SparkMaxPIDController extensionPidController;

    private final double armMaxExtension = Units.inchesToMeters(35.0);
    private final double armMinExtension = 0.0;
    private double armExtensionSetpoint = armMinExtension;

    public ArmExtensionSubsystem() {

        extensionMotorConfig();

        armExtensionSetpoint = 0.0;

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Extension Motor Speed", extensionMotor.get());
        SmartDashboard.putNumber("Arm Extension setpoint", armExtensionSetpoint);
    }

    public void extendArm(double speed) {
        extensionMotor.set(speed);
    }

    public void setArmExtension(double armExtensionSetpoint) {
        this.armExtensionSetpoint = armExtensionSetpoint;
        // Put setpoint in meters
        extensionPidController.setReference(setExtensionSetpoint(armExtensionSetpoint), ControlType.kSmartMotion);
    }

    private double setExtensionSetpoint(double armExtensionSetpoint){
        if (armExtensionSetpoint > armMaxExtension){
            return armExtensionSetpoint = armMaxExtension;
        } else if (armExtensionSetpoint < armMinExtension){
            return armExtensionSetpoint = armMinExtension;
        } else {
            return armExtensionSetpoint;
        }
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

    public void stopArm(){
        extensionMotor.set(0);
    }
}
