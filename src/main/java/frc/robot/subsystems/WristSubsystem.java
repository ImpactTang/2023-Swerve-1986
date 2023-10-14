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
import frc.robot.utils.Constants.WristConstants;

public class WristSubsystem extends SubsystemBase {

  private CANSparkMax wristMotor;
  private SparkMaxAbsoluteEncoder wristMotorEncoder;
  private SparkMaxPIDController wristPidController;
  private CANCoder wristCanCoder;

  private final double wristMaxRotationRadians = Math.PI;
  private final double wristMinRotationRadians = 0.0;
  private Double wristSetpoint = null;

  public WristSubsystem() {

    wristMotor = new CANSparkMax(WristConstants.wristMotorId, MotorType.kBrushless);
    wristMotor.restoreFactoryDefaults();

    wristCanCoder = new CANCoder(WristConstants.wristCanCoderId);
    wristCanCoder.configFactoryDefault();

    wristCanCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    wristCanCoder.configSensorDirection(WristConstants.wristCanCoderReversed);
    wristCanCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    wristCanCoder.configGetFeedbackTimeBase();

    wristSetpoint = Units.degreesToRadians(wristCanCoder.getAbsolutePosition() + WristConstants.wristCanCoderOffset);

    wristMotor.setInverted(WristConstants.wristMotorReversed);
    wristMotorEncoder = wristMotor.getAbsoluteEncoder(Type.kDutyCycle);
    wristPidController = wristMotor.getPIDController();
    wristPidController.setFeedbackDevice(wristMotorEncoder);

    wristPidController.setP(WristConstants.kP);
    wristPidController.setI(WristConstants.kI);
    wristPidController.setD(WristConstants.kD);
    wristPidController.setIZone(WristConstants.kIz);
    wristPidController.setFF(WristConstants.kFF);
    wristPidController.setOutputRange(WristConstants.wristMinOutput, WristConstants.wristMaxOutput);

    wristPidController.setSmartMotionMaxVelocity(WristConstants.wristMaxVel, 0);
    wristPidController.setSmartMotionMinOutputVelocity(WristConstants.wristMinVel, 0);
    wristPidController.setSmartMotionMaxAccel(WristConstants.wristMaxAccel, 0);
    wristPidController.setSmartMotionAllowedClosedLoopError(WristConstants.allowedError, 0);

    wristMotor.setSmartCurrentLimit(WristConstants.wristMotorCurrentLimit);
    wristMotor.setIdleMode(IdleMode.kBrake);
    wristMotor.burnFlash();
  }

  @Override
  public void periodic() {
    if (wristSetpoint != null){
      if (wristSetpoint > wristMaxRotationRadians){
        wristSetpoint = wristMaxRotationRadians;
      } else if (wristSetpoint < wristMinRotationRadians){
        wristSetpoint = wristMinRotationRadians;
      }

      double cosineScalar = Math.cos(getWristPosition());
      double feedForward = WristConstants.gravityFF * cosineScalar;
      wristPidController.setReference(rotateRadiansToRotations(wristSetpoint), ControlType.kSmartMotion, 0, feedForward, ArbFFUnits.kPercentOut);
    }

    SmartDashboard.putNumber("Wrist Position Radians", getWristPosition());
    SmartDashboard.putNumber("Wrist Position Raw", wristCanCoder.getAbsolutePosition());
    SmartDashboard.putNumber("Wrist Setpoint", wristSetpoint);
  }

  public void rotateWrist(double speed){
    wristMotor.set(speed);
  }

  public void jogRight(){
    wristSetpoint -= Units.degreesToRadians(12);
  }

  public void jogLeft(){
    wristSetpoint += Units.degreesToRadians(12);
  }

  public void setWristPosition(double radians){
    this.wristSetpoint = radians;
    wristSetpoint = radians;
  }

  public double getWristPosition(){
    return Units.degreesToRadians(wristCanCoder.getPosition() + WristConstants.wristCanCoderOffset);
  }

  private double rotateRadiansToRotations(double rotateRadians) {
    // Convert input radians to rotations, [0, 1]
    return Units.radiansToRotations(rotateRadians + Units.degreesToRadians(WristConstants.wristCanCoderOffset));
  }

  public void stopWrist(){
    wristMotor.set(0);
  }

}
