package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.utils.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  private CANSparkMax intakeMotor;
  private RelativeEncoder intakeMotorEncoder;

  public IntakeSubsystem() {

    intakeMotor = new CANSparkMax(IntakeConstants.intakeMotorId, MotorType.kBrushless);
    intakeMotor.setIdleMode(IdleMode.kCoast);
    intakeMotor.setInverted(IntakeConstants.intakeMotorReversed);
    intakeMotor.setSmartCurrentLimit(IntakeConstants.intakeMotorStallCurrentLimit, IntakeConstants.intakeMotorFreeSpinCurrentLimit);
    intakeMotor.setOpenLoopRampRate(IntakeConstants.openLoopRampRate);
    intakeMotor.burnFlash();

    intakeMotorEncoder = intakeMotor.getEncoder();

  }

  @Override
  public void periodic() {
    updateSmartDashboard();
  }

  public void startIntake(){
    intakeMotor.set(1);
  }

  public void stopIntake(){
    intakeMotor.set(0);
  }

  public void reverseIntake(){
    intakeMotor.set(-0.5);
  }

  public void setIntakeSpeed(double speed){
    intakeMotor.set(speed);
  }

  public double getIntakeMotorSpeed(){
    return intakeMotor.get();
  }

  public void updateSmartDashboard() {
    SmartDashboard.putNumber("Intake Motor Speed", getIntakeMotorSpeed());
  }

}
