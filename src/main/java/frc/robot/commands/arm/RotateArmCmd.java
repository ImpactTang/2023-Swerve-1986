package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmRotationSubsystem;

public class RotateArmCmd extends CommandBase {

  private final ArmRotationSubsystem armRotationSubsystem;
  private double radians;

  public RotateArmCmd(ArmRotationSubsystem armRotationSubsystem, double radians) {
    this.armRotationSubsystem = armRotationSubsystem;
    this.radians = radians;
    addRequirements(armRotationSubsystem);
  }

  @Override
  public void initialize() {
    armRotationSubsystem.setArmRotation(radians);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    armRotationSubsystem.stopArm();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
