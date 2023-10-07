package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmRotationSubsystem;

public class ToggleArmBrake extends CommandBase {

  private final ArmRotationSubsystem armRotationSubsystem;

  public ToggleArmBrake(ArmRotationSubsystem armRotationSubsystem) {
    this.armRotationSubsystem = armRotationSubsystem;
    addRequirements(armRotationSubsystem);
  }

  @Override
  public void initialize() {
    armRotationSubsystem.toggleArmBrake();
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
