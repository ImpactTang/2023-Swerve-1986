package frc.robot.commands.arm;

import frc.robot.subsystems.ArmExtensionSubsystem;
import frc.robot.subsystems.ArmRotationSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class HighCmd extends CommandBase {

  private final ArmRotationSubsystem armRotationSubsystem;
  private final ArmExtensionSubsystem armExtensionSubsystem;

  public HighCmd(ArmRotationSubsystem armRotationSubsystem, ArmExtensionSubsystem armExtensionSubsystem) {
    this.armRotationSubsystem = armRotationSubsystem;
    this.armExtensionSubsystem = armExtensionSubsystem;
     addRequirements(armRotationSubsystem, armExtensionSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    armRotationSubsystem.setArmRotation(55 * (Math.PI / 180));
    armExtensionSubsystem.setArmExtension(20.0);
  }

  @Override
  public void end(boolean interrupted) {
    armRotationSubsystem.stopArm();
    armExtensionSubsystem.stopArm();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}   
