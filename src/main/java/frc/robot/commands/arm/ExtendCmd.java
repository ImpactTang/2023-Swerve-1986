package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmExtensionSubsystem;

public class ExtendCmd extends CommandBase {

  private final ArmExtensionSubsystem armExtensionSubsystem;
  private double setpoint;

  public ExtendCmd(ArmExtensionSubsystem armExtensionSubsystem, double setpoint) {
    this.armExtensionSubsystem = armExtensionSubsystem;
    this.setpoint = setpoint;
    addRequirements(armExtensionSubsystem);
  }

  @Override
  public void initialize() {
    armExtensionSubsystem.setArmExtension(setpoint);
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
