package frc.robot.commands.arm;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmJoystickCmd extends CommandBase {

  private final ArmSubsystem armSubsystem;
  Supplier<Double> rotationSetpoint, extensionSetpoint;

  public ArmJoystickCmd(ArmSubsystem armSubsystem, Supplier<Double> rotationSetpoint, Supplier<Double> extensionSetpoint) {
    this.armSubsystem = armSubsystem;
    this.rotationSetpoint = rotationSetpoint;
    this.extensionSetpoint = extensionSetpoint;
    
    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    armSubsystem.setArmExtensionMeters(extensionSetpoint.get());
    armSubsystem.setArmRotationRadians(rotationSetpoint.get());
  }

  @Override
  public void end(boolean interrupted) {
    armSubsystem.stopArm();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}