package frc.robot.commands.arm;

import frc.robot.subsystems.ArmExtensionSubsystem;
import frc.robot.subsystems.ArmRotationSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.WristSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class StowCmd extends CommandBase {

  private final ArmRotationSubsystem armRotationSubsystem;
  private final ArmExtensionSubsystem armExtensionSubsystem;
  private final WristSubsystem wristSubsystem;

  public StowCmd(ArmRotationSubsystem armRotationSubsystem, ArmExtensionSubsystem armExtensionSubsystem, WristSubsystem wristSubsystem) {
    this.armRotationSubsystem = armRotationSubsystem;
    this.armExtensionSubsystem = armExtensionSubsystem;
    this.wristSubsystem = wristSubsystem;
    addRequirements(armRotationSubsystem, armExtensionSubsystem, wristSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    armRotationSubsystem.setArmRotation(Math.PI / 2.0);
    armExtensionSubsystem.setArmExtension(0);
    wristSubsystem.setWristPosition(Math.PI / 2.0);
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
