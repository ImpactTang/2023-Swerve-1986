package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSubsystem;

public class DefaultWristCommand extends CommandBase {

  private final WristSubsystem wristSubsystem;

  public DefaultWristCommand(WristSubsystem wristSubsystem) {
    this.wristSubsystem = wristSubsystem;
    addRequirements(wristSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    wristSubsystem.setWristPosition(Math.PI / 2.0);
  }

  @Override
  public void end(boolean interrupted) {
    wristSubsystem.stopWrist();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
