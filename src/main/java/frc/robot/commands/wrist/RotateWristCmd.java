package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSubsystem;

public class RotateWristCmd extends CommandBase {

  private final WristSubsystem wristSubsystem;
  private double radians;

  public RotateWristCmd(WristSubsystem wristSubsystem, double radians) {
    this.wristSubsystem = wristSubsystem;
    this.radians = radians;
    addRequirements(wristSubsystem);
  }

  @Override
  public void initialize() {
    wristSubsystem.setWristPosition(radians);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    wristSubsystem.stopWrist();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
