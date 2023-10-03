package frc.robot.commands.intake;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeButtonCmd extends CommandBase {

  private final IntakeSubsystem intakeSubsystem;

  Supplier<Boolean> button;

  public IntakeButtonCmd(IntakeSubsystem intakeSubsystem, Supplier<Boolean> button) {
    this.intakeSubsystem = intakeSubsystem;
    this.button = button;
    
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (button.get()) {
      intakeSubsystem.startIntake();
    } else {
      intakeSubsystem.stopIntake();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
