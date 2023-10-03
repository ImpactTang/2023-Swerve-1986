package frc.robot.commands.arm;

import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class HighCmd extends CommandBase {
    private final ArmSubsystem armSubsystem;

    public HighCmd(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;

        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
      armSubsystem.setArmExtensionSetpoint(1.5);
      armSubsystem.setArmRotationRadians(0.5);
      if (armSubsystem.getArmExtensionMeters() > 3) {
        armSubsystem.setArmExtensionMeters(0);
      }
    }

    @Override
    public void end(boolean interrupted) {
      
    }

    @Override
    public boolean isFinished() {
      return false;
    }
}   
