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
      armSubsystem.setArmPosition(Math.PI);
    }

    @Override
    public void end(boolean interrupted) {
      
    }

    @Override
    public boolean isFinished() {
      return false;
    }
}   
