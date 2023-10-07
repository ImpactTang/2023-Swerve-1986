package frc.robot.commands.arm;

import frc.robot.subsystems.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class StowCmd extends CommandBase {
    private final ArmSubsystem armSubsystem;

    public StowCmd(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;

        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
      armSubsystem.setArmRotation(Math.PI / 2.0);
      armSubsystem.setArmExtension(0);
    }

    @Override
    public void end(boolean interrupted) {
      armSubsystem.stopArm();
    }

    @Override
    public boolean isFinished() {
      return true;
    }
}   
