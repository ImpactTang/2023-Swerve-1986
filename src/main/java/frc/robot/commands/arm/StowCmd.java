package frc.robot.commands.arm;

import frc.robot.subsystems.ArmSubsystem;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
      armSubsystem.setArmExtensionSetpoint(0);
      armSubsystem.setArmRotationRadians(0);
    }

    @Override
    public void end(boolean interrupted) {
      
    }

    @Override
    public boolean isFinished() {
      return false;
    }
}   
