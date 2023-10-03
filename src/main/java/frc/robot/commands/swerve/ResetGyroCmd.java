package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class ResetGyroCmd extends CommandBase{
    
    private final SwerveSubsystem swerveSubsystem;

    public ResetGyroCmd(SwerveSubsystem swerveSubsystem){
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        swerveSubsystem.resetHeading();
    }

    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}