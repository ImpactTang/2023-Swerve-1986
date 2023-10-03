package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeForwardCmd extends CommandBase{

    private final IntakeSubsystem intakeSubsystem;


    public IntakeForwardCmd(IntakeSubsystem intakeSubsystem){
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        intakeSubsystem.startIntake();
    }

    @Override
    public void end(boolean interrupted){
        intakeSubsystem.stopIntake();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
    
}