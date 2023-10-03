package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeHoldCmd extends CommandBase{

    private final IntakeSubsystem intakeSubsystem;


    public IntakeHoldCmd(IntakeSubsystem intakeSubsystem){
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        intakeSubsystem.setIntakeSpeed(-0.1);
    }

    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished(){
        return false;
    }
    
}