package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeMoveCmd extends CommandBase{

    private final IntakeSubsystem intakeSubsystem;
    private double intakeSpeed;

    public IntakeMoveCmd(IntakeSubsystem intakeSubsystem, double intakeSpeed){
        this.intakeSubsystem = intakeSubsystem;
        this.intakeSpeed = intakeSpeed;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        intakeSubsystem.setIntakeSpeed(intakeSpeed);
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