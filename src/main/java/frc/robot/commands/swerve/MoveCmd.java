package frc.robot.commands.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.Constants.DriveConstants;

public class MoveCmd extends CommandBase{

    private final SwerveSubsystem swerveSubsystem;
    private final double xSpeed, ySpeed, turningSpeed;


    public MoveCmd(SwerveSubsystem swerveSubsystem, double xSpeed, double ySpeed, double turningSpeed){
        this.swerveSubsystem = swerveSubsystem;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.turningSpeed = turningSpeed;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        ChassisSpeeds chassisSpeeds;
        chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);

        SwerveModuleState[] moduleStates = DriveConstants.kSwerveDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted){
        swerveSubsystem.stopSwerve();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
    
}