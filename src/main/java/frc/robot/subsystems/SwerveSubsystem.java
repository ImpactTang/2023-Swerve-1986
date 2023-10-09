package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.DriveConstants;
import frc.robot.utils.Constants.ModuleConstants.*;

public class SwerveSubsystem extends SubsystemBase {

    private final SwerveModule frontLeft = new SwerveModule(
        FrontLeftModule.driveMotorId, FrontLeftModule.turnMotorId,
        FrontLeftModule.driveMotorReversed, FrontLeftModule.turnMotorReversed,
        FrontLeftModule.turnCanCoderId, FrontLeftModule.absoluteEncoderOffsetRad,
        FrontLeftModule.absoluteEncoderReversed, FrontLeftModule.name);
    private final SwerveModule frontRight = new SwerveModule(
        FrontRightModule.driveMotorId, FrontRightModule.turnMotorId,
        FrontRightModule.driveMotorReversed, FrontRightModule.turnMotorReversed,
        FrontRightModule.turnCanCoderId, FrontRightModule.absoluteEncoderOffsetRad,
        FrontRightModule.absoluteEncoderReversed, FrontRightModule.name);
    private final SwerveModule backLeft = new SwerveModule(
        BackLeftModule.driveMotorId, BackLeftModule.turnMotorId,
        BackLeftModule.driveMotorReversed, BackLeftModule.turnMotorReversed,
        BackLeftModule.turnCanCoderId, BackLeftModule.absoluteEncoderOffsetRad,
        BackLeftModule.absoluteEncoderReversed, BackLeftModule.name);
    private final SwerveModule backRight = new SwerveModule(
        BackRightModule.driveMotorId, BackRightModule.turnMotorId,
        BackRightModule.driveMotorReversed, BackRightModule.turnMotorReversed,
        BackRightModule.turnCanCoderId, BackRightModule.absoluteEncoderOffsetRad,
        BackRightModule.absoluteEncoderReversed, BackRightModule.name);

    private Pigeon2 gyro = new Pigeon2(50, "Canivore");

    public SwerveDriveOdometry swerveDriveOdometry;
    
    public SwerveSubsystem() {

        gyro.configFactoryDefault();
        gyro.setYaw(0);

        resetModuleEncoders();

        swerveDriveOdometry = new SwerveDriveOdometry(DriveConstants.kSwerveDriveKinematics, 
        getRotation2d(), getModulePositions());
        
    }

    @Override
    public void periodic(){
        swerveDriveOdometry.update(getRotation2d(), getModulePositions());

        SmartDashboard.putNumber("Robot Heading", gyro.getYaw());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());

        // Update SmartDashboard data for each module
        frontLeft.update();
        frontRight.update();
        backLeft.update();
        backRight.update();
    }

    public Rotation2d getRotation2d(){
        return Rotation2d.fromDegrees(gyro.getYaw());
    }

    public void setModuleStates(SwerveModuleState[] desiredStates){

        // Makes sure movement is valid
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, 
        DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public SwerveModulePosition[] getModulePositions() {
        return( new SwerveModulePosition[]{
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()});
    }

    public Pose2d getPose(){
        return swerveDriveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose){
        swerveDriveOdometry.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    public void resetOdometry(Pose2d pose, Rotation2d rotation){
        swerveDriveOdometry.resetPosition(rotation, getModulePositions(), pose);
    }

    public void resetHeading(){
        gyro.setYaw(0);
    }
    
    public void resetModuleEncoders(){
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();
    }

    public void stopSwerve(){
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }
}