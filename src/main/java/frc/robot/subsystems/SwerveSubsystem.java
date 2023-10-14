/*
 * Swerve Drive Subsystem
 */

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

    /* --------------------> Initializing the Swerve Modules <-------------------- */

    private final SwerveModule frontLeft = new SwerveModule(
        FrontLeftModule.driveMotorId, FrontLeftModule.turnMotorId,
        FrontLeftModule.driveMotorReversed, FrontLeftModule.turnMotorReversed,
        FrontLeftModule.moduleWheelOffset,
        FrontLeftModule.turnCanCoderId, FrontLeftModule.absoluteEncoderOffsetRad,
        FrontLeftModule.absoluteEncoderReversed, FrontLeftModule.name);
    private final SwerveModule frontRight = new SwerveModule(
        FrontRightModule.driveMotorId, FrontRightModule.turnMotorId,
        FrontRightModule.driveMotorReversed, FrontRightModule.turnMotorReversed,
        FrontRightModule.moduleWheelOffset,
        FrontRightModule.turnCanCoderId, FrontRightModule.absoluteEncoderOffsetRad,
        FrontRightModule.absoluteEncoderReversed, FrontRightModule.name);
    private final SwerveModule backLeft = new SwerveModule(
        BackLeftModule.driveMotorId, BackLeftModule.turnMotorId,
        BackLeftModule.driveMotorReversed, BackLeftModule.turnMotorReversed,
        BackLeftModule.moduleWheelOffset,
        BackLeftModule.turnCanCoderId, BackLeftModule.absoluteEncoderOffsetRad,
        BackLeftModule.absoluteEncoderReversed, BackLeftModule.name);
    private final SwerveModule backRight = new SwerveModule(
        BackRightModule.driveMotorId, BackRightModule.turnMotorId,
        BackRightModule.driveMotorReversed, BackRightModule.turnMotorReversed,
        BackRightModule.moduleWheelOffset,
        BackRightModule.turnCanCoderId, BackRightModule.absoluteEncoderOffsetRad,
        BackRightModule.absoluteEncoderReversed, BackRightModule.name);

    private Pigeon2 gyro = new Pigeon2(50, "Canivore");

    public SwerveDriveOdometry swerveDriveOdometry;
    

    /* --------------------> Swerve Drive Constructor <-------------------- */

    public SwerveSubsystem() {

        // Config Gyro
        gyro.configFactoryDefault();
        gyro.setYaw(0);

        // Reset Encoders
        resetModuleEncoders();

        // Configure the odometry
        swerveDriveOdometry = new SwerveDriveOdometry(DriveConstants.kSwerveDriveKinematics, 
        getRotation2d(), getModulePositions());
        
    }

    /* --------------------> Periodic Updates <-------------------- */
    @Override
    public void periodic(){

        // Update the odometry to the current module positions and heading
        swerveDriveOdometry.update(getRotation2d(), getModulePositions());

        // Update SmartDashboard data for the robot heading and location
        SmartDashboard.putNumber("Robot Heading", gyro.getYaw());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());

        // Update SmartDashboard data for each module
        frontLeft.update();
        frontRight.update();
        backLeft.update();
        backRight.update();
    }

    // Get the Rotation2d of the Robot
    public Rotation2d getRotation2d(){
        return Rotation2d.fromDegrees(gyro.getYaw());
    }


    /* --------------------> Set the Swerve Module States <-------------------- */
    public void setModuleStates(SwerveModuleState[] desiredStates){

        // Makes sure movement is valid
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, 
        DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    /* --------------------> Get the Swerve Module Positions <-------------------- */
    public SwerveModulePosition[] getModulePositions() {
        return( new SwerveModulePosition[]{
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()});
    }

    // Get the current Pose2d of the robot
    public Pose2d getPose(){
        return swerveDriveOdometry.getPoseMeters();
    }


    /* --------------------> Reset the Swerve Drive Odometry <-------------------- */
    public void resetOdometry(Pose2d pose){
        swerveDriveOdometry.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    public void resetOdometry(Pose2d pose, Rotation2d rotation){
        swerveDriveOdometry.resetPosition(rotation, getModulePositions(), pose);
    }

    // Reset the gyro heading
    public void resetHeading(){
        gyro.setYaw(0);
    }
    
    // Reset the module encoders
    public void resetModuleEncoders(){
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();
    }

    // Stop the swerve drive
    public void stopSwerve(){
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }
}