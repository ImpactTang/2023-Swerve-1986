package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.arm.HighCmd;
import frc.robot.commands.arm.StowCmd;
import frc.robot.commands.intake.IntakeForwardCmd;
import frc.robot.commands.intake.IntakeHoldCmd;
import frc.robot.commands.swerve.SwerveJoystickCmd;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.Constants.IOConstants.ButtonBoxButtons;


public class RobotContainer {

  public SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  public ArmSubsystem armSubsystem = new ArmSubsystem();
  private IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  public XboxController driveController = new XboxController(0);
  public CommandXboxController cmdDriveController = new CommandXboxController(0);

  /* NOTE: BUTTON BOX BUTTONS START AT 1!!! */
  public CommandJoystick buttonBox = new CommandJoystick(1);  

  private final JoystickButton robotCentric = new JoystickButton(driveController, XboxController.Button.kA.value);

  public RobotContainer() {

    // Xbox Controller Driving
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(swerveSubsystem,
    () -> -driveController.getRawAxis(0), // Axis 0 = Left X Stick
    () -> driveController.getRawAxis(1), // Axis 1 = Left Y Stick
    () -> driveController.getRawAxis(4), // Axis 2 = Right X Stick
    () -> robotCentric.getAsBoolean()));

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    cmdDriveController.leftBumper().onTrue(new InstantCommand(() -> swerveSubsystem.resetHeading()));

    cmdDriveController.x().onTrue(new IntakeForwardCmd(intakeSubsystem));
    cmdDriveController.x().onFalse(new IntakeHoldCmd(intakeSubsystem));

    cmdDriveController.b().onTrue(new StowCmd(armSubsystem));
    cmdDriveController.y().onTrue(new HighCmd(armSubsystem));
    buttonBox.button(ButtonBoxButtons.straightUpButton).onTrue(new StowCmd(armSubsystem));
  }
  
}