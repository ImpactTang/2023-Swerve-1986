package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.commands.intake.IntakeForwardCmd;
import frc.robot.commands.intake.IntakeHoldCmd;
import frc.robot.commands.routines.StowCmd;
import frc.robot.commands.routines.scoring.ScoreHighCmd;
import frc.robot.commands.routines.scoring.ScoreLowCmd;
import frc.robot.commands.routines.scoring.ScoreMidCmd;
import frc.robot.commands.swerve.SwerveJoystickCmd;
import frc.robot.subsystems.ArmExtensionSubsystem;
import frc.robot.subsystems.ArmRotationSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.utils.Constants.IOConstants.ButtonBoxButtons;


public class RobotContainer {

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final ArmRotationSubsystem armRotationSubsystem = new ArmRotationSubsystem();
  private final ArmExtensionSubsystem armExtensionSubsystem = new ArmExtensionSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final WristSubsystem wristSubsystem = new WristSubsystem();

  private final CommandXboxController cmdDriveController = new CommandXboxController(0);

  /* NOTE: BUTTON BOX BUTTONS START AT 1!!! */
  private final CommandJoystick buttonBox = new CommandJoystick(1);  

  public RobotContainer() {

    // Xbox Controller Driving
    
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(swerveSubsystem,
    () -> -cmdDriveController.getRawAxis(0), // Axis 0 = Left X Stick
    () -> -cmdDriveController.getRawAxis(1), // Axis 1 = Left Y Stick
    () -> cmdDriveController.getRawAxis(4), // Axis 2 = Right X Stick
    () -> cmdDriveController.start().getAsBoolean()));

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    cmdDriveController.leftBumper().onTrue(new InstantCommand(() -> swerveSubsystem.resetHeading()));

    cmdDriveController.x().onTrue(new IntakeForwardCmd(intakeSubsystem));
    cmdDriveController.x().onFalse(new IntakeHoldCmd(intakeSubsystem));

    cmdDriveController.rightBumper().onTrue(new ScoreHighCmd(armRotationSubsystem, armExtensionSubsystem, wristSubsystem, intakeSubsystem));
    cmdDriveController.y().onTrue(new ScoreMidCmd(armRotationSubsystem, armExtensionSubsystem, wristSubsystem, intakeSubsystem));
    cmdDriveController.b().onTrue(new ScoreLowCmd(armRotationSubsystem, armExtensionSubsystem, wristSubsystem, intakeSubsystem));
    cmdDriveController.a().onTrue(new StowCmd(armRotationSubsystem, armExtensionSubsystem, wristSubsystem, intakeSubsystem, false));
    // buttonBox.button(ButtonBoxButtons.straightUpButton).onTrue(new StowCmd(armSubsystem));

    if (buttonBox.getX() <= 0.2){
      buttonBox.button(ButtonBoxButtons.jogUpSwitch).whileTrue(new InstantCommand(() -> armExtensionSubsystem.jogUp()));
      buttonBox.button(ButtonBoxButtons.jogDownSwitch).whileTrue(new InstantCommand(() -> armExtensionSubsystem.jogDown()));
    } else if (buttonBox.getX() > 0.2 && buttonBox.getX() <= 0.4){
      buttonBox.button(ButtonBoxButtons.jogUpSwitch).whileTrue(new InstantCommand(() -> armRotationSubsystem.jogRight()));
      buttonBox.button(ButtonBoxButtons.jogDownSwitch).whileTrue(new InstantCommand(() -> armRotationSubsystem.jogLeft()));
    }
  }

  public Command getAutonomousCommand() {
    
    return null;
  }
  
}
