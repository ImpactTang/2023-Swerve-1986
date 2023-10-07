package frc.robot.commands.routines;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.arm.ExtendArmCmd;
import frc.robot.commands.arm.RotateArmCmd;
import frc.robot.commands.intake.IntakeMoveCmd;
import frc.robot.commands.wrist.RotateWristCmd;
import frc.robot.subsystems.ArmExtensionSubsystem;
import frc.robot.subsystems.ArmRotationSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class StowCmd extends ParallelCommandGroup{

  private IntakeMoveCmd intakeMoveCmd;

  public StowCmd(ArmRotationSubsystem armRotationSubsystem, ArmExtensionSubsystem armExtensionSubsystem, WristSubsystem wristSubsystem, IntakeSubsystem intakeSubsystem, boolean hasGamePiece){
    
    if (hasGamePiece == true){
      intakeMoveCmd = new IntakeMoveCmd(intakeSubsystem, -0.1);
    } else{
      intakeMoveCmd = new IntakeMoveCmd(intakeSubsystem, 0.0);
    }

    addCommands(
      new ExtendArmCmd(armExtensionSubsystem, Units.inchesToMeters(0.0)),
      new RotateArmCmd(armRotationSubsystem, Math.PI / 2.0),
      new RotateWristCmd(wristSubsystem, 0.0),
      intakeMoveCmd);
  }
 
}
