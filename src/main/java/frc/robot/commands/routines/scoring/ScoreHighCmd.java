package frc.robot.commands.routines.scoring;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ExtendCmd;
import frc.robot.commands.arm.RotateCmd;
import frc.robot.commands.intake.IntakeForwardCmd;
import frc.robot.subsystems.ArmExtensionSubsystem;
import frc.robot.subsystems.ArmRotationSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ScoreHighCmd extends SequentialCommandGroup{

    public ScoreHighCmd(ArmRotationSubsystem armRotationSubsystem, ArmExtensionSubsystem armExtensionSubsystem, IntakeSubsystem intakeSubsystem){

        addCommands(
            new ParallelCommandGroup(
                new ExtendCmd(armExtensionSubsystem, Units.inchesToMeters(35.0)),
                new RotateCmd(armRotationSubsystem, Units.degreesToRadians(55.0))));
        addCommands(
            new IntakeForwardCmd(intakeSubsystem));

    }
    
}
