package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Shoot;
import frc.robot.commands.DriveTrain.DriveStraight;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class OneBallAuto extends SequentialCommandGroup {

  public OneBallAuto(Intake intake, Shooter shooter, DriveTrain driveTrain) {

    addCommands(
        new DriveStraight(driveTrain, () -> -24).withTimeout(5),
        new Shoot(intake, shooter, -2550, 1000).withTimeout(5),
        new DriveStraight(driveTrain, () -> -100).withTimeout(5));
  }
}
