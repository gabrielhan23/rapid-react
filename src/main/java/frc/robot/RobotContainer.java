package frc.robot;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// import frc.robot.commands.Climber.AutoClimb;
import frc.robot.commands.Climber.Calibration;
import frc.robot.commands.Climber.ClimbDown;
import frc.robot.commands.Climber.ClimbUp;
import frc.robot.commands.DriveTrain.ArcadeDrive;
import frc.robot.commands.DriveTrain.DriveStraight;
import frc.robot.commands.IntakeBalls;
import frc.robot.commands.Shoot;
import frc.robot.commands.Climber.PivotRelative;

import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.Pivots;
import frc.robot.subsystems.Shooter;

import frc.robot.utils.LogitechGamingPad;
import frc.robot.utils.ShuffleBoard;

public class RobotContainer {
  private final LogitechGamingPad drivePad = new LogitechGamingPad(0);

  private final DriveTrain driveTrain = new DriveTrain();
  private final Shooter shooter = new Shooter();
  private final Climber climber = new Climber();
  private final Intake intake = new Intake();
  private final NavX navX = new NavX();
  private final Pivots pivots = new Pivots();

  private final JoystickButton driveA = new JoystickButton(drivePad, 1);
  private final JoystickButton driveB = new JoystickButton(drivePad, 2);
  private final JoystickButton driveX = new JoystickButton(drivePad, 3);
  private final JoystickButton driveY = new JoystickButton(drivePad, 4);
  private final JoystickButton leftBumper = new JoystickButton(drivePad, 5);
  private final JoystickButton rightBumper = new JoystickButton(drivePad, 6);
  // private final JoystickButton driveBackButton = new JoystickButton(drivePad,
  // 7);
  // private final JoystickButton driveStartButton = new JoystickButton(drivePad,
  // 8);

  private final ShuffleBoard shuffleBoard = new ShuffleBoard(
      intake, shooter, driveTrain, navX);

  // should be temp climb
  private int selectCounter = 0;

  private int select() {
    return selectCounter;
  }

  public void increaseSelect(){
    if (selectCounter == 5) {
      selectCounter = 0;
    } else {
      selectCounter++;
    }
    System.out.println("BUTTON CLICKED IS NOW AT: "+selectCounter);
  }

  private final Command autoClimb = new SelectCommand(
      Map.ofEntries(
          Map.entry(0, new PivotRelative(pivots, -230).beforeStarting(new InstantCommand(()->increaseSelect()))),
          Map.entry(1, new ClimbDown(climber, true).withTimeout(2).beforeStarting(new InstantCommand(()->increaseSelect()))),
          Map.entry(2, new PivotRelative(pivots, 20).beforeStarting(new InstantCommand(()->increaseSelect()))),
          Map.entry(3, new ClimbUp(climber, 100000, 100000).beforeStarting(new InstantCommand(()->increaseSelect())).withTimeout(2)),
          Map.entry(4, new PivotRelative(pivots, 90).beforeStarting(new InstantCommand(()->increaseSelect()))),
          // Map.entry(3, new PivotRelative(climber, 90).deadlineWith(new InstantCommand(() -> climber.climb(0.2)))),
          Map.entry(5, new ClimbUp(climber).beforeStarting(new InstantCommand(()->increaseSelect())))
          // Map.entry(5, new PivotRelative(climber, -50).deadlineWith(new ClimbDown(climber, false)))
          ),
      this::select);
  // end of should be temp climb
  
  public RobotContainer() {
    driveTrain.setDefaultCommand(new ArcadeDrive(driveTrain, drivePad));
  }

  public void configureButtonBindings() {
    selectCounter = 0;
    rightBumper.whileHeld(new IntakeBalls(intake));
    // leftBumper.whileHeld(new CargoManipulation(intake, shooter, false, -1));
    leftBumper.whileHeld(new DriveStraight(driveTrain, () -> shuffleBoard.getMoveBack())
        .alongWith(new Shoot(intake, shooter, () -> shuffleBoard.getShooterVelocity())));

    // driveX.whenPressed(new AutoClimb(climber));
    driveX.whenPressed(autoClimb);
    driveY.whileHeld(new ClimbUp(climber));
    driveA.whileHeld(new ClimbDown(climber, false));
    driveB.whenPressed(new InstantCommand(driveTrain::toggleSlowMode));
  }


  public Command getAutonomousCommand() {
    return shuffleBoard.getAutonomousCommand();
  }

  public Command calibrateClimber() {
    return new Calibration(climber, pivots);
  }
}
