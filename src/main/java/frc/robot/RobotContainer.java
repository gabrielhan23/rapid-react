package frc.robot;

// import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Climber.AutoClimb;
import frc.robot.commands.Climber.ClimbDown;
import frc.robot.commands.Climber.ClimbUp;
import frc.robot.commands.DriveTrain.ArcadeDrive;
import frc.robot.commands.DriveTrain.DriveStraight;
import frc.robot.commands.Calibration;
import frc.robot.commands.IntakeBalls;
import frc.robot.commands.Shoot;
// import frc.robot.commands.Climber.PivotRelative;

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
  // private final LogitechGamingPad opPad = new LogitechGamingPad(1);

  private final Shooter shooter = new Shooter();
  private final Climber climber = new Climber();
  private final Intake intake = new Intake();
  private final NavX navX = new NavX();
  private final DriveTrain driveTrain = new DriveTrain(navX);
  private final Pivots pivots = new Pivots();

  private final JoystickButton driveA = new JoystickButton(drivePad, 1);
  private final JoystickButton driveB = new JoystickButton(drivePad, 2);
  private final JoystickButton driveX = new JoystickButton(drivePad, 3);
  private final JoystickButton driveY = new JoystickButton(drivePad, 4);
  private final JoystickButton leftBumper = new JoystickButton(drivePad, 5);
  private final JoystickButton rightBumper = new JoystickButton(drivePad, 6);
  // private final JoystickButton driveBackButton = new
  // JoystickButton(drivePad,7);
  private final JoystickButton driveStartButton = new JoystickButton(drivePad, 8);

  // private final JoystickButton opA = new JoystickButton(opPad, 1);
  // private final JoystickButton opB = new JoystickButton(opPad, 2);
  // private final JoystickButton opX = new JoystickButton(opPad, 3);
  // private final JoystickButton opY = new JoystickButton(opPad, 4);
  // private final JoystickButton opLeftBumper = new JoystickButton(opPad, 5);
  // private final JoystickButton opRightBumper = new JoystickButton(opPad, 6);
  // private final JoystickButton opBackButton = new JoystickButton(opPad, 7);
  // private final JoystickButton opStartButton = new JoystickButton(opPad, 8);

  private final ShuffleBoard shuffleBoard = new ShuffleBoard(
      intake, shooter, driveTrain, navX);

  // should be temp climb
  // private int selectCounter = 0;

  // private int select() {
  // return selectCounter;
  // }

  // public void increaseSelect(){
  // System.out.println("WE ARE ON STEP: "+selectCounter);
  // if (selectCounter == 10) {
  // selectCounter = 0;
  // } else {
  // selectCounter++;
  // }
  // }

  // private final Command autoClimb = new SelectCommand(
  // Map.ofEntries(
  // Map.entry(0, new PivotRelative(pivots, -230).beforeStarting(new
  // InstantCommand(()->increaseSelect())).withTimeout(2)),
  // Map.entry(1, new ClimbDown(climber, 1000,
  // 1000).withTimeout(2).beforeStarting(new
  // InstantCommand(()->increaseSelect())).andThen(new PivotRelative(pivots,
  // 20).withTimeout(3))),
  // Map.entry(2, new ClimbUp(climber, 40000, 40000).beforeStarting(new
  // InstantCommand(()->increaseSelect())).withTimeout(2).andThen(new
  // PivotRelative(pivots, -10).withTimeout(3))),
  // Map.entry(3, new PivotRelative(pivots, 90).beforeStarting(new
  // InstantCommand(()->increaseSelect())).withTimeout(2)),
  // Map.entry(4, new ClimbUp(climber).beforeStarting(new
  // InstantCommand(()->increaseSelect())).withTimeout(3)),
  // Map.entry(5, new PivotRelative(pivots, -12).beforeStarting(new
  // InstantCommand(()->increaseSelect())).withTimeout(1)),
  // Map.entry(6, new ClimbDown(climber, 220000, 220000).beforeStarting(new
  // InstantCommand(()->increaseSelect())).withTimeout(2)),
  // Map.entry(7, new PivotRelative(pivots, -100).beforeStarting(new
  // InstantCommand(()->increaseSelect())).withTimeout(2)),
  // Map.entry(8, new ClimbDown(climber, 150000, 150000).beforeStarting(new
  // InstantCommand(()->increaseSelect())).withTimeout(2)),
  // Map.entry(9, new ClimbDown(climber, 175000, 175000).beforeStarting(new
  // InstantCommand(()->increaseSelect())).withTimeout(2)),
  // Map.entry(10, new InstantCommand(()-> pivots.setPositionPivots(0,
  // 0)).beforeStarting(new InstantCommand(()->increaseSelect())).withTimeout(2))
  // ),
  // this::select);
  // end of should be temp climb

  public RobotContainer() {
    driveTrain.setDefaultCommand(new ArcadeDrive(driveTrain, drivePad));
  }

  public void configureButtonBindings() {
    // selectCounter = 0;
    driveTrain.setSlowMode(false);
    rightBumper.whileHeld(new IntakeBalls(intake));
    // leftBumper.whileHeld(new CargoManipulation(intake, shooter, false, -1));
    leftBumper.whileHeld(new DriveStraight(driveTrain, () -> shuffleBoard.getMoveBack())
        .alongWith(new Shoot(intake, shooter, () -> shuffleBoard.getShooterVelocity(),
            () -> shuffleBoard.getRollerVelocity())));
    driveStartButton.whenPressed(new Calibration(climber, pivots, intake, driveTrain));

    driveX.whenPressed(new AutoClimb(climber, pivots));
    // driveX.whenPressed(autoClimb);
    driveY.whileHeld(new ClimbUp(climber, 1));
    driveY.whenPressed(new InstantCommand(driveTrain::climbingTrue));
    driveA.whileHeld(new ClimbDown(climber));
    driveB.whenPressed(new InstantCommand(driveTrain::toggleSlowMode));
  }

  public Command getAutonomousCommand() {
    return shuffleBoard.getAutonomousCommand();
  }

  public Command calibrate() {
    return new Calibration(climber, pivots, intake, driveTrain);
  }
}
