// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.Constants;
import frc.robot.subsystems.Climber;
public class ClimbUp extends CommandBase {
  private final Climber climber;
  private double encoderLeft;
  private double encoderRight;
  private int pidProfile;

  public ClimbUp(Climber climber, int pidProfile) {
    addRequirements(climber);
    this.climber = climber;
    this.pidProfile = pidProfile;
    encoderLeft = Constants.Climber.UP_ENCODER_LEFT;
    encoderRight = Constants.Climber.UP_ENCODER_RIGHT;
  }

  public ClimbUp(Climber climber, double encoderLeft, double encoderRight, int pidProfile) {
    addRequirements(climber);
    this.climber = climber;
    this.pidProfile = pidProfile;
    this.encoderLeft = encoderLeft;
    this.encoderRight = encoderRight;
  }

  @Override
  public void initialize() {
    // climber.climb(Constants.Climber.UP_SPEED);
    climber.selectProfile(pidProfile);
    climber.setPositionLeft(encoderLeft);
    climber.setPositionRight(encoderRight);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    climber.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
