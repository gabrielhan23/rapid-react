// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.LinearServo;

public class PistonMove extends CommandBase {
    /** Creates a new ClimbTeleop. */

    private final LinearServo servo;
    private int counter;
    private boolean calibrating;

    public PistonMove(LinearServo servo) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.servo = servo;
        counter = 0;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        servo.setPosition(0.4);
        counter = 0;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (counter > 100) {
            servo.setPosition(1);
        }
        counter++;

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        servo.setPosition(1);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (counter > 200){
            return true;
        }
        return false;
    }
}
