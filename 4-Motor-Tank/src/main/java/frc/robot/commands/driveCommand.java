// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;


public class driveCommand extends CommandBase {
  /** Creates a new driveCommand. */
  public driveCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.mDriveSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

double leftSide = 0;
double rightSide = 0;

leftSide -= RobotContainer.driver.getLeftY();
rightSide -= RobotContainer.driver.getRightY();

RobotContainer.mDriveSub.setPower(Math.signum(leftSide)*Math.pow(leftSide, 2), Math.signum(rightSide)*Math.pow(rightSide, 2));

  }  
}