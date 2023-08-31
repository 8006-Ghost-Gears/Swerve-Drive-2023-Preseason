// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDrive;

public class SetSwerveCoastMode extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveDrive m_swerveDrive;

  public SetSwerveCoastMode(SwerveDrive swerveDrive) {
    m_swerveDrive = swerveDrive;
    addRequirements(swerveDrive);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_swerveDrive.setNeutralMode(IdleMode.kCoast);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}