// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;


public class IntakePiece extends Command {
  private final IntakeSubsystem m_intake;
  private final ElevatorSubsystem m_elevator;

  public IntakePiece(
      IntakeSubsystem intakeSubsystem,
      ElevatorSubsystem elevatorSubsystem) {
    m_intake = intakeSubsystem;
    m_elevator = elevatorSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem, elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_elevator.setHeight(Constants.Elevator.stowHeight);
    m_intake.setAngle(Constants.Intake.intakeDeployAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
      m_intake.setRollerVelocity(Constants.Intake.intakeRollerVelocity);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setAngle(Constants.Intake.intakeStowAngle);
    m_intake.setRollerVelocity(0);
    // m_launcher.moveFeedAndRedirectToPositionOffset(Constants.Launcher.rollerBeamBreakOffset);
  }

  // Returns true when the command should end.
  // @Override
  // public boolean isFinished() {
  //   return m_intake.hasPiece();
  // }
}
