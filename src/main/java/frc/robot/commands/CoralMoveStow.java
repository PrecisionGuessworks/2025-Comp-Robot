// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem;


public class CoralMoveStow extends Command {
  private final IntakeSubsystem m_intake;
  private final ElevatorSubsystem m_elevator;
  private final ArmSubsystem m_arm;
  public int pastscoreheight = 0;
  public boolean trueendtrigger = false;
  private Timer m_placeTimer = new Timer();

  public CoralMoveStow(
      IntakeSubsystem intakeSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      ArmSubsystem armSubsystem) {
    m_intake = intakeSubsystem;
    m_elevator = elevatorSubsystem;
    m_arm = armSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem, elevatorSubsystem, armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_placeTimer.restart();
    trueendtrigger = false;
    m_arm.setArmRollerCurrent(30, 30);
    m_arm.setRollerVelocity(100);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

      if (0.5 <= m_placeTimer.get()){
        trueendtrigger = true;
      }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  if(m_elevator.isAtScore()){
    m_arm.setArmAngle(Constants.Arm.armStowAngle);
   m_arm.setWristAngle(Constants.Arm.wristStowAngle);
    m_arm.setRollerVelocity(0);
    m_elevator.setHeight(Constants.Elevator.stowHeight);
  }
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return trueendtrigger && m_elevator.isAtScore();
  }
}
