// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem;


public class CoralMoveScore extends Command {
  private final IntakeSubsystem m_intake;
  private final ElevatorSubsystem m_elevator;
  private final ArmSubsystem m_arm;
  public int pastscoreheight = 0;
  public CoralMoveScore(
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
    pastscoreheight = 0;
   m_arm.setArmAngle(Constants.Arm.armScoreAngle);
   m_arm.setWristAngle(Constants.Arm.wristScoreAngle);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(pastscoreheight != m_elevator.getHeightLocation()){
      if(m_elevator.getHeightLocation() == 1){
        pastscoreheight = 1;
        m_elevator.setHeight(Constants.Elevator.L1);
      } else if(m_elevator.getHeightLocation() == 2){
        pastscoreheight = 2;
        m_elevator.setHeight(Constants.Elevator.L2);
      } else if(m_elevator.getHeightLocation() == 3){
        pastscoreheight = 3;
        m_elevator.setHeight(Constants.Elevator.L3);
      } else if(m_elevator.getHeightLocation() == 4){
        pastscoreheight = 4;
        m_elevator.setHeight(Constants.Elevator.L4);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  m_arm.setArmAngle(Constants.Arm.armStowAngle);
   m_arm.setWristAngle(Constants.Arm.wristStowAngle);
    m_arm.setRollerVelocity(0);
    m_elevator.setHeight(Constants.Elevator.stowHeight);
    // m_launcher.moveFeedAndRedirectToPositionOffset(Constants.Launcher.rollerBeamBreakOffset);
  }

  // Returns true when the command should end.
  // @Override
  // public boolean isFinished() {
  //   return m_intake.hasPiece();
  // }
}
