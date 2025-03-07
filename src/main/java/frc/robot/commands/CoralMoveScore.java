// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.Telemetry;


public class CoralMoveScore extends Command {
  private final ElevatorSubsystem m_elevator;
  private final ArmSubsystem m_arm;
  //private Pose2d m_pose;
  public int pastscoreheight = 4;
  public CoralMoveScore(
      ElevatorSubsystem elevatorSubsystem,
      ArmSubsystem armSubsystem) {
    
    //m_pose = currentPose;
    m_elevator = elevatorSubsystem;
    m_arm = armSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevatorSubsystem, armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pastscoreheight = 0;
    
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.setArmRollerCurrent(20, 10); 
    m_arm.setRollerVelocity(-20);
    
    Pose2d m_pose = RobotContainer.drivetrain.getState().Pose;
    Pose2d targetpose = new Pose2d(4.5,4,new Rotation2d(0));
    //    && m_arm.getWristAngle() < 91 && m_arm.getArmAngle() < 91
    // 4 >= PhotonUtils.getDistanceToPose(m_pose,targetpose)
   if (2.5 >= PhotonUtils.getDistanceToPose(m_pose,targetpose)||Constants.PoseSoring){
      //System.out.println("CoralMoveScore");
      if(pastscoreheight != m_elevator.getHeightLocation()){
        
        
        //System.out.println("part");
        if(m_elevator.getHeightLocation() == 1){
          pastscoreheight = 1;
          m_elevator.setHeight(Constants.Elevator.L1);
          m_arm.setWristAngle(Constants.Arm.wristL1Score);
          m_arm.setArmAngle(Constants.Arm.armL1Score);
        } else if(m_elevator.getHeightLocation() == 2){
          pastscoreheight = 2;
          m_elevator.setHeight(Constants.Elevator.L2);
          m_arm.setWristAngle(Constants.Arm.wristScoreAngle);
          m_arm.setArmAngle(Constants.Arm.armScoreAngle);
        } else if(m_elevator.getHeightLocation() == 3){
          pastscoreheight = 3;
          m_elevator.setHeight(Constants.Elevator.L3);
          m_arm.setWristAngle(Constants.Arm.wristScoreAngle);
          m_arm.setArmAngle(Constants.Arm.armScoreAngle);
        } else if(m_elevator.getHeightLocation() == 4){
          pastscoreheight = 4;
          m_elevator.setHeight(Constants.Elevator.L4);
          m_arm.setWristAngle(Constants.Arm.wristL4Score);
          m_arm.setArmAngle(Constants.Arm.armScoreAngle);
        }
      }
    } else {
      m_elevator.setHeight(Constants.Elevator.stowHeight);
      m_arm.setArmAngle(Constants.Arm.armStowAngle);
      m_arm.setWristAngle(Constants.Arm.wristStowAngle);

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  //m_arm.setArmAngle(Constants.Arm.armStowAngle);
  // m_arm.setWristAngle(Constants.Arm.wristStowAngle);
    
   // m_elevator.setHeight(Constants.Elevator.stowHeight);
  }

  // Returns true when the command should end.
  // @Override
  // public boolean isFinished() {
  //   return m_intake.hasPiece();
  // }
}
