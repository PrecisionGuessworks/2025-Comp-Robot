package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Elevator;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ClimbZero extends Command {

  private final ClimberSubsystem m_climber;
  private final ElevatorSubsystem m_elevator;
  private final ArmSubsystem m_arm;

  public ClimbZero(ClimberSubsystem climber, ElevatorSubsystem elevator, ArmSubsystem arm) {
    m_climber = climber;
    m_elevator = elevator;
    m_arm = arm;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber, elevator, arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
 
    m_elevator.setHeight(Constants.Elevator.stowHeight);
    m_arm.setArmAngle(Constants.Arm.armStowAngle);
    m_arm.setWristAngle(Constants.Arm.wristStowAngle);
    m_climber.setZero();
    m_climber.setHeight(Constants.Climber.stowHeight);
    // m_intake.setRollerVelocity(-1.0);
    // m_intake.setAngle(Constants.Intake.intakeClimbAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //m_climber.setHeight(Constants.Climber.L2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //m_climber.setHeight(Constants.Climber.stowHeight);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
