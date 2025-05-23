package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;


public class Moveup extends Command {

  private final ElevatorSubsystem m_elevator;
  private Timer m_ejectTimer = new Timer();

  public Moveup(ElevatorSubsystem elevator) {

    m_elevator = elevator;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //m_elevator.setHeight(Constants.Elevator.L2);
    if(m_elevator.getHeightLocation() == 1){
      m_elevator.setHeight(Constants.Elevator.L1);
    } else if(m_elevator.getHeightLocation() == 2){
      m_elevator.setHeight(Constants.Elevator.L2);
    } else if(m_elevator.getHeightLocation() == 3){
      m_elevator.setHeight(Constants.Elevator.L3);
    } else if(m_elevator.getHeightLocation() == 4){
      m_elevator.setHeight(Constants.Elevator.L4);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.setHeight(Constants.Elevator.stowHeight);

  }

  // Returns true when the command should end.
  // @Override
  // public boolean isFinished() {
  //   return m_ejectTimer.get() > 0.35;
  // }
}
