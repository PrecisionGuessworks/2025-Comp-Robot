// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.quixlib.motorcontrol.QuixTalonFX;
import frc.quixlib.viz.Link2d;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class ElevatorSubsystem extends SubsystemBase {
  private final QuixTalonFX m_motor =
      new QuixTalonFX(
          Constants.Elevator.motorID,
          Constants.Elevator.motorRatio,
          QuixTalonFX.makeDefaultConfig()
              .setBrakeMode()
              .setSupplyCurrentLimit(40.0)
              .setStatorCurrentLimit(90.0)
              .setInverted(Constants.Elevator.motorInvert)
              .setPIDConfig(Constants.Elevator.motorPositionSlot, Constants.Elevator.motorPIDConfig)
              .setMotionMagicConfig(
                  Constants.Elevator.maxVelocity,
                  Constants.Elevator.maxAcceleration,
                  Constants.Elevator.maxJerk,
                  Constants.Elevator.Expo_kV,
                  Constants.Elevator.Expo_kA)
              .setReverseSoftLimit(Constants.Elevator.minHeight)
              .setForwardSoftLimit(Constants.Elevator.maxHeight));

  private final QuixTalonFX m_follower = new QuixTalonFX(
      Constants.Elevator.followerID,
      m_motor,
      Constants.Elevator.followerInvert,
      QuixTalonFX.makeDefaultConfig().setBrakeMode()
      .setSupplyCurrentLimit(40.0)
      .setStatorCurrentLimit(90.0)
      .setInverted(Constants.Elevator.motorInvert)
      .setPIDConfig(Constants.Elevator.motorPositionSlot, Constants.Elevator.motorPIDConfig)
      .setMotionMagicConfig(
          Constants.Elevator.maxVelocity,
          Constants.Elevator.maxAcceleration,
          Constants.Elevator.maxJerk)
      .setReverseSoftLimit(Constants.Elevator.minHeight)
      .setForwardSoftLimit(Constants.Elevator.maxHeight));

  private double m_setTargetHeight = Constants.Elevator.minHeight;
  private double m_targetHeight = Constants.Elevator.minHeight;
  public int m_HeightLocation = 4;
  private boolean Loc1 = false;
  private boolean Loc2 = false;
  private boolean Loc3 = false;
  private boolean Loc4 = false;
  private boolean m_ElevatorOff = Constants.ElevatorOff;
  private boolean m_ElevatorOffLast = m_ElevatorOff;

  public ElevatorSubsystem(Link2d elevatorCarriageViz) {
    // Show scheduler status in SmartDashboard.
    SmartDashboard.putData(this);

    // Setup viz.
    m_elevatorCarriageViz = elevatorCarriageViz;
  }

  public boolean isAtScore(){
    if (m_HeightLocation == 4){
      return isAtHeight(Constants.Elevator.L4, Units.inchesToMeters(3));
    } else if (m_HeightLocation == 3){
      return isAtHeight(Constants.Elevator.L3, Units.inchesToMeters(3));
    } else if (m_HeightLocation == 2){
      return isAtHeight(Constants.Elevator.L2, Units.inchesToMeters(3));
    } else if (m_HeightLocation == 1){
      return isAtHeight(Constants.Elevator.L1, Units.inchesToMeters(3));
    } else {
      return false;
    }
  }

  public double getHeight() {
    return Constants.Elevator.motorRatio.sensorRadiansToMechanismPosition(m_motor.getSensorPosition());
  }

  public void setHeight(double targetHeight) {
    m_setTargetHeight = targetHeight;
  }
  public void setHeightLocation(int targetHeight) {
    m_HeightLocation = targetHeight;
  }
  public int getHeightLocation() {
    return m_HeightLocation;
  }

  public boolean isAtHeight(double height, double tolerance) {
    return Math.abs(height - getHeight()) <= tolerance;
  }
  private double armAngle = 0;
  private double wristAngle = 0;

  public void setElevatorOn(boolean lineup){
    m_ElevatorOff = lineup;
}
public boolean getElevatorOn(){
    return m_ElevatorOff;
}

  @Override
  public void periodic() {
    armAngle = RobotContainer.arm.getArmAngle();
    wristAngle = RobotContainer.arm.getWristAngle();
    if (armAngle <= 50 && m_setTargetHeight <= Constants.Elevator.armStowHeight){ 
      m_targetHeight = Constants.Elevator.armStowHeight;
    } else if (wristAngle < 60 && m_setTargetHeight < Constants.Elevator.wristStowHeight && getHeight() >= Constants.Elevator.armStowHeight){ 
      m_targetHeight = Constants.Elevator.wristStowHeight; 
    } else if ((armAngle < 90 && wristAngle < 90 && m_setTargetHeight > Constants.Elevator.armStowHeight)||armAngle < 70 && m_setTargetHeight > Constants.Elevator.armStowHeight){
      m_targetHeight = m_setTargetHeight;
    }else if (armAngle < 115 && armAngle > 91){ // intake
      m_targetHeight = Constants.Elevator.stowHeight;
    } else if (armAngle > 100 && m_setTargetHeight <= Constants.Elevator.intakeHeight && wristAngle > 85){
      m_targetHeight = m_setTargetHeight;
    } else {
      m_targetHeight = Constants.Elevator.stowHeight;
    }

    //m_targetHeight = m_setTargetHeight;

    // This method will be called once per scheduler run

    if (m_ElevatorOff != m_ElevatorOffLast){
      m_ElevatorOffLast = m_ElevatorOff;
      if (m_ElevatorOff){
        m_motor.setStatorCurrentLimit(1,1);
        m_follower.setStatorCurrentLimit(1,1);
      } else {
        m_motor.setStatorCurrentLimit(70,30);
        m_follower.setStatorCurrentLimit(70,30);
      }
    }
    
    m_motor.setMotionMagicPositionSetpointExpo(
        Constants.Elevator.motorPositionSlot,
        m_targetHeight
        );

    if (m_HeightLocation==1){
      Loc1 = true;
      Loc2 = false;
      Loc3 = false;
      Loc4 = false;
    } else if (m_HeightLocation==2){
      Loc1 = false;
      Loc2 = true;
      Loc3 = false;
      Loc4 = false;
    } else if (m_HeightLocation==3){
      Loc1 = false;
      Loc2 = false;
      Loc3 = true;
      Loc4 = false;
    } else if (m_HeightLocation==4){
      Loc1 = false;
      Loc2 = false;
      Loc3 = false;
      Loc4 = true;
    }
    
    SmartDashboard.putBoolean(
          "Elevator", !m_ElevatorOff);
          
          if(Constants.ExtraInfo){
    SmartDashboard.putNumber(
        "Elevator: Current Height (in)", Units.metersToInches(getHeight()));
    SmartDashboard.putNumber(
        "Elevator: Target Height (in)", Units.metersToInches(Constants.Elevator.motorRatio.sensorRadiansToMechanismPosition(m_motor.getClosedLoopReference())));
        SmartDashboard.putNumber(
          "Elevator: Target set Height (in)",
          Units.metersToInches(m_targetHeight));      
      }
    SmartDashboard.putBoolean(
          "L1", Loc1);
    SmartDashboard.putBoolean(
          "L2", Loc2);
    SmartDashboard.putBoolean(  
          "L3", Loc3);    
    SmartDashboard.putBoolean(
          "L4", Loc4);
          if(Constants.ExtraInfo){
    m_motor.logMotorState();
    m_follower.logMotorState();
          }
  }

  // --- BEGIN STUFF FOR SIMULATION ---
  private static final ElevatorSim m_elevatorSim =
      new ElevatorSim(
          DCMotor.getKrakenX60Foc(2),
          Constants.Elevator.motorRatio.reduction(),
          Constants.Elevator.simCarriageMass,
          Constants.Elevator.sprocketPitchDiameter * 0.5,
          Constants.Elevator.minHeight,
          Constants.Elevator.maxHeight,
          true,
          0);

  // Visualization
  private final Link2d m_elevatorCarriageViz;

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    m_elevatorSim.setInput(m_motor.getPercentOutput() * RobotController.getBatteryVoltage());
    m_elevatorSim.update(TimedRobot.kDefaultPeriod);
    m_motor.setSimSensorPositionAndVelocity(
        m_elevatorSim.getPositionMeters(),
        // m_elevatorSim.getVelocityMetersPerSecond(), // TODO: Figure out why this causes jitter
        0.0,
        TimedRobot.kDefaultPeriod,
        Constants.Elevator.motorRatio);

    // Update carriage viz.
    m_elevatorCarriageViz.setRelativeTransform(
        new Transform2d(m_elevatorSim.getPositionMeters(), 0.0, new Rotation2d()));
  }
  // --- END STUFF FOR SIMULATION ---
}
