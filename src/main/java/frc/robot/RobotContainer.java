// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import org.json.simple.parser.ParseException;
import org.photonvision.PhotonUtils;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.quixlib.viz.Link2d;
import frc.quixlib.viz.Viz2d;
import frc.robot.commands.IntakePiece;
import frc.robot.commands.Moveup;
import frc.robot.commands.CoralMoveScore;
import frc.robot.commands.CoralMoveStow;
import frc.robot.commands.IntakeCoral;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import static frc.robot.Constants.Drive.*;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

public class RobotContainer {
    private double MaxSpeed = MaxSpeedPercentage*(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(MaxAngularRatePercentage).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    public final CommandXboxController driver = new CommandXboxController(0);
    public final CommandXboxController operator = new CommandXboxController(1);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public static final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * DriveDeadband).withRotationalDeadband(MaxAngularRate * RotationDeadband)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

            private final SwerveRequest.FieldCentricFacingAngle angle = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * DriveDeadband).withRotationalDeadband(MaxAngularRate * SnapRotationDeadband) // Add a deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors 
          //  .withSteerRequestType(SteerRequestType.MotionMagicExpo); // Use motion magic control for steer motors

    private PowerDistribution powerDistribution = new PowerDistribution();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;


Map<String, Command> robotCommands  = new HashMap<String, Command>();



private static  final Viz2d robotViz =
      new Viz2d("Robot Viz", Units.inchesToMeters(80.0), Units.inchesToMeters(120.0), 1.0);

private static  final Link2d chassisViz =
      robotViz.addLink(
          new Link2d(
              robotViz,
              "Chassis",
              Units.inchesToMeters(29.0),
              30.0,
              new Color("#FAB604"),
              new Transform2d(Constants.Viz.xOffset, Units.inchesToMeters(3.0), new Rotation2d())));

  // Elevator viz
  private  static final Link2d elevatorFrameViz =
      robotViz.addLink(
          new Link2d(
              robotViz,
              "Elevator Base",
              Constants.Viz.elevatorBaseLength,
              4.0,
              Color.kGreen,
              new Transform2d(
                  Constants.Viz.elevatorBaseX,
                  Constants.Viz.elevatorBaseY,
                  Constants.Viz.elevatorAngle)));
  private static  final Link2d elevatorCarriageViz =
      elevatorFrameViz.addLink(
          new Link2d(
              robotViz,
              "Elevator Carriage",
              Constants.Viz.elevatorCarriageLength,
              6.0,
              Color.kLightGreen));
// Intake viz
private static final Link2d intakeArmViz =
robotViz.addLink(
    new Link2d(robotViz, "Intake Arm", Constants.Viz.intakeArmLength, 10.0, Color.kBlue));
private static final Link2d intakeRollerViz =
intakeArmViz.addLink(
    new Link2d(robotViz, "Intake Roller", Units.inchesToMeters(1.0), 10.0, Color.kLightBlue));


private static final Link2d ArmArmViz =
elevatorCarriageViz.addLink(
        new Link2d(robotViz, "Arm Arm", Constants.Viz.ArmArmLength, 10, Color.kRed));
private static final Link2d ArmWristViz =
ArmArmViz.addLink(
        new Link2d(robotViz, "Arm Wrist", Constants.Viz.ArmWristLength, 10, Color.kOrange));
private static final Link2d ArmWheelViz =
ArmWristViz.addLink(
        new Link2d(robotViz, "Arm Wheel", Units.inchesToMeters(2.0), 10, Color.kCoral));

    

        public static final ElevatorSubsystem elevator = new ElevatorSubsystem(elevatorCarriageViz);
        public static final IntakeSubsystem intake = new IntakeSubsystem(intakeArmViz, intakeRollerViz);
        public static final ArmSubsystem arm =
      new ArmSubsystem(
          ArmArmViz,
          ArmWristViz,
          ArmWheelViz
          );








    public RobotContainer() {

        robotCommands.put("IntakePiece", new IntakePiece(intake, elevator).withTimeout(2.5));
        robotCommands.put("CoralMoveScore", new CoralMoveScore(intake, elevator, arm).withTimeout(4));
        robotCommands.put("CoralMoveStow", new CoralMoveStow(intake, elevator, arm));
        robotCommands.put("IntakeCoral", new IntakeCoral(intake, elevator, arm));
    
        NamedCommands.registerCommands(robotCommands);




        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto", autoChooser);
        angle.HeadingController.setPID( PRotation,  IRotation , DRotation);
        configureBindings();

    


        SmartDashboard.putData(
        "Gyro",
        builder -> {
          builder.setSmartDashboardType("Gyro");
          builder.addDoubleProperty("Value", () -> drivetrain.getPigeon2().getYaw().getValueAsDouble(), null);
        });
          SmartDashboard.putNumber("Time",Timer.getMatchTime());
          SmartDashboard.putNumber("Time2",DriverStation.getMatchTime());
          SmartDashboard.putNumber("Voltage",RobotController.getBatteryVoltage());
          SmartDashboard.putNumber("CAN",RobotController.getCANStatus().percentBusUtilization * 100.0);
          SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
        SmartDashboard.putData("Power Distribution Panel", powerDistribution);

        PathfindingCommand.warmupCommand().ignoringDisable(true).schedule();;
        
    }

    

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // driver.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
        // ));

        driver.leftBumper().onTrue(new ParallelCommandGroup(new CoralMoveScore(intake, elevator, arm), pathfindingCommand(true)));
        driver.leftBumper().onFalse(new CoralMoveStow(intake, elevator, arm));
        driver.rightBumper().onTrue(new ParallelCommandGroup(new CoralMoveScore(intake, elevator, arm), pathfindingCommand(false)));
        driver.rightBumper().onFalse(new CoralMoveStow(intake, elevator, arm));

        //driver.y().whileTrue(pathfindingCommand(false));
        driver.x().whileTrue(pathfindingtofollowCommand());
        driver.b().whileTrue(new IntakeCoral(intake, elevator, arm));
        driver.rightTrigger().whileTrue(new IntakePiece(intake, elevator));

        

        // driver.pov(0).whileTrue(drivetrain.applyRequest(() ->
        //     angle.withVelocityX(-driver.getLeftY() * MaxSpeed)
        //     .withVelocityY(-driver.getLeftX() * MaxSpeed)
        //     .withTargetDirection(new Rotation2d(Math.toRadians(0))))
        // );


        //elevator.m_HeightLocation = 4;
        if (driver.pov(0).getAsBoolean() == true){
            elevator.m_HeightLocation = 4;
            System.out.println("4");
    
        } else if (driver.pov(90).getAsBoolean() == true){
            elevator.m_HeightLocation = 3;
    
        } else if (driver.pov(180).getAsBoolean() == true){
            elevator.m_HeightLocation = 2;
    
        } else if (driver.pov(270).getAsBoolean() == true){
            elevator.m_HeightLocation = 1;
    
        }
        // driver.pov(90).onTrue( new ScoreHeight(3, elevator, arm));
        // driver.pov(180).onTrue( new ScoreHeight(2, elevator, arm));
        // driver.pov(270).onTrue( new ScoreHeight(1, elevator, arm));
        
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driver.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));







        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* First put the drivetrain into auto run mode, then run the auto */
        return autoChooser.getSelected();
    }

    public Rotation2d targetangle() {
        /* First put the drivetrain into auto run mode, then run the auto */
        SwerveDriveState state = drivetrain.getState();
        Pose2d pose = state.Pose;
        pose = new Pose2d(pose.getTranslation(), new Rotation2d(0));
        Pose2d targetpose = new Pose2d(16.7,5.5,new Rotation2d(0));
        System.out.println(PhotonUtils.getYawToPose(pose,targetpose));
        return PhotonUtils.getYawToPose(pose,targetpose);
        
    }

    double X;
    double Y;
    double intercpet = Math.tan(Units.degreesToRadians(30))*4.5;
    double slope = Math.tan(Units.degreesToRadians(30));
     // Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(
            2.5, 2.5,
            Units.degreesToRadians(540), Units.degreesToRadians(720));

    private Command pathfindingCommand(boolean left) {
        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Map<String, Command> commandMap = new HashMap<>();
        commandMap.put("error", AutoBuilder.pathfindToPose(
            new Pose2d(6, 6, Rotation2d.fromDegrees(0)),
            constraints,
            0.0
        ));
        commandMap.put("A", AutoBuilder.pathfindToPose(
            new Pose2d(3.2, 4.3, Rotation2d.fromDegrees(0)),
            constraints,
            0.0
        ));
        commandMap.put("B", AutoBuilder.pathfindToPose(
            new Pose2d(3.2, 3.8, Rotation2d.fromDegrees(0)),
            constraints,
            0.0
        ));
        commandMap.put("C", AutoBuilder.pathfindToPose(
            new Pose2d(3.7, 3.0, Rotation2d.fromDegrees(60)),
            constraints,
            0.0
        ));
        commandMap.put("D", AutoBuilder.pathfindToPose(
            new Pose2d(4.1, 2.8, Rotation2d.fromDegrees(60)),
            constraints,
            0.0
        ));
        commandMap.put("E", AutoBuilder.pathfindToPose(
            new Pose2d(4.9, 2.8, Rotation2d.fromDegrees(120)),
            constraints,
            0.0
        ));
        commandMap.put("F", AutoBuilder.pathfindToPose(
            new Pose2d(5.4, 3.1, Rotation2d.fromDegrees(120)),
            constraints,
            0.0
        ));
        commandMap.put("G", AutoBuilder.pathfindToPose(
            new Pose2d(5.8, 3.7, Rotation2d.fromDegrees(180)),
            constraints,
            0.0
        ));
        commandMap.put("H", AutoBuilder.pathfindToPose(
            new Pose2d(5.8, 4.3, Rotation2d.fromDegrees(180)),
            constraints,
            0.0
        ));
        commandMap.put("I", AutoBuilder.pathfindToPose(
            new Pose2d(5.4, 5, Rotation2d.fromDegrees(240)),
            constraints,
            0.0
        ));
        commandMap.put("J", AutoBuilder.pathfindToPose(
            new Pose2d(5, 5.3, Rotation2d.fromDegrees(240)),
            constraints,
            0.0
        ));
        commandMap.put("K", AutoBuilder.pathfindToPose(
            new Pose2d(4.1, 5.3, Rotation2d.fromDegrees(300)),
            constraints,
            0.0
        ));
        commandMap.put("L", AutoBuilder.pathfindToPose(
            new Pose2d(3.7, 5, Rotation2d.fromDegrees(300)),
            constraints,
            0.0
        ));
        

        return Commands.select(commandMap, () -> {
            Pose2d currentPose = drivetrain.getState().Pose;
            X = currentPose.getTranslation().getX();
            Y = currentPose.getTranslation().getY();
            System.out.println(X);
            System.out.println(Y);
            if (Y <= -X*slope + intercpet+4 && Y >= X*slope - intercpet+4 && left && X <= 4.5) {
            System.out.println("A");
            return "A";
            } else if (Y <= -X*slope + intercpet+4 && Y >= X*slope - intercpet+4 && !left && X <= 4.5) {
            System.out.println("B");
            return "B";
            } else if (Y <= X*slope - intercpet+4 && X <= 4.5 && left) {
            System.out.println("C");
            return "C";
            } else if (Y <= X*slope - intercpet+4 && X <= 4.5 && !left) {
            System.out.println("D");
            return "D";
            } else if (Y <= -X*slope + intercpet+4 && X >= 4.5 && left) {
            System.out.println("E");
            return "E";
            } else if (Y <= -X*slope + intercpet+4 && X >= 4.5 && !left) {
            System.out.println("F");
            return "F";
            } else if (Y >= -X*slope + intercpet+4 && Y <= X*slope - intercpet+4 && left && X >= 4.5) {
            System.out.println("G");
            return "G";
            } else if (Y >= -X*slope + intercpet+4 && Y <= X*slope - intercpet+4 && !left && X >= 4.5) {
            System.out.println("H");
            return "H";
            } else if (Y >= X*slope - intercpet+4 && X >= 4.5 && left) {
            System.out.println("I");
            return "I";
            } else if (Y >= X*slope - intercpet+4 &&  X >= 4.5 && !left) {
            System.out.println("J");
            return "J";
            } else if (Y >= -X*slope + intercpet+4 && X <= 4.5 && left) {
            System.out.println("K");
            return "K";
            } else if (Y >= -X*slope + intercpet+4 && X <= 4.5 && !left) {
            System.out.println("L");
            return "L";
            } else {
            System.out.println("error");
            return "error";
            }
        });
    }
    
    private Command pathfindingtofollowCommand() {
        // Since we are using a holonomic drivetrain, the rotation component of this pose
        // represents the goal holonomic rotation
        PathPlannerPath path = null;
        try {
            path = PathPlannerPath.fromPathFile("Testpath");
        } catch (FileVersionException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        } catch (ParseException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }

        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
                4.0, 4.0,
                Units.degreesToRadians(540), Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        return AutoBuilder.pathfindThenFollowPath(
                path,
                constraints
                 
        );
    }
}
