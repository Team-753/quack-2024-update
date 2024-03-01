// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Drive.DriveTrain;
import frc.robot.subsystems.Mandible;
import frc.robot.subsystems.StreamDeck;
import frc.robot.commands.ArmConfirmPositionCommand;
import frc.robot.commands.ArmManualControlCommand;
import frc.robot.commands.ConsistentChargeStationAuto;
import frc.robot.commands.DefaultDriveCommand;
// import frc.robot.commands.DriveInDirectionCommand;
import frc.robot.commands.MandibleOuttakeCommand;
import frc.robot.commands.SetArmPositionCommand;
import frc.robot.commands.SubstationPickupCommand;
import frc.robot.commands.TurnToCommand;
import frc.robot.commands.AutonomousPickup.DriveUntilOnPieceCommand;
import frc.robot.commands.AutonomousPickup.LockOnPieceCommand;
import frc.robot.commands.AutonomousPickup.TurnToPieceCommand;
import frc.robot.commands.AutonomousPlacement.CalculateArmPositionCommand;
import frc.robot.commands.AutonomousPlacement.DriverConfirmCommand;
import frc.robot.commands.AutonomousPlacement.MandiblePlacementCommand;
import frc.robot.commands.AutonomousPlacement.MoveToPlacementCommand;
import frc.robot.commands.CommandGroups.FastChargeStationAutoCommand;
import frc.robot.subsystems.Arm;

//import java.io.File;
import java.util.HashMap;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.auto.AutoBuilder;

public class RobotContainer {
  private DriveTrain driveTrain;
  private Mandible mandible;
  private Arm arm;
  private StreamDeck streamDeck;
  private AutoBuilder swerveAutoBuilder;
  private HashMap<String, Command> eventMap;
  private CommandJoystick joystick;
  private CommandXboxController xboxController;
//  private String[] pathnames;
  public SendableChooser<String> autoChooser = new SendableChooser<>();
  SendableChooser<Integer> secondPiecePlacementChooser = new SendableChooser<>();
  private PathConstraints autoPathConstraints;

  public RobotContainer() {
    LimelightHelper.setPipelineIndex(Config.DimensionalConstants.limelightName, 0);
    // initialize subsystems
    driveTrain = new DriveTrain();
    arm = new Arm();
    mandible = new Mandible();


    // initialize inputs
    joystick = new CommandJoystick(Config.TeleoperatedConstants.joystickPort);
    xboxController = new CommandXboxController(Config.TeleoperatedConstants.xboxControllerPort);
    streamDeck = new StreamDeck(arm);


    // set default command
    driveTrain.setDefaultCommand(new DefaultDriveCommand(joystick, driveTrain));
    arm.setDefaultCommand(new ArmManualControlCommand(arm, xboxController));
    // mandible default command is set internally
    // arm doesn't have a default command, is triggerd on a given stick input

    autoPathConstraints = new PathConstraints(Config.AutonomousConstants.maxVelocity, Config.AutonomousConstants.maxAccel, 3.0, 6.0);
    // we need this for on the fly generation w/ no eventmap and no alliance color 
    generateEventMap();
    //swerveAutoBuilder = new SwerveAutoBuilder(driveTrain::getEstimatedPose, driveTrain::resetPose, new PIDConstants(Config.AutonomousConstants.translationKP, Config.AutonomousConstants.translationKI, Config.AutonomousConstants.translationKD), new PIDConstants(Config.AutonomousConstants.rotationKP, Config.AutonomousConstants.rotationKI, Config.AutonomousConstants.rotationKD), driveTrain::PPDrive, eventMap, true, driveTrain);
    //swerveAutoBuilder = AutoBuilder(driveTrain::getEstimatedPose, driveTrain::resetPose, driveTrain.kinematics, new PIDConstants(Config.AutonomousConstants.translationKP, Config.AutonomousConstants.translationKI, Config.AutonomousConstants.translationKD), new PIDConstants(Config.AutonomousConstants.rotationKP, Config.AutonomousConstants.rotationKI, Config.AutonomousConstants.rotationKD), driveTrain::PPSetStates, eventMap, true, driveTrain);
    configureBindings();

    // setting our autos
    autoChooser.setDefaultOption("Place Cube", "Place Cube");
    autoChooser.addOption("Place & Charge", "Place & Charge");
    autoChooser.addOption("Cable 2 Piece", "Cable 2 Piece");
    autoChooser.addOption("Open 2 Piece", "Open 2 Piece");
    autoChooser.addOption("Cable 2 Piece Alt", "Cable 2 Piece Alt");
    autoChooser.addOption("Open 2 Piece Alt", "Open 2 Piece Alt");
    autoChooser.addOption("Cable Taxi", "Cable Taxi");
    autoChooser.addOption("Open Taxi", "Open Taxi");
    autoChooser.addOption("Cable Pickup with Charge", "Cable Pickup with Charge");
    autoChooser.addOption("Open Pickup with Charge", "Open Pickup with Charge");
    autoChooser.addOption("Cable 2 Piece Mid", "Cable 2 Piece Mid");
    autoChooser.addOption("Open 2 Piece Mid", "Open 2 Piece Mid");
    // Grabbing the auto path names as to automatically populate our dashboard
    // File f = new File(System.getProperty("user.dir") + "/src/main/deploy/pathplanner");
    // pathnames = f.list();
    // for (int i = 0; i < pathnames.length; i++) {
    //   pathnames[i] = pathnames[i].replace(".path", "");
    //   autoChooser.addOption(pathnames[i], pathnames[i]);
    // }
    //autoChooser.addOption("Testing", "Testing");
    SmartDashboard.putData("Autonomous Chooser", autoChooser);

    secondPiecePlacementChooser.setDefaultOption("High Cone Right", 2);
    secondPiecePlacementChooser.addOption("High Cone Left", 0);
    secondPiecePlacementChooser.addOption("High Cube", 1);
    secondPiecePlacementChooser.addOption("Mid Cube", 4);
    SmartDashboard.putData("Auto Piece Chooser", secondPiecePlacementChooser);
    
    // DEBUGGING
    // if (Config.DEBUGGING.useDebugTab) {
    //   ShuffleboardTab debuggingTab = Shuffleboard.getTab("DEBUGGING");
    // }
    
  }

  private void configureBindings() {
    Trigger joystickFour = joystick.button(4);
    joystickFour.whileTrue(driveTrain.goXModeCommand);

    Trigger joystickTwelve = joystick.button(12);
    joystickTwelve.onTrue(driveTrain.enableSpeedLimiterCommand);
    joystickTwelve.onFalse(driveTrain.disableSpeedLimiterCommand);

    Trigger joystickEleven = joystick.button(11);
    joystickEleven.whileTrue(Commands.runOnce(() -> driveTrain.resetPose(new Pose2d()), driveTrain));

    // Trigger joystickTen = joystick.button(10);
    // joystickTen.whileTrue(new DriveInDirectionCommand(driveTrain, 1, 0, 0, false));

    // Trigger joystickNine = joystick.button(9);
    // joystickNine.whileTrue(new SequentialCommandGroup(
    //   // new SetArmPositionCommand(arm, "FloorPickupPrep"), // getting the arm into position
    //   //new TurnToPieceCommand(driveTrain), // turning to the expected angle of the game piece
    //   new LockOnPieceCommand(driveTrain, mandible), // doing the final correction using the limelight google coral pipeline
    //   new ArmConfirmPositionCommand(arm, "Floor"), // moving the arm into pickup position
    //   new DriveUntilOnPieceCommand(driveTrain, mandible),
    //   new SetArmPositionCommand(arm, "Optimized")));

    // Trigger joystickTwo = joystick.button(2);
    // joystickTwo.whileTrue(new SequentialCommandGroup(
    //   new CalculateArmPositionCommand(arm, streamDeck, false),
    //   new MoveToPlacementCommand(driveTrain, streamDeck),
    //   new DriverConfirmCommand(joystick, driveTrain, mandible),
    //   new CalculateArmPositionCommand(arm, streamDeck, true),
    //   new MandiblePlacementCommand(mandible),
    //   new SetArmPositionCommand(arm, "Optimized")
    // ));

    Trigger joystickThree = joystick.button(3);
    joystickThree.whileTrue(new SubstationPickupCommand(driveTrain, joystick));
    Trigger joystickFive = joystick.button(5);
    joystickFive.onTrue(Commands.runOnce(() -> mandible.setOpen(false), mandible));
    Trigger joystickSix = joystick.button(6);
    joystickSix.onTrue(Commands.runOnce(() -> mandible.setOpen(true), mandible));

    Trigger joystickSeven = joystick.button(7);
    joystickSeven.whileTrue(mandible.toggleIntakeInCommand);

    Trigger joystickEight = joystick.button(8);
    joystickEight.whileTrue(mandible.toggleIntakeOutCommand);

    xboxController.x().onTrue(Commands.runOnce(() -> mandible.setOpen(false), mandible));
    xboxController.b().onTrue(Commands.runOnce(() -> mandible.setOpen(true), mandible));
    xboxController.leftBumper().onTrue(new MandiblePlacementCommand(mandible));
    xboxController.a().whileTrue(mandible.toggleIntakeInCommand);
    xboxController.y().whileTrue(mandible.toggleIntakeOutCommand);

    xboxController.pov(0).onTrue(new SetArmPositionCommand(arm, "Optimized"));
    xboxController.pov(90).onTrue(new SetArmPositionCommand(arm, "Substation"));
    xboxController.pov(180).onTrue(new SetArmPositionCommand(arm, "Floor"));
    xboxController.pov(270).onTrue(new SetArmPositionCommand(arm, "FullyRetracted"));
  }

  public Command getAutonomousCommand() {
    //arm.startArmMovement(); // could be bad
    // mandible.setOpen(true);
    // mandible.passiveIntake();
    // SmartDashboard.putBoolean("Autonomous Finished", false);
    // String autoName = autoChooser.getSelected();
    // Command command;
    // switch (autoName) {
    //   case "Place Cube":
    //     command = eventMap.get("Place Cube");
    //     break;
    //   case "Place & Charge":
    //     // command = new SequentialCommandGroup(
    //     //   eventMap.get("Place Cube"),
    //     //   new ArmConfirmPositionCommand(arm, "FullyRetracted"),
    //     //   new ConsistentChargeStationAuto(driveTrain, arm, false),
    //     //   new SetArmPositionCommand(arm, "Optimized"),
    //     //   new WaitCommand(0.75),
    //     //   new ConsistentChargeStationAuto(driveTrain, arm, true),
    //     //   new SetArmPositionCommand(arm, "Substation")
    //     // );
    //     command = new SequentialCommandGroup(
    //       eventMap.get("Place Cube"),
    //       // new FastChargeStationAutoCommand(driveTrain, arm)
    //       new SetArmPositionCommand(arm, "FullyRetracted"),
    //       new TurnToCommand(driveTrain, Rotation2d.fromRadians(3.0 * Math.PI / 4.0)),
    //       new ConsistentChargeStationAuto(driveTrain, arm, false, false),
    //       new WaitCommand(0.65),
    //       new TurnToCommand(driveTrain, Rotation2d.fromRadians(Math.PI)),
    //       new ConsistentChargeStationAuto(driveTrain, arm, true, false),
    //       new SetArmPositionCommand(arm, "Substation")
    //     );
    //     break;
    //   default:
    //     //command = swerveAutoBuilder.fullAuto(PathPlannerAuto.loadPathGroup(autoName, autoPathConstraints));
    //     command = null;
    // }
    // return new SequentialCommandGroup(command, Commands.runOnce(() -> SmartDashboard.putBoolean("Autonomous Finished", true)));
    return null;
  }

  public void generateEventMap() {
    eventMap = new HashMap<String, Command>();
    eventMap.put("Place Cube", new SequentialCommandGroup(
      new ArmConfirmPositionCommand(arm, "HighCube"), 
      new MandibleOuttakeCommand(mandible), 
      new SetArmPositionCommand(arm, "Optimized")));
    eventMap.put("Mandible In", Commands.run(() -> mandible.intakeWheels()));
    eventMap.put("Mandible Out", Commands.run(() -> mandible.outtakeWheels()));
    eventMap.put("Mandible Off", Commands.run(() -> mandible.passiveIntake()));
    eventMap.put("Open Mandible", Commands.runOnce(() -> mandible.setOpen(true), mandible));
    eventMap.put("Close Mandible", Commands.runOnce(() -> mandible.setOpen(false), mandible));
    eventMap.put("Pickup Piece", new SequentialCommandGroup(
      // new SetArmPositionCommand(arm, "FloorPickupPrep"), // getting the arm into position
      new TurnToPieceCommand(driveTrain), // turning to the expected angle of the game piece
      new LockOnPieceCommand(driveTrain, mandible), // doing the final correction using the limelight google coral pipeline
      new ArmConfirmPositionCommand(arm, "Floor"), // moving the arm into pickup position
      new DriveUntilOnPieceCommand(driveTrain, mandible),
      new SetArmPositionCommand(arm, "Substation")));
    eventMap.put("Place Piece", new SequentialCommandGroup(
      new CalculateArmPositionCommand(arm, secondPiecePlacementChooser, false),
      new MoveToPlacementCommand(driveTrain, secondPiecePlacementChooser),
      new CalculateArmPositionCommand(arm, secondPiecePlacementChooser, true),
      new MandiblePlacementCommand(mandible),
      new SetArmPositionCommand(arm, "Optimized")
    ));
    eventMap.put("ArmFloorPickupPrep", new SetArmPositionCommand(arm, "FloorPickupPrep"));
    eventMap.put("ArmOptimized", new SetArmPositionCommand(arm, "Optimized"));
    eventMap.put("ForwardCharge", new SequentialCommandGroup(
      new ConsistentChargeStationAuto(driveTrain, arm, false),
      new WaitCommand(0.75),
      new ConsistentChargeStationAuto(driveTrain, arm, true)
    ));
    eventMap.put("ReverseCharge", new SequentialCommandGroup(
      new ConsistentChargeStationAuto(driveTrain, arm, false, true),
      new WaitCommand(0.75),
      new ConsistentChargeStationAuto(driveTrain, arm, true, true)
    ));
  }

  public void disabledPeriodic() {
    if (DriverStation.getAlliance().equals(DriverStation.Alliance.Red)) {
      driveTrain.setRedAlliance(true);
    }
    else {
      driveTrain.setRedAlliance(false);
    }
  }
}
