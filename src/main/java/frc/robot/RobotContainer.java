// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.AutoElevatorCommand;
import frc.robot.commands.GetCoral;
import frc.robot.commands.GripperInManual;
import frc.robot.commands.GripperOutManual;
import frc.robot.commands.ManualArmCommand;
import frc.robot.commands.ManualElevatorCommand;
import frc.robot.commands.TransferPosition;
import frc.robot.subsystems.ArmSubsytem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  // Controllers
  final XboxController driverXbox = new XboxController(ControllerConstants.DriverUSBPort);
  final XboxController operatorXbox = new XboxController(ControllerConstants.OperatorUSBPort);

  // Subsystems
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final ArmSubsytem arm = new ArmSubsytem();

  // Commands
  private final ManualElevatorCommand manualElevator;
  private final ManualArmCommand manualArm;
  private final AutoElevatorCommand autoElevator;;
  private final TransferPosition transfer;
  private final GetCoral getCoral;
  private final GripperInManual gripperIn;
  private final GripperOutManual gripperOut;

  private final SequentialCommandGroup AutoTransfer;


  //All the YAGSL Swerve Stuff - 
  private final SwerveSubsystem drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/neo"));
  // Applies deadbands and inverts controls because joysticks are back-right positive while robot controls are front-left positive
  // left stick controls translation
  // right stick controls the rotational velocity 
  // buttons are quick rotation positions to different ways to face
  // WARNING: default buttons are on the same buttons as the ones defined in configureBindings


  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(driverXbox::getRightX)
                                                            .deadband(ControllerConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);


  // Applies deadbands and inverts controls because joysticks are back-right positive while robot controls are front-left positive
  // left stick controls translation
  // right stick controls the desired angle NOT angular rotation
  Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);

  // Applies deadbands and inverts controls because joysticks are back-right positive while robot controls are front-left positive
  // left stick controls translation
  // right stick controls the angular velocity of the robot
  Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);


  public RobotContainer()
  {
    //initialize all teleop and auto commands here
    manualElevator = new ManualElevatorCommand(elevator, operatorXbox);
    manualArm = new ManualArmCommand(arm, operatorXbox);
    autoElevator = new AutoElevatorCommand(elevator, operatorXbox);
    transfer = new TransferPosition(elevator, arm, operatorXbox);
    getCoral = new GetCoral(elevator, arm, operatorXbox);
    gripperIn = new GripperInManual(arm, operatorXbox);
    gripperOut = new GripperOutManual(arm, operatorXbox);

    AutoTransfer = new SequentialCommandGroup(transfer, getCoral); //sequential command group for auto transfer

    //Pathplanner named commands go here
    //NamedCommands.registerCommand("Fire From Subwoofer", new FireFromSubwoofer(m_arm, m_shooter));

    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
  
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    // Default Commands
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    elevator.setDefaultCommand(manualElevator);
    arm.setDefaultCommand(manualArm);

    //Joystick Button Actions
    new JoystickButton(driverXbox, 8).onTrue(new InstantCommand(drivebase::zeroGyro));
    new JoystickButton(driverXbox, 9).whileTrue(drivebase.centerModulesCommand());

    new JoystickButton(operatorXbox, Constants.ElevatorConstants.L4JoystickButton).onTrue(autoElevator); //when pressing button 3, go to elevator position
    new JoystickButton(operatorXbox, Constants.ElevatorConstants.TransferButton).onTrue(transfer); //when pressing button 4, move to transfer position (change to autotransfer)
    new JoystickButton(operatorXbox, Constants.ArmConstants.gripperInButton).onTrue(gripperIn); //manual gripper in
    new JoystickButton(operatorXbox, Constants.ArmConstants.gripperOutButton).onTrue(gripperOut); //manual gripper out
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("New Auto");
  }

  public void setDriveMode()
  {
    configureBindings();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
