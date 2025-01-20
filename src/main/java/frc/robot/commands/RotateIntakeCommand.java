// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RotateIntakeCommand extends Command {
  /** Creates a new RotateIntakeCommand. */
  public final IntakeSubsystem intake;
  public final XboxController intakeJoystick;

  public RotateIntakeCommand(IntakeSubsystem m_intake, XboxController m_joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    intake = m_intake;
    intakeJoystick = m_joystick;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (intakeJoystick.getLeftBumperButton() == true){
      intake.ManualIntakeRotate(Constants.IntakeConstants.DownSpeed);
    }
    else if (intakeJoystick.getRightBumperButton() == true){
      intake.ManualIntakeRotate(Constants.IntakeConstants.UpSpeed);
    }
    else{
      intake.ManualIntakeRotate(0);// stop motor
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
