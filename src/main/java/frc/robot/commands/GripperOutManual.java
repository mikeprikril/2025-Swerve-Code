// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsytem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GripperOutManual extends Command {
  /** Creates a new GripperInManual. */
  public final ArmSubsytem arm;
  public final XboxController operatorJoystick;

  public GripperOutManual(ArmSubsytem m_arm, XboxController m_operatorJoystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    arm = m_arm;
    operatorJoystick = m_operatorJoystick;

    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.GripperSpitOut();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.StopGripper();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !operatorJoystick.getRawButton(Constants.ArmConstants.gripperOutButton);
  }
}
