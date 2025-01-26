// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (115 + 15) * 0.453592; // weight in lb x kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS); //robot Center of Mass
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(17.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.


  public static final class DrivebaseConstants
  {
    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class ControllerConstants
  {
    //Joystick USB Port
    public static final int DriverUSBPort = 0;
    public static final int OperatorUSBPort = 2;

    // Joystick Deadband
    public static final double DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;

  }

  public static class IntakeConstants
  {
    public static final int LeftIntakeCANID = 15;
    public static final int RightIntakeCANID = 16;
    public static final int RotateCANID = 17;

    public static final int IntakeBottomLimitSwitchIO = 3;
    public static final int IntakeTopLimitSwitchIO = 4;
    
    public static final double MotorSpeed = .5;
    public static final double Offset = .8; //How much do we slow down one wheel to coral rotates
    public static final double SpitOutSpeed = -.8;
    public static final double AutoRotateSpeed = 0.5;

    public static final double DownSpeed = -.3;
    public static final double UpSpeed = .5;
      
  }

  public static class ElevatorConstants{
    public static final int leftMotorCANID = 10;
    public static final int rightMotorCANID = 11;

    public static final int elevatorBottomLimitSwitchIO = 1;
    public static final int elevatorTopLimitSwitchIO = 2;

    public static final int AlmostDownValue = 100; //slow down when close to bottom
    public static final int AlmostUpValue = 900; //slow down when close to top
    public static final double SlowDown = 0.3; //slow down by 30% if close to limit

    public static final double deadband = 10;
    public static final double TransferHeight = 50;
    public static final double L1Height = 40;
    public static final double L2Height = 90;
    public static final double L3Height = 120;
    public static final double L4Height = 200;
    public static final double AutoUpSpeed = .4;
    public static final double AutoDownSpeed = -.3;
    public static final double HoldElevatorSpeed = 0.1;

    public static final int TransferButton = 1;
    public static final int L4JoystickButton = 3;
  }

  public static class ArmConstants{
    public static final int armMotorCANID = 12;
    public static final int GripperCANID = 21;

    public static final int armBottomLimitSwitchIO = 3;
    public static final int armTopLimitSwitchIO = 4;

    public static final int AlmostDownValue = 30; //slow down when close to bottom
    public static final int AlmostUpValue = 90; //slow down when close to top
    public static final double SlowDown = 0.2; //slow down by 20% if close to limit

    public static final double ArmDownSpeed = -.3;

    public static final double GripperInSpeed = .5;
    public static final double GripperOutSpeed = -.3;
  }

}
