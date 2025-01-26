// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  SparkMax leftElevatorMotor;
  SparkMax rightElevatorMotor;

  //SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
  //SparkMaxConfig rightMotorConfig = new SparkMaxConfig();
  double elevatorEncoder;

  DigitalInput elevatorBottomLimitSwitch;
  DigitalInput elevatorTopLimitSwitch;

  private SparkClosedLoopController pidController;

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  
  public ElevatorSubsystem() {
    
    /*leftMotorConfig.inverted(false) //dont invert left motor
    .idleMode(IdleMode.kBrake); //keep brake on
    leftMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder) //define encoder for control (might have to change)
    .pid(.0001, 0, 0) //PID constants
    .outputRange(-1, 1); //PID elevator range

    rightMotorConfig.inverted(true) //invert right motor so it spins opposite way
    .idleMode(IdleMode.kBrake) //keep brake on
    .follow(leftElevatorMotor); //follow the left motor
    */
    
    leftElevatorMotor = new SparkMax(Constants.ElevatorConstants.leftMotorCANID, MotorType.kBrushless);
    //leftElevatorMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    elevatorEncoder = leftElevatorMotor.getAbsoluteEncoder().getPosition();

    rightElevatorMotor = new SparkMax(Constants.ElevatorConstants.rightMotorCANID, MotorType.kBrushless);
    //rightElevatorMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    pidController = leftElevatorMotor.getClosedLoopController();

  }

  public void ElevatorJoystickControl(double elevatorCommandSpeed){
    if(elevatorCommandSpeed > 0 && elevatorBottomLimitSwitch.get() == false){ //dont move down if pushing lower limit switch
      leftElevatorMotor.stopMotor();
      rightElevatorMotor.stopMotor();
    }
    else if(elevatorCommandSpeed > 0 && elevatorEncoder < Constants.ElevatorConstants.AlmostDownValue){//go downn slow if close to low limit
      leftElevatorMotor.set(Constants.ElevatorConstants.SlowDown*elevatorCommandSpeed);
      rightElevatorMotor.set(Constants.ElevatorConstants.SlowDown*-elevatorCommandSpeed);
    }
    else if(elevatorCommandSpeed < 0 && elevatorTopLimitSwitch.get() == false){//dont move up if pushing upper limit switch
      leftElevatorMotor.stopMotor();
      rightElevatorMotor.stopMotor();
    }
    else if(elevatorCommandSpeed < 0 && elevatorEncoder > Constants.ElevatorConstants.AlmostUpValue){//go up slow if close to high limit
      leftElevatorMotor.set(Constants.ElevatorConstants.SlowDown*elevatorCommandSpeed);
      rightElevatorMotor.set(Constants.ElevatorConstants.SlowDown*-elevatorCommandSpeed);
    }
    else{
      leftElevatorMotor.set(elevatorCommandSpeed);
      rightElevatorMotor.set(-elevatorCommandSpeed);
    }
  }

  public void AutoElevator (double requestSpeed){
    leftElevatorMotor.set(requestSpeed);
    rightElevatorMotor.set(-requestSpeed);
  }

  public void PIDElevator(double ElevatorSetpoint){
    pidController.setReference(ElevatorSetpoint, ControlType.kPosition);
  
  }

  public void StopElevator(){
    leftElevatorMotor.stopMotor();
    rightElevatorMotor.stopMotor();
  }

  public double GetElevatorEncoderPosition(){
    return elevatorEncoder;
   }
  
  public void ResetArmEncoder(){
    if (elevatorBottomLimitSwitch.get() == false){
      leftElevatorMotor.getEncoder().setPosition(0);
    }
   }
  
  public boolean GetTopLimitSwitch(){
    return elevatorTopLimitSwitch.get();
   }
    
  public boolean GetBottomLimitSwitch(){
    return elevatorBottomLimitSwitch.get();
   }


  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator Encoder Value", elevatorEncoder); //show elevator encoder on dashboard
    SmartDashboard.putBoolean("Elevator Bottom Limit", elevatorBottomLimitSwitch.get());
    SmartDashboard.putBoolean("Elevator Top Limit", elevatorTopLimitSwitch.get());
    SmartDashboard.putNumber("Left Elevator Current ", leftElevatorMotor.getOutputCurrent()); //display motor current draw in amps
    SmartDashboard.putNumber("Right Elevator Current ", rightElevatorMotor.getOutputCurrent());

    SmartDashboard.putNumber("Alt Encoder Velocity", leftElevatorMotor.getAbsoluteEncoder().getVelocity());
    SmartDashboard.putNumber("Left Elevator Applied Output", leftElevatorMotor.getAppliedOutput());
    SmartDashboard.putNumber("Right Elevator Applied Output", rightElevatorMotor.getAppliedOutput());

  }
}
