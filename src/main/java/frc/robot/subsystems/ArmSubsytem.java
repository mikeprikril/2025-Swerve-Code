// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsytem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  SparkMax armMotor;
  WPI_VictorSPX gripperWheel;

  SparkMaxConfig armMotorConfig = new SparkMaxConfig();
  double shoulderEncoder;

  DigitalInput armBottomLimitSwitch;
  DigitalInput armTopLimitSwitch;

  private SparkClosedLoopController pidController;

  public ArmSubsytem() {
    armMotorConfig.inverted(false) //dont invert shoulder motor
    .idleMode(IdleMode.kBrake); //keep brake on
    armMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder) //define encoder for control (might have to change)
    .pid(.0001, 0, 0) //PID constants
    .outputRange(-.3, 1); //arm PID range

    armMotor = new SparkMax(Constants.ArmConstants.armMotorCANID, MotorType.kBrushless);
    armMotor.configure(armMotorConfig, null, null);
    shoulderEncoder = armMotor.getAbsoluteEncoder().getPosition();

    gripperWheel = new WPI_VictorSPX(Constants.ArmConstants.GripperCANID);
  }

    public void ArmJoystickControl(double armCommandSpeed){
    if(armCommandSpeed > 0 && armBottomLimitSwitch.get() == false){ //dont move down if pushing lower limit switch
      armMotor.stopMotor();
    }
    else if(armCommandSpeed > 0 && shoulderEncoder < Constants.ArmConstants.AlmostDownValue){//go downn slow if close to low limit
      armMotor.set(Constants.ArmConstants.SlowDown*armCommandSpeed);
    }
    else if(armCommandSpeed < 0 && armTopLimitSwitch.get() == false){//dont move up if pushing upper limit switch
      armMotor.stopMotor();
    }
    else if(armCommandSpeed < 0 && shoulderEncoder > Constants.ArmConstants.AlmostUpValue){//go up slow if close to high limit
      armMotor.set(Constants.ArmConstants.SlowDown*armCommandSpeed);
    }
    else{
      armMotor.set(armCommandSpeed);
    }
  }

  public void AutoArmMove (double requestSpeed){
    armMotor.set(requestSpeed);
  }

  public void PIDArm(double ArmSetpoint){
    pidController.setReference(ArmSetpoint, ControlType.kPosition);
  
  }

  public void StopArm(){
    armMotor.stopMotor();
  }

  public double GetArmEncoderPosition(){
    return shoulderEncoder;
   }
  
  public void ResetArmEncoder(){
    if (armBottomLimitSwitch.get() == false){
      armMotor.getEncoder().setPosition(0); //reset encoder to zero when bottom limit is pressed
    }
   }
  
  public boolean GetTopLimitSwitch(){
    return armTopLimitSwitch.get();
   }
    
  public boolean GetBottomLimitSwitch(){
    return armBottomLimitSwitch.get();
   }

  public void GripperIntake(){
    gripperWheel.set(Constants.ArmConstants.GripperInSpeed);
  }
  public void GripperSpitOut(){
    gripperWheel.set(Constants.ArmConstants.GripperOutSpeed);

  }  public void StopGripper(){
    gripperWheel.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
