// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  WPI_VictorSPX leftIntakeMotor;
  WPI_VictorSPX rightIntakeMotor;
  SparkMax intakeRotateMotor;

  SparkMaxConfig intakeRotateConfig = new SparkMaxConfig();


  DigitalInput IntakeDownLimitSwitch;
  DigitalInput IntakeUpLimitSwitch;

  public IntakeSubsystem() {
  
  leftIntakeMotor = new WPI_VictorSPX(Constants.IntakeConstants.LeftIntakeCANID);
  rightIntakeMotor = new WPI_VictorSPX(Constants.IntakeConstants.RightIntakeCANID);

  intakeRotateConfig.inverted(false);
  intakeRotateMotor = new SparkMax(Constants.IntakeConstants.RotateCANID, MotorType.kBrushless);
  intakeRotateMotor.configure(intakeRotateConfig, null, null);

  IntakeDownLimitSwitch = new DigitalInput(Constants.IntakeConstants.IntakeBottomLimitSwitchIO);
  IntakeUpLimitSwitch = new DigitalInput(Constants.IntakeConstants.IntakeTopLimitSwitchIO);

  }

  public void intakeRest()  {
    leftIntakeMotor.stopMotor();
    rightIntakeMotor.stopMotor();
  }

public void intakeActive()  {
    leftIntakeMotor.set(Constants.IntakeConstants.MotorSpeed);
    rightIntakeMotor.set(-Constants.IntakeConstants.MotorSpeed*Constants.IntakeConstants.Offset);
  }

public void spitOut()  {
    leftIntakeMotor.set(Constants.IntakeConstants.SpitOutSpeed);
    rightIntakeMotor.set(-Constants.IntakeConstants.SpitOutSpeed);
  }

public void ManualIntakeRotate(double intakeRotateSpeed){
  if(intakeRotateSpeed > 0 && IntakeDownLimitSwitch.get() == false){ //dont move down if pushing lower limit switch
    intakeRotateMotor.stopMotor();
  }
  else if(intakeRotateSpeed < 0 && IntakeUpLimitSwitch.get() == false){//dont move up if pushing upper limit switch
    intakeRotateMotor.stopMotor();
  }
  else{
    intakeRotateMotor.set(intakeRotateSpeed);
  }
}

public void autoIntakeDown(){
  if(IntakeDownLimitSwitch.get() == false){ //dont move down if pushing lower limit switch
    intakeRotateMotor.stopMotor();
  }
  else{
    intakeRotateMotor.set(-Constants.IntakeConstants.AutoRotateSpeed);
  }
}

public void autoIntakeUp(){
  if(IntakeUpLimitSwitch.get() == false){ //dont move up if pushing upper limit switch
    intakeRotateMotor.stopMotor();
  }
  else{
    intakeRotateMotor.set(Constants.IntakeConstants.AutoRotateSpeed);
  }
}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
