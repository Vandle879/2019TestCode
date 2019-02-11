/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.RemoteLimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Robot;
import frc.robot.commands.*;

import edu.wpi.first.wpilibj.command.Subsystem;



/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class ExampleSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public TalonSRX testSRX = new TalonSRX(4);
  public TalonSRX testFollower1 = new TalonSRX(5);
  public TalonSRX testFollower2 = new TalonSRX(6);
  

// another change


  public ExampleSubsystem (){
    testFollower1.follow(testSRX);
    testFollower2.follow(testSRX);
    //testFollower1.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    //testSRX.configReverseLimitSwitchSource(RemoteLimitSwitchSource.RemoteTalonSRX,LimitSwitchNormal.NormallyOpen, 5, 10);
    testSRX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    Robot.LOG.addSource("Encoder", this, f -> "" + ((ExampleSubsystem)(f)).getPosition());
	






  }

  public boolean GetLimitSwitchValue (){

    return testFollower1.getSensorCollection().isRevLimitSwitchClosed();

    
  }

  public double getPosition(){
    return testSRX.getSensorCollection().getQuadraturePosition();
  }


  public void drive (double power){

    testSRX.set(ControlMode.PercentOutput, power);

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
   setDefaultCommand(new DriveWithJoystick());
  }
}
