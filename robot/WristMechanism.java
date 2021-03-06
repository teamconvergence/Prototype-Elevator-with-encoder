/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * Add your docs here.
 */
public class WristMechanism extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  // public Encoder wristEncoder = new Encoder(RobotMap.p_secondencoderchannel1, RobotMap.p_secondencoderchannel2, false, Encoder.EncodingType.k4X);
  public WPI_TalonSRX wristMotor1 = new WPI_TalonSRX(RobotMap.p_wristmotor1);
  public WPI_TalonSRX wristMotor2 = new WPI_TalonSRX(RobotMap.p_wristmotor2);

  public DifferentialDrive wristMotor = new DifferentialDrive(wristMotor1, wristMotor2);

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  public void resetEncoder(){
    // wristEncoder.reset();
  }
  public void moveWristMotor(){

  }

}
