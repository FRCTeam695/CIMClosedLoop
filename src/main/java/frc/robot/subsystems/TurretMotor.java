/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretMotor extends SubsystemBase {
  /**
   * Creates a new TurretMotor.
   */
  private VictorSPX motor;
  private CimClosedLoop TopShooterMotor;
  private CimClosedLoop BottomShooterMotor;
  private double gain = .4;
  //private double multiple = .3;
  private NetworkTable LimeLight;
  private NetworkTableEntry LimeLightAzimuth;
  private NetworkTableEntry LimeLightCoPolar;
  private NetworkTableEntry LimeLightContourArea;
  
  public TurretMotor(NetworkTableInstance RobotMainNetworkTableInstance, int motorNum,CimClosedLoop TopShootorMotor,CimClosedLoop BottomShootorMotor) {
    this.LimeLight = RobotMainNetworkTableInstance.getTable("limelight");
    this.LimeLightAzimuth = LimeLight.getEntry("tx");
		this.LimeLightCoPolar = LimeLight.getEntry("ty");
    this.LimeLightContourArea = LimeLight.getEntry("ta");
    this.motor = new VictorSPX(motorNum);
    this.TopShooterMotor = TopShootorMotor;
    this.BottomShooterMotor = BottomShootorMotor;
  }
  
  public void setPower(double power) {
    if(Math.abs(power) > 1)
      throw new IllegalArgumentException("Speed is too high");
    power *= gain;

    motor.set(ControlMode. PercentOutput, power);
  }

  public double getAzimuth(){
    return LimeLightAzimuth.getDouble(0.0);
  }
  public double getCoPolar(){
    return LimeLightCoPolar.getDouble(0.0);
  }
  public double getContourArea(){
    return LimeLightContourArea.getDouble(0.0);
  }

  public double getDistanceToContour() {
    return (82.0-Constants.LIMELIGHT_MOUNT_HEIGHT)/Math.tan(Math.toRadians(Constants.LIMELIGHT_MOUNT_ANGLE+getCoPolar()));
  }

  public double getDistanceToContourInFeet() {
    return 11.391581132*getDistanceToContour();
  }

  public double determineBottomMotorPercent() {
    double distance = getDistanceToContourInFeet();
    return -23.80952+12.53968*distance-1.160714*Math.pow(distance,2)+0.03472222*Math.pow(distance,3);
  }
  public double setMotorPower(double percent) {
    
  }
  public double setMotorPowers(double topPercent,double bottomPercent) {
    TopShooterMotor.setVelocity(topPercent*10000);
    BottomShooterMotor.setVelocity(bottomPercent*10000);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
