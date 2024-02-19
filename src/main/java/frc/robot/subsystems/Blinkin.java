// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.constant.Constable;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.OIConstants;

public class Blinkin extends SubsystemBase {
  Spark blinkin;
XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
	public Blinkin() {
		blinkin = new Spark(0);
	}
		
	/**
	 * if the robot is not in hatMode and in normal drive, the LED turns solid white (0.93)
	 */
	public void c_lightsNormal() {
		blinkin.set(0.93);
	} 

  public void c_lightSet(double value) {
    blinkin.set(value);
  } 

  public void c_intakeBlinkin(){
    if (Constants.MyConstants.colorSensed == "ORANGE") {
      blinkin.set(0.77);
    } else if (Constants.MyConstants.colorSensed == "ORANGE THRU POLY") {
      blinkin.set(0.77);
    } else {
      blinkin.set(-0.25);
    }
  }

  public void c_shooterBlinkin() {
    if (Constants.MyConstants.ktriggerL >= 0.5) {
   blinkin.set(-0.35);
    } else {
  blinkin.set(0.77);
}
  }

  public void c_ampBlinkin() {
   blinkin.set(-0.85);
}


  public void c_sourceBlinkin() {
   if (Constants.MyConstants.colorSensed == "ORANGE") {
      blinkin.set(0.77);
    } else if (Constants.MyConstants.colorSensed == "ORANGE THRU POLY") {
      blinkin.set(0.77);
    } else {
      blinkin.set(-0.07);
    }
  } 
  public void c_floorBlinkin() {
     if (Constants.MyConstants.colorSensed == "ORANGE") {
      blinkin.set(0.77);
    } else if (Constants.MyConstants.colorSensed == "ORANGE THRU POLY") {
      blinkin.set(0.77);
    } else {
      blinkin.set(0.67);
    }
  }
}

