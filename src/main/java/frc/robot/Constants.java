// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kLeftLead = 0;
    public static final double kLeftFollow = 0;
    public static final double kRightLead = 0;
    public static final double kRightFollow = 0;
    public static final double kVeolcityRPMConversion = 0;
    public static final double kIOMotorRPM = 0;
    public static final double kHoldingMotorPower = 0;
    public static final double kIOEjectRPM = 0;
    public static final double kPivotMotorVelocity = 0;
    public static final double kPivotMotorPower = 0;
    public static final double kPivotMotorMinAngle = 0;
    public static final double kPivotMotorMaxAngle = 0;
    // kPivotMotorMinAngle should be kept at zero, as it is not absolute, but a representation of the furthest back the arm is capable of extending.
    public static final int kPivotMotorID = 0;
    public static final int kLeftIOMotorID = 1;
    public static final int kRightIOMotorID = 2;
    public static final int kLeftHoldingMotorID = 3;
    public static final int kRightHoldingMotorID = 4;
    public static final int kLeftJoystickID = 100;
    public static final int kRightJoystickID = 101;
  }
}
