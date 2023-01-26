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
 // public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kLeftLead = 0;
    public static final int kLeftFollow = 0;
    public static final int kRightLead = 0;
    public static final int kRightFollow = 0;
    public static final double kVeolcityRPMConversion = 0;
    public static final double kIOMotorGatherPower = 0;
    public static final double kIOMotorLowPower = 0;
    public static final double kIOMotorMidPower = 0;
    public static final double kIOMotorHighPower = 0;
    public static final double kIOMotorMaxPower = 0;
    public static final double kPivotMotorVelocity = 0;
    public static final double kCOMRadius = 0; //Beginning of the arm to its center of mass (end effector included)
    //Do NOT set this above one
    public static final double kPivotMotorPower = 0;
    public static final double kPivotMotorGatherAngle = 0;
    public static final double kPivotMotorLowAngle = 0;
    public static final double kPivotMotorMidAngle = 0;
    public static final double kPivotMotorHighAngle = 0;
    public static final double kPivotMotorMinAngle = 0;
    public static final double kPivotMotorMaxAngle = 0;
    // Min and Max angle represent the angles at which the arm contacts the ground or its frame
    public static final int kPivotMotorID = 0;
    public static final int kTopIOMotorID = 1;
    public static final int kBottomIOMotorID = 2;
    public static final int kLeftHoldingMotorID = 3;
    public static final int kRightHoldingMotorID = 4;
    public static final int kLeftJoystickID = 100;
    public static final int kRightJoystickID = 101;
 // }
}
