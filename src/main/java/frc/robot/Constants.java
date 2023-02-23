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
    public static final int eeRefRateMod = 5;
    public static final int kDriverControllerPort = 2;
    public static final int kRightLead = 3;
    public static final int kLeftFollow = 4;
    public static final int kLeftLead = 5;
    public static final int kRightFollow = 6;
    public static final int kPDB = 63;
    public static final double kVeolcityRPMConversion = 0;
    public static final double kIOMotorGatherPower = -1.3;
    public static final double kIOMotorLowPower = 2;
    public static final double kIOMotorMidPower = 4;
    public static final double kIOMotorHighPower = 7.5;
    public static final double kIOMotorMaxPower = 12;
    public static final double kIOBottomToTopVoltageConversion = 1;
    public static final double kZeroOffset = 673.72314453125-562.4014282226562+100;
    public static final double kPivotMotorStowAngle = 1;
    public static final double kPivotMotorGatherAngle = 430;
    public static final double kPivotMotorLowAngle = 20;
    public static final double kPivotMotorMidAngle = 110;
    public static final double kPivotMotorHighAngle = 400;
    public static final double kPivotMotorMinAngle = 0;
    public static final double kPivotMotorMaxAngle = 500;
    // Min and Max angle represent the angles at which the arm contacts the ground or its frame
    public static final int kPivotMotorID = 2;
    public static final int kTopIOMotorID = 1;
    public static final int kBottomIOMotorID = 7;
    public static final int kLeftHoldingMotorID = 23;
    public static final int kRightHoldingMotorID = 24;
    public static final int kLeftJoystickID = 100;
    public static final int kRightJoystickID = 101;
    public static final double veolcityRPMConversion = 0;
    public static final double kVelocityConversionFactor = 1;
    public static final double kPositionConversionFactor = 1000;
    public static final double kWrappedPIDMinInput = -1;
    public static final double kWrappedPIDMaxInput = 1;

    public static final int kFrontSensorID = 10;
    public static final int kBackSensorID = 11;
    public static final double kMMToInches = 0.039;
    public static final double kDetectionThreshold = 13;

    // Multiply raw encoder output by this to convert that to feet travelled
    public static final double kEncoderTicksToFeet = 0; //TBD
    // For trajectory differentialDriveKinematics
    public static final double distBetweenWheelsMeters = 0; //TBD
 // }
}
