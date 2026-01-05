package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Wayfinder.Wayfinder;

public class WayfinderBlocks extends BlocksOpModeCompanion {

    public static Wayfinder wayfinder;

    @ExportToBlocks(
            comment = "",
            parameterLabels = {"P Gain", "D Gain", "Acceleration", "DistanceUnit", "Tolerance"},
            parameterDefaultValues = {"0.008", "0.0001", "10", "DistanceUnit.MM", "12"}
    )
    public static void setXYCoefficients(double p, double d, double acceleration, DistanceUnit unit, double tolerance){
        wayfinder.setXYCoefficients(p,d,acceleration,unit,tolerance);
    }

    @ExportToBlocks(
            comment = "",
            parameterLabels = {"P Gain", "D Gain", "Acceleration", "AngleUnit", "Tolerance"},
            parameterDefaultValues = {"5.0", "0.0", "20", "AngleUnit.DEGREES", "2.0"}
    )
    public static void setYawCoefficients(double p, double d, double acceleration, AngleUnit unit, double tolerance){
        wayfinder.setYawCoefficients(p,d,acceleration,unit,tolerance);
    }

    @ExportToBlocks(
            comment = ""
    )
    public static double getFrontLeftMotorOutput(){
        return wayfinder.getFrontLeftMotorOutput();
    }

    @ExportToBlocks(
            comment = ""
    )
    public static double getFrontRightMotorOutput(){
        return wayfinder.getFrontRightMotorOutput();
    }

    @ExportToBlocks(
            comment = ""
    )
    public static double getBackLeftMotorOutput(){
        return wayfinder.getBackLeftMotorOutput();
    }

    @ExportToBlocks(
            comment = ""
    )
    public static double getBackRightMotorOutput(){
        return wayfinder.getBackRightMotorOutput();
    }

    @ExportToBlocks(
            comment = "",
            parameterLabels = {"Current Position", "Target Position", "maxPower", "holdTime (seconds)"},
            parameterDefaultValues = {"New Pose2D", "New Pose2D", "0.5", "2"}
    )
    public static boolean driveTo(Pose2D currentPosition, Pose2D targetPosition, double maxPower, double holdTime){
        return wayfinder.driveTo(currentPosition,targetPosition,maxPower,holdTime);
    }
}
