package org.firstinspires.ftc.teamcode.vision;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Rotation2d;

import org.opencv.core.RotatedRect;

@Config
public class Sample {
    public static double TO_INCHES_RATIO = 0.25;
    public static double Y_OFFSET = 6;
    public static double ANGLE_COEFF = -0.32;
    public static double X_ANGLE_COEFF = 0;
    public static double Y_ANGLE_COEFF = 0.1;
    public static double DISTANCE_COEFF = 1.4;

    private final double x, y, z;

    private final String color;

    private final RotatedRect rect;

    public Sample(double x, double y, double z, String color, RotatedRect rect) {
        this.x = toInches(x);
        this.y = -toInches(y) + Y_OFFSET;
        this.z = toInches(z);
        this.color = color;
        this.rect = rect;
    }

    public static double toInches(double a) {
        return a * TO_INCHES_RATIO;
    }

    @NonNull
    public String toString() {
        return "Sample at (" + Math.round(x) + ", " + Math.round(y) + ", " + Math.round(z) + ") with color " + color + "With size " + this.rect.size;
    }

    public double getAngle() {
        return Math.toDegrees(Math.atan(x / y)) * ANGLE_COEFF * (1 + y * Y_ANGLE_COEFF) * (1 + x * X_ANGLE_COEFF);
    }

    public double getDistance() {
        return Math.sqrt(x * x + y * y) * DISTANCE_COEFF;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getZ() {
        return z;
    }

    public String getColor() {
        return color;
    }

    public RotatedRect getRect() {
        return rect;
    }
}
