package org.firstinspires.ftc.teamcode.vision;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Rotation2d;

import org.opencv.core.RotatedRect;

@Config
public class Sample {
    public static double TO_INCHES_RATIO = 0.4;

    private double x, y, z;

    private final String color;

    private final RotatedRect rect;

    public Sample(double x, double y, double z, String color, RotatedRect rect) {
        this.x = toInches(x);
        this.y = toInches(y);
        this.z = toInches(z);
        this.color = color;
        this.rect = rect;
    }

    public static double toInches(double a) {
        return a * TO_INCHES_RATIO;
    }

    @NonNull
    public String toString() {
        return "Sample at (" + Math.round(x) + ", " + Math.round(y) + ", " + Math.round(z) + ") with color " + color;
    }

    public double getAngle() {
        return Math.atan(x / y);
    }

    public double getDistance() {
        return Math.sqrt(x * x + z * z);
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
