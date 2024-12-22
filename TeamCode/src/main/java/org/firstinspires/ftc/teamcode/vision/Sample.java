package org.firstinspires.ftc.teamcode.vision;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.RotatedRect;

@Config
public class Sample {
    // TODO: tune this value
    public static final double TO_INCHES_RATIO = 0.0393701;

    private double x, y, z;

    private String color;

    private RotatedRect rect;

    public Sample(double x, double y, double z, String color, RotatedRect rect) {
        this.x = toInches(x);
        this.y = toInches(y);
        this.z = toInches(z);
        this.color = color;
        this.rect = rect;
    }

    @NonNull
    public String toString() {
        return "Sample at (" + Math.round(x) + ", " + Math.round(y) + ", " + Math.round(z) + ") with color " + color;
    }

    public double getAngle() {
        return Math.atan2(z, x);
    }

    public double getDistance() {
        return Math.sqrt(x * x + z * z);
    }

    public double toInches(double a) {
        return a * TO_INCHES_RATIO;
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
}
