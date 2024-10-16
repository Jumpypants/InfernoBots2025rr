package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import org.opencv.core.RotatedRect;

public class Sample {
    double x = 0;
    double y = 0;
    double z = 0;

    String color = null;

    RotatedRect rect = null;

    public Sample(double x, double y, double z, String color, RotatedRect rect) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.color = color;
        this.rect = rect;
    }

    @NonNull
    public String toString() {
        return "Sample at (" + Math.round(x) + ", " + Math.round(y) + ", " + Math.round(z) + ") with color " + color;
    }
}
