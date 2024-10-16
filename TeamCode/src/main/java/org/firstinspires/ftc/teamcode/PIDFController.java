package org.firstinspires.ftc.teamcode;

public class PIDFController {

    private double kP, kI, kD, kF;
    private double setpoint;
    private double integralSum;
    private double previousError;
    private double previousTime;

    public PIDFController(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.integralSum = 0.0;
        this.previousError = 0.0;
        this.previousTime = System.currentTimeMillis();
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
        // Reset integral and previous error when setting a new setpoint
        this.integralSum = 0.0;
        this.previousError = 0.0;
    }

    public double calculate(double current) {
        double currentTime = System.currentTimeMillis();
        double deltaTime = (currentTime - previousTime) / 1000.0; // Convert to seconds
        double error = setpoint - current;

        // Proportional term
        double proportional = kP * error;

        // Integral term
        integralSum += error * deltaTime;
        double integral = kI * integralSum;

        // Derivative term
        double derivative = kD * (error - previousError) / deltaTime;

        // Feedforward term (typically constant)
        double feedforward = kF * setpoint;

        // Update previous error and time
        previousError = error;
        previousTime = currentTime;

        // Calculate the PIDF output
        return proportional + integral + derivative + feedforward;
    }

    public double getSetpoint() {
        return setpoint;
    }

    public double getP() {
        return kP;
    }

    public void setP(double kP) {
        this.kP = kP;
    }

    public double getI() {
        return kI;
    }

    public void setI(double kI) {
        this.kI = kI;
    }

    public double getD() {
        return kD;
    }

    public void setD(double kD) {
        this.kD = kD;
    }

    public double getF() {
        return kF;
    }

    public void setF(double kF) {
        this.kF = kF;
    }
}

