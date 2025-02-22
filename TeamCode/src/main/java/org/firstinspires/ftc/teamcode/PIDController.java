package org.firstinspires.ftc.teamcode;

public class PIDController {
    private double kP, kI, kD;
    private double setpoint;
    private double previousError;
    private double integral;

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.previousError = 0;
        this.integral = 0;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public double calculate(double current) {
        double error = setpoint - current;
        integral += error;
        double derivative = error - previousError;
        previousError = error;

        return kP * error + kI * integral + kD * derivative;
    }
}

