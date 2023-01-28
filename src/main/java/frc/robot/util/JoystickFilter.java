package frc.robot.util;

public class JoystickFilter {
    public enum Mode {
        LINEAR, SQUARED, CUBED, ROOT
    }

    private Mode mode;
    private double deadband;
    private double minPower;
    private double maxPower;

    public JoystickFilter(double deadband, double minPower, double maxPower, Mode mode) {
        this.deadband = deadband;
        this.minPower = minPower;
        this.maxPower = maxPower;
        this.mode = mode;
    }

    public Mode getMode() {
        return mode;
    }

    public void setMode(Mode mode) {
        this.mode = mode;
    }

    public double getDeadband() {
        return deadband;
    }

    public void setDeadband(double deadband) {
        this.deadband = deadband;
    }

    public double getMinPower() {
        return minPower;
    }

    public void setMinPower(double minPower) {
        this.minPower = minPower;
    }

    public double getMaxPower() {
        return maxPower;
    }

    public void setMaxPower(double maxPower) {
        this.maxPower = maxPower;
    }

    private double scale(double input,
            double lowInput, double highInput, double lowOutput, double highOutput)
    {
        final double inputRange = highInput - lowInput;
        final double outputRange = highOutput - lowOutput;
        
        return (input - lowInput) * outputRange / inputRange + lowOutput;
    }

    public double filter(double input) {
        double sign = Math.signum(input);
        double output = 0;
        
        input = Math.abs(input);
        if (input < deadband)
            return 0;

        switch (mode) {
        case LINEAR:
            output = scale(input, deadband, 1, minPower, maxPower);
            break;

        case SQUARED:
            output = scale(input * input, deadband * deadband, 1, minPower, maxPower);
            break;

        case CUBED:
            output = scale(input * input * input, deadband * deadband * deadband , 1, minPower, maxPower);
            break;
        
        case ROOT:
            output = scale(Math.sqrt(input), Math.sqrt(deadband), 1, minPower, maxPower);
            break;

        }
        output *= sign;
        return output;
    }
}