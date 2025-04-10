package org.firstinspires.ftc.teamcode.Swerve;

/**
 * Custom PIDF controller implementation that handles continuous inputs (like angles)
 * Includes features needed for swerve drive control
 */
public class CustomPIDFController {

    // Controller coefficients
    private double kP, kI, kD, kF;
    private double setPoint;        // Target value
    private double measuredValue;   // Current/actual value
    private double minIntegral, maxIntegral;  // Integration limits to prevent windup

    // Error tracking
    private double errorVal_p;      // Position error (proportional term)
    private double errorVal_v;      // Velocity error (derivative term)

    private double totalError;      // Accumulated error (integral term)
    private double prevErrorVal;    // Previous error for derivative calculation

    // Tolerance settings - when is "close enough" good enough
    private double errorTolerance_p = 0.05;
    private double errorTolerance_v = Double.POSITIVE_INFINITY;

    // Timing for derivative calculation
    private double lastTimeStamp;
    private double period;

    // Continuous input mode (for wrapping values like angles)
    private boolean continuous = false;
    private double minimumInput;
    private double maximumInput;

    /**
     * Basic constructor with just PIDF coefficients
     */
    public CustomPIDFController(double kp, double ki, double kd, double kf) {
        this(kp, ki, kd, kf, 0, 0);
    }

    /**
     * Full constructor with PIDF coefficients, setpoint, and measured value
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     * @param kf Feed-forward gain
     * @param sp Initial setpoint
     * @param pv Initial measured value
     */
    public CustomPIDFController(double kp, double ki, double kd, double kf, double sp, double pv) {
        kP = kp;
        kI = ki;
        kD = kd;
        kF = kf;

        setPoint = sp;
        measuredValue = pv;

        // Default integral limits
        minIntegral = -1.0;
        maxIntegral = 1.0;

        lastTimeStamp = 0;
        period = 0;

        errorVal_p = setPoint - measuredValue;
        reset();
    }

    /**
     * Reset controller state (accumulated error and timing)
     */
    public void reset() {
        totalError = 0;
        prevErrorVal = 0;
        lastTimeStamp = 0;
    }

    /**
     * Check if continuous input mode is enabled
     */
    public boolean isContinuousInputEnabled() {
        return continuous;
    }

    /**
     * Enable continuous input handling for wrapping values (like angles)
     * @param minimumInput Minimum value of valid range
     * @param maximumInput Maximum value of valid range
     */
    public void enableContinuousInput(double minimumInput, double maximumInput) {
        continuous = true;
        this.minimumInput = minimumInput;
        this.maximumInput = maximumInput;
    }

    /**
     * Disable continuous input handling
     */
    public void disableContinuousInput() {
        continuous = false;
    }

    /**
     * Set position error tolerance
     * @param positionTolerance Maximum acceptable position error
     */
    public void setTolerance(double positionTolerance) {
        setTolerance(positionTolerance, Double.POSITIVE_INFINITY);
    }

    /**
     * Set position and velocity error tolerances
     * @param positionTolerance Maximum acceptable position error
     * @param velocityTolerance Maximum acceptable velocity error
     */
    public void setTolerance(double positionTolerance, double velocityTolerance) {
        errorTolerance_p = positionTolerance;
        errorTolerance_v = velocityTolerance;
    }

    /**
     * Get current setpoint
     * @return The current setpoint value
     */
    public double getSetPoint() {
        return setPoint;
    }

    /**
     * Set a new setpoint for the controller
     * @param sp The new setpoint value
     */
    public void setSetPoint(double sp) {
        setPoint = sp;
        if (continuous) {
            // Handle wrap-around for continuous inputs (like angles)
            errorVal_p = inputModulus(sp - measuredValue, minimumInput, maximumInput);
        } else {
            errorVal_p = setPoint - measuredValue;
        }
        errorVal_v = (errorVal_p - prevErrorVal) / period;
    }

    /**
     * Calculate the modulus for continuous inputs
     * Handles wrap-around for values like angles
     */
    public static double inputModulus(double input, double minimumInput, double maximumInput) {
        double modulus = maximumInput - minimumInput;

        // Wrap input if it's above the maximum input
        int numMax = (int) ((input - minimumInput) / modulus);
        input -= numMax * modulus;

        // Wrap input if it's below the minimum input
        int numMin = (int) ((input - maximumInput) / modulus);
        input -= numMin * modulus;

        return input;
    }

    /**
     * Check if controller has reached setpoint
     * @return True if error is within tolerance
     */
    public boolean atSetPoint() {
        return Math.abs(errorVal_p) < errorTolerance_p
                && Math.abs(errorVal_v) < errorTolerance_v;
    }

    /**
     * Get current controller coefficients
     * @return Array of [kP, kI, kD, kF] values
     */
    public double[] getCoefficients() {
        return new double[]{kP, kI, kD, kF};
    }

    /**
     * Get current position error
     * @return Position error (setpoint - measured)
     */
    public double getPositionError() {
        return errorVal_p;
    }

    /**
     * Get current tolerances
     * @return Array of [position tolerance, velocity tolerance]
     */
    public double[] getTolerance() {
        return new double[]{errorTolerance_p, errorTolerance_v};
    }

    /**
     * Get current velocity error
     * @return Rate of change of error
     */
    public double getVelocityError() {
        return errorVal_v;
    }

    /**
     * Calculate control output using current measured value
     * @return Control output
     */
    public double calculate() {
        return calculate(measuredValue);
    }

    /**
     * Calculate control output for new measured value and setpoint
     * @param pv Current measured value
     * @param sp New setpoint
     * @return Control output
     */
    public double calculate(double pv, double sp) {
        // Set the setpoint to the provided value
        setSetPoint(sp);
        return calculate(pv);
    }

    /**
     * Main PIDF calculation method
     * @param pv Current measured value
     * @return Control output value
     */
    public double calculate(double pv) {
        prevErrorVal = errorVal_p;

        // Calculate time period since last call
        double currentTimeStamp = (double) System.nanoTime() / 1E9;
        if (lastTimeStamp == 0) lastTimeStamp = currentTimeStamp;
        period = currentTimeStamp - lastTimeStamp;
        lastTimeStamp = currentTimeStamp;

        measuredValue = pv;

        // Calculate error, handling continuous inputs if needed
        if (continuous) {
            errorVal_p = inputModulus(setPoint - pv, minimumInput, maximumInput);
        } else {
            errorVal_p = setPoint - pv;
        }

        // Calculate velocity error (derivative of error)
        if (Math.abs(period) > 1E-6) {
            errorVal_v = (errorVal_p - prevErrorVal) / period;
        } else {
            errorVal_v = 0;
        }

        // Calculate and limit integral term
        totalError += period * (setPoint - measuredValue);
        totalError = totalError < minIntegral ? minIntegral : Math.min(maxIntegral, totalError);

        // Calculate final control output using PIDF formula
        return kP * errorVal_p + kI * totalError + kD * errorVal_v + kF * setPoint;
    }

    /**
     * Set all PIDF coefficients
     */
    public void setPIDF(double kp, double ki, double kd, double kf) {
        kP = kp;
        kI = ki;
        kD = kd;
        kF = kf;
    }

    /**
     * Set integration limits to prevent windup
     */
    public void setIntegrationBounds(double integralMin, double integralMax) {
        minIntegral = integralMin;
        maxIntegral = integralMax;
    }

    /**
     * Reset accumulated integral error
     */
    public void clearTotalError() {
        totalError = 0;
    }

    // Getters and setters for individual coefficients
    public void setP(double kp) {
        kP = kp;
    }

    public void setI(double ki) {
        kI = ki;
    }

    public void setD(double kd) {
        kD = kd;
    }

    public void setF(double kf) {
        kF = kf;
    }

    public double getP() {
        return kP;
    }

    public double getI() {
        return kI;
    }

    public double getD() {
        return kD;
    }

    public double getF() {
        return kF;
    }

    public double getPeriod() {
        return period;
    }
}