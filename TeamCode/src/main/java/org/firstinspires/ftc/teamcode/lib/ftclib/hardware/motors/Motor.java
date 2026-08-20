package org.firstinspires.ftc.teamcode.lib.ftclib.hardware.motors;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.lib.ftclib.hardware.HardwareDevice;

import java.util.function.Supplier;

/**
 * This is the common wrapper for the {@link DcMotor} object in the
 * FTC SDK.
 *
 * @author Jackson
 */
public class Motor implements HardwareDevice {

    public enum GoBILDA {
        RPM_30(5264, 30), RPM_43(3892, 43), RPM_60(2786, 60), RPM_84(1993.6, 84),
        RPM_117(1425.2, 117), RPM_223(753.2, 223), RPM_312(537.6, 312), RPM_435(383.6, 435),
        RPM_1150(145.6, 1150), RPM_1620(103.6, 1620), BARE(28, 6000), NONE(0, 0);

        private double cpr, rpm;

        GoBILDA(double cpr, double rpm) {
            this.cpr = cpr;
            this.rpm = rpm;
        }

        public double getCPR() {
            return cpr;
        }

        public double getRPM() {
            return rpm;
        }

        public double getAchievableMaxTicksPerSecond() {
            return cpr * rpm / 60;
        }
    }

    public enum Direction {
        FORWARD(1), REVERSE(-1);

        private int val;

        Direction(int multiplier) {
            val = multiplier;
        }

        public int getMultiplier() {
            return val;
        }
    }

    public class Encoder {

        private Supplier<Integer> m_position;
        private int resetVal, lastPosition;
        private Direction direction;
        private double lastTimeStamp, veloEstimate, dpp, accel, lastVelo;

        /**
         * The encoder object for the motor.
         *
         * @param position the position supplier which just points to the
         *                 current position of the motor in ticks
         */
        public Encoder(Supplier<Integer> position) {
            m_position = position;
            dpp = 1;
            resetVal = 0;
            lastPosition = 0;
            veloEstimate = 0;
            direction = Direction.FORWARD;
            lastTimeStamp = (double) System.nanoTime() / 1E9;
        }

        /**
         * @return the current position of the encoder
         */
        public int getPosition() {
            int currentPosition = m_position.get();
            if (currentPosition != lastPosition) {
                double currentTime = (double) System.nanoTime() / 1E9;
                double dt = currentTime - lastTimeStamp;
                veloEstimate = (currentPosition - lastPosition) / dt;
                lastPosition = currentPosition;
                lastTimeStamp = currentTime;
            }
            return direction.getMultiplier() * currentPosition - resetVal;
        }

        /**
         * @return the distance traveled by the encoder
         */
        public double getDistance() {
            return dpp * getPosition();
        }

        /**
         * @return the velocity of the encoder adjusted to account for the distance per pulse
         */
        public double getRate() {
            return dpp * getVelocity();
        }

        /**
         * Resets the encoder without having to stop the motor.
         */
        public void reset() {
            resetVal += getPosition();
        }

        /**
         * Sets the distance per pulse of the encoder.
         *
         * @param distancePerPulse the desired distance per pulse (in units per tick)
         */
        public Encoder setDistancePerPulse(double distancePerPulse) {
            dpp = distancePerPulse;
            return this;
        }

        /**
         * Sets the direction of the encoder to forward or reverse
         *
         * @param direction the desired direction
         */
        public void setDirection(Direction direction) {
            this.direction = direction;
        }

        /**
         * @return the number of revolutions turned by the encoder
         */
        public double getRevolutions() {
            return getPosition() / getCPR();
        }

        /**
         * @return the raw velocity of the motor reported by the encoder
         */
        public double getRawVelocity() {
            double velo = getVelocity();
            if (velo != lastVelo) {
                double currentTime = (double) System.nanoTime() / 1E9;
                double dt = currentTime - lastTimeStamp;
                accel = (velo - lastVelo) / dt;
                lastVelo = velo;
                lastTimeStamp = currentTime;
            }
            return velo;
        }

        /**
         * @return the estimated acceleration of the motor in ticks per second squared
         */
        public double getAcceleration() {
            return accel;
        }

        private final static int CPS_STEP = 0x10000;

        /**
         * Corrects for velocity overflow
         *
         * @return the corrected velocity
         */
        public double getCorrectedVelocity() {
            double real = getRawVelocity();
            while (Math.abs(veloEstimate - real) > CPS_STEP / 2.0) {
                real += Math.signum(veloEstimate - real) * CPS_STEP;
            }
            return real;
        }

    }

    /**
     * The RunMode of the motor.
     */
    public enum RunMode {
        VelocityControl, PositionControl, RawPower
    }

    public enum ZeroPowerBehavior {
        UNKNOWN(DcMotor.ZeroPowerBehavior.UNKNOWN),
        BRAKE(DcMotor.ZeroPowerBehavior.BRAKE),
        FLOAT(DcMotor.ZeroPowerBehavior.FLOAT);

        private final DcMotor.ZeroPowerBehavior m_behavior;

        ZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
            m_behavior = behavior;
        }

        public DcMotor.ZeroPowerBehavior getBehavior() {
            return m_behavior;
        }
    }

    public DcMotor motor;
    public Encoder encoder;

    /**
     * The runmode of the motor
     */
    protected RunMode runmode;

    /**
     * The achievable ticks per second velocity of the motor
     */
    public double ACHIEVABLE_MAX_TICKS_PER_SECOND;

    /**
     * The motor type
     */
    protected GoBILDA type;

    private boolean targetIsSet = false;

    protected double bufferFraction = 0.9;

    public Motor() {
    }

    /**
     * Constructs the instance motor for the wrapper
     *
     * @param hMap the hardware map from the OpMode
     * @param id   the device id from the RC config
     */
    public Motor(@NonNull HardwareMap hMap, String id) {
        this(hMap, id, GoBILDA.NONE);
        ACHIEVABLE_MAX_TICKS_PER_SECOND = motor.getMotorType().getAchieveableMaxTicksPerSecond();
    }

    /**
     * Constructs the instance motor for the wrapper
     *
     * @param hMap        the hardware map from the OpMode
     * @param id          the device id from the RC config
     * @param gobildaType the type of gobilda 5202 series motor being used
     */
    public Motor(@NonNull HardwareMap hMap, String id, @NonNull GoBILDA gobildaType) {
        motor = hMap.get(DcMotor.class, id);
        encoder = new Encoder(motor::getCurrentPosition);

        runmode = RunMode.RawPower;
        type = gobildaType;

        ACHIEVABLE_MAX_TICKS_PER_SECOND = gobildaType.getAchievableMaxTicksPerSecond();
    }

    /**
     * Constructs an instance motor for the wrapper
     *
     * @param hMap the hardware map from the OpMode
     * @param id   the device id from the RC config
     * @param cpr  the counts per revolution of the motor
     * @param rpm  the revolutions per minute of the motor
     */
    public Motor(@NonNull HardwareMap hMap, String id, double cpr, double rpm) {
        this(hMap, id, GoBILDA.NONE);

        MotorConfigurationType type = motor.getMotorType().clone();
        type.setMaxRPM(rpm);
        type.setTicksPerRev(cpr);
        motor.setMotorType(type);

        ACHIEVABLE_MAX_TICKS_PER_SECOND = cpr * rpm / 60;
    }

    /**
     * Common method for setting the speed of a motor.
     *
     * @param output The percentage of power to set. Value should be between -1.0 and 1.0.
     */
    public void set(double output) {
            motor.setPower(output);

    }

    /**
     * Sets the distance per pulse of the encoder in units per tick.
     *
     * @param distancePerPulse the desired distance per pulse
     * @return an encoder an object with the specified distance per pulse
     */
    public Encoder setDistancePerPulse(double distancePerPulse) {
        return encoder.setDistancePerPulse(distancePerPulse);
    }

    /**
     * @return the distance traveled by the encoder
     */
    public double getDistance() {
        return encoder.getDistance();
    }

    public double getRevolutions() {
        return encoder.getRevolutions();
    }

    /**
     * @return the rate of the encoder
     */
    public double getRate() {
        return encoder.getRate();
    }

    /**
     * Resets the external encoder wrapper value.
     */
    public void resetEncoder() {
        encoder.reset();
    }

    /**
     * Resets the internal position of the motor.
     */
    public void stopAndResetEncoder() {
        encoder.resetVal = 0;
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * A wrapper method for the zero power behavior
     *
     * @param behavior the behavior desired
     */
    public void setZeroPowerBehavior(ZeroPowerBehavior behavior) {
        motor.setZeroPowerBehavior(behavior.getBehavior());
    }

    /**
     * @return the current position of the motor in ticks
     */
    public int getCurrentPosition() {
        return encoder.getPosition();
    }

    /**
     * @return the corrected velocity for overflow
     */
    public double getCorrectedVelocity() {
        return encoder.getCorrectedVelocity();
    }

    /**
     * @return the counts per revolution of the motor
     */
    public double getCPR() {
        return type == GoBILDA.NONE ? motor.getMotorType().getTicksPerRev() : type.getCPR();
    }

    /**
     * @return the max RPM of the motor
     */
    public double getMaxRPM() {
        return type == GoBILDA.NONE ? motor.getMotorType().getMaxRPM() : type.getRPM();
    }

    /**
     * Set the buffer for the motor. This adds a fractional value to the velocity control.
     *
     * @param fraction a fractional value between (0, 1].
     */
    public void setBuffer(double fraction) {
        if (fraction <= 0 || fraction > 1) {
            throw new IllegalArgumentException("Buffer must be between 0 and 1, exclusive to 0");
        }
        bufferFraction = fraction;
    }

    protected double getVelocity() {
        return ((DcMotorEx) motor).getVelocity();
    }

    /**
     * Common method for getting the current set speed of a motor.
     *
     * @return The current set speed. Value is between -1.0 and 1.0.
     */
    public double get() {
        return motor.getPower();
    }

    public double getVoltage() {
        return motor.getPower() * 12;
    }

    /**
     * Common method for inverting direction of a motor.
     *
     * @param isInverted The state of inversion true is inverted.
     */
    public void setInverted(boolean isInverted) {
        motor.setDirection(isInverted ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
    }

    /**
     * Common method for returning if a motor is in the inverted state or not.
     *
     * @return isInverted The state of the inversion true is inverted.
     */
    public boolean getInverted() {
        return DcMotor.Direction.REVERSE == motor.getDirection();
    }

    /**
     * Disable the motor.
     */
    @Override
    public void disable() {
        motor.close();
    }

    @Override
    public String getDeviceType() {
        return "Motor " + motor.getDeviceName() + " from " + motor.getManufacturer()
                + " in port " + motor.getPortNumber();
    }

    /**
     * Stops motor movement. Motor can be moved again by calling set without having to re-enable the
     * motor.
     */
    public void stopMotor() {
        motor.setPower(0);
    }

}
