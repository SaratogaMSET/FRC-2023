package frc.robot.util.drivers;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;

import edu.wpi.first.wpilibj.Timer;

/**
 * Creates CANTalon objects and configures all the parameters we care about to factory defaults. Closed-loop and sensor
 * parameters are not set, as these are expected to be set by the application.
 */
public class CANCoderFactory {

    public static final CANCoderConfiguration kDefaultConfig = new CANCoderConfiguration();
    static {
        // TODO flesh out default configuration for absolute encoder
    }

    public static CANCoder createDefaultCANCoder(int id) {
        return createCANCoder(id, kDefaultConfig);
    }

    public static CANCoder createCANCoder(int id, CANCoderConfiguration config) {
        // Delay for CAN bus bandwidth to clear up.
        Timer.delay(0.25);
        CANCoder encoder = new CANCoder(id);
        encoder.configFactoryDefault();

        encoder.configAllSettings(config);
        return encoder;
    }
}