package frc.robot.subsystems.CANdle;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.MathUtils;

public class CANdleSubsystem extends SubsystemBase {
    private static final CANdle candle1 = new CANdle(59, "649-Hammerhead-CANivore"); // Front Left
    private static final CANdle candle2 = new CANdle(60, "649-Hammerhead-CANivore"); // Front Right
    private static final CANdle candle3 = new CANdle(61, "649-Hammerhead-CANivore"); // Back Left
    private static final CANdle candle4 = new CANdle(62, "649-Hammerhead-CANivore"); // Back Right
    public static final Color black = new Color(0, 0, 0);

    // Game piece colors
    private static final Color yellow = new Color(242, 60, 0);
    private static final Color purple = new Color(184, 0, 185);

    public CANdleSubsystem() {
        CANdleConfiguration candleConfiguration = new CANdleConfiguration();
        candleConfiguration.statusLedOffWhenActive = true;
        candleConfiguration.disableWhenLOS = false;
        candleConfiguration.stripType = LEDStripType.RGB;
        candleConfiguration.brightnessScalar = 1.0;
        candleConfiguration.vBatOutputMode = VBatOutputMode.Modulated;
        candle1.configAllSettings(candleConfiguration, 400);
        candle2.configAllSettings(candleConfiguration, 400);
        candle3.configAllSettings(candleConfiguration, 400);
        candle4.configAllSettings(candleConfiguration, 400);
    }

    public void setColor(Color color) {
        LEDSegment.FrontLeftStrip.setColor(color);
        LEDSegment.BackRightStrip.setColor(color);
        LEDSegment.FrontRightStrip.setColor(color);
        LEDSegment.BackLeftStrip.setColor(color);
    }

    public void setBrightness(double percent) {
        candle1.configBrightnessScalar(percent, 100);
        candle2.configBrightnessScalar(percent, 100);
        candle3.configBrightnessScalar(percent, 100);
        candle4.configBrightnessScalar(percent, 100);
    }

    public void indicateConeNoStrobe() {
        LEDSegment.BackRightStrip.setColor(yellow);
        LEDSegment.BackLeftStrip.setColor(yellow);
        LEDSegment.FrontLeftStrip.setColor(yellow);
        LEDSegment.FrontRightStrip.setColor(yellow);
    }

    public void indicateConeStrobe() {
        LEDSegment.FrontLeftStrip.setStrobeAnimation(yellow, 0.5);
        LEDSegment.FrontRightStrip.setStrobeAnimation(yellow, 0.5);
        LEDSegment.BackRightStrip.setStrobeAnimation(yellow, 0.5);
        LEDSegment.BackLeftStrip.setStrobeAnimation(yellow, 0.5);
    }

    public void indicateCubeStrobe() {
        LEDSegment.FrontLeftStrip.setStrobeAnimation(purple, 0.5);
        LEDSegment.FrontRightStrip.setStrobeAnimation(purple, 0.5);
        LEDSegment.BackRightStrip.setStrobeAnimation(purple, 0.5);
        LEDSegment.BackLeftStrip.setStrobeAnimation(purple, 0.5);
    }

    public void indicateCubeNoStrobe(){
        LEDSegment.BackRightStrip.setColor(purple);
        LEDSegment.BackLeftStrip.setColor(purple);
        LEDSegment.FrontLeftStrip.setColor(purple);
        LEDSegment.FrontRightStrip.setColor(purple);
    }

    public static enum LEDSegment {

        FrontLeftStrip(8, 30, 3, candle1),
        FrontRightStrip(8, 30, 7, candle2),
        BackLeftStrip(8, 30, 8, candle3),
        BackRightStrip(8, 30, 2, candle4);

        // MainStrip(8, 296, 2);

        public final int startIndex;
        public final int segmentSize;
        public final int animationSlot;
        public final CANdle candle;

        // private enum constructor
        private LEDSegment(int startIndex, int segmentSize, int animationSlot, CANdle candle) {
            this.startIndex = startIndex;
            this.segmentSize = segmentSize;
            this.animationSlot = animationSlot;
            this.candle = candle;
        }

        public void setColor(Color color) {
            clearAnimation();
            candle.setLEDs(color.red, color.green, color.blue, 0, startIndex, segmentSize);
        }

        public void setColor(Color color, int index) {
            clearAnimation();
            candle.setLEDs(color.red, color.green, color.blue, 0, index, 1);
        }

        public void setColor(Color color, int index, int count) {
            clearAnimation();
            candle.setLEDs(color.red, color.green, color.blue, 0, index, count);
        }

        private void setAnimation(Animation animation) {
            candle.animate(animation, animationSlot);
        }

        public void fullClear() {
            clearAnimation();
            disableLEDs();
        }

        public void clearAnimation() {
            candle.clearAnimation(animationSlot);
        }

        public void disableLEDs() {
            setColor(black);
        }

        public void setStrobeAnimation(Color color, double speed) {
            setAnimation(new StrobeAnimation(color.red, color.green, color.blue, 0, speed, segmentSize, startIndex));
        }
    }

    public static class Color {
        public int red;
        public int green;
        public int blue;

        public Color(int red, int green, int blue) {
            this.red = red;
            this.green = green;
            this.blue = blue;
        }

        public Color(Color color) {
            this.red = color.red;
            this.green = color.green;
            this.blue = color.blue;
        }

        /**
         * Highly imperfect way of dimming the LEDs. It does not maintain color or
         * accurately adjust perceived brightness.
         *
         * @param dimFactor
         * @return The dimmed color
         */
        public Color dim(double dimFactor) {
            int newRed = (int) (MathUtils.ensureRange(red * dimFactor, 0, 200));
            int newGreen = (int) (MathUtils.ensureRange(green * dimFactor, 0, 200));
            int newBlue = (int) (MathUtils.ensureRange(blue * dimFactor, 0, 200));

            return new Color(newRed, newGreen, newBlue);
        }
    }

    @Override
    public void periodic() {
    }
}