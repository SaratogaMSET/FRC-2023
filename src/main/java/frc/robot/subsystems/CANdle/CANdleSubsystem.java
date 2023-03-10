package frc.robot.subsystems.CANdle;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.MathUtils;

public class CANdleSubsystem extends SubsystemBase {
    private static final CANdle candle1 = new CANdle(60);
    private static final CANdle candle2 = new CANdle(61);
    // private static final CANdle candle3 = new CANdle(62);
    // private static final CANdle candle4 = new CANdle(63);
    public static final Color black = new Color(0, 0, 0);

    // Game piece colors
    public static final Color yellow = new Color(242, 60, 0);
    public static final Color purple = new Color(184, 0, 185);

    // Indicator colors
    public static final Color white = new Color(255, 230, 220);
    public static final Color green = new Color(56, 209, 0);
    public static final Color blue = new Color(8, 32, 255);
    public static final Color red = new Color(227, 26, 0);

    public CANdleSubsystem() {
        CANdleConfiguration candleConfiguration = new CANdleConfiguration();
        candleConfiguration.statusLedOffWhenActive = true;
        candleConfiguration.disableWhenLOS = false;
        candleConfiguration.stripType = LEDStripType.RGB;
        candleConfiguration.brightnessScalar = 1.0;
        candleConfiguration.vBatOutputMode = VBatOutputMode.Modulated;
        candle1.configAllSettings(candleConfiguration, 100);
        candle2.configAllSettings(candleConfiguration, 100);
        // candle3.configAllSettings(candleConfiguration, 100);
        // candle4.configAllSettings(candleConfiguration, 100);
        setDefaultCommand(defaultCommand());
    }

    public void setColor(Color color){
        // LEDSegment.FrontLeftStrip.setColor(color);
        LEDSegment.BackRightStrip.setColor(color);
        LEDSegment.FrontRightStrip.setColor(color);
        // LEDSegment.BackLeftStrip.setColor(color);
    }
    public void setBrightness(double percent) {
        candle1.configBrightnessScalar(percent, 100);
        candle2.configBrightnessScalar(percent, 100);
        // candle3.configBrightnessScalar(percent, 100);
        // candle4.configBrightnessScalar(percent, 100);
    }

    public Command defaultCommand() {
        return runOnce(() -> {
            // LEDSegment.BatteryIndicator.fullClear();
            // LEDSegment.PressureIndicator.fullClear();
            // LEDSegment.MastEncoderIndicator.fullClear();
            // LEDSegment.BoomEncoderIndicator.fullClear();
            // LEDSegment.WristEncoderIndicator.fullClear();

            // LEDSegment.FrontLeftStrip.setColor(yellow);
            LEDSegment.BackRightStrip.setColor(yellow);
            LEDSegment.FrontRightStrip.setColor(yellow);
            // LEDSegment.BackLeftStrip.setColor(yellow);
            // LEDSegment.MainStrip.setColor(orange);

            // runDefaultMainStripAnimation();
        });
    }

    public Command clearSegmentCommand(LEDSegment segment) {
        return runOnce(() -> {
            segment.clearAnimation();
            segment.disableLEDs();
        });
    }

    public Command indicateConeCommand() {
        return buildSideStripCommand(() -> {
            LEDSegment.BackRightStrip.setColor(CANdleSubsystem.yellow);
            // LEDSegment.BackLeftStrip.setColor(CANdleSubsystem.yellow);
            // LEDSegment.FrontLeftStrip.setColor(CANdleSubsystem.yellow);
            LEDSegment.FrontRightStrip.setColor(CANdleSubsystem.yellow);
        });
    }

    public Command indicateCubeCommand() {
        return buildSideStripCommand(() -> {
            LEDSegment.BackRightStrip.setColor(CANdleSubsystem.purple);
            // LEDSegment.BackLeftStrip.setColor(CANdleSubsystem.purple);
            // LEDSegment.FrontLeftStrip.setColor(CANdleSubsystem.purple);
            LEDSegment.FrontRightStrip.setColor(CANdleSubsystem.purple);
        });
    }

    private Command buildSideStripCommand(Runnable runnable) {
        return startEnd(runnable, CANdleSubsystem::runDefaultSideAnimation);
        // return run(runnable).finallyDo((interrupted) -> {
        //     runDefaultSideAnimation();
        // });
    }

    // private Command buildTopStripCommand(Runnable runnable) {
    //     return startEnd(runnable, LightsSubsystem::runDefaultTopAnimation);
    //     // return run(runnable).finallyDo((interrupted) -> {
    //     //     runDefaultTopAnimation();
    //     // });
    // }

    // public static void runDefaultMainStripAnimation() {
    //     runDefaultSideAnimation();
    //     runDefaultTopAnimation();
    // }

    public static void runDefaultSideAnimation() {
        LEDSegment.BackRightStrip.setFadeAnimation(CANdleSubsystem.black, 0.5);
        // LEDSegment.BackLeftStrip.setFadeAnimation(CANdleSubsystem.black, 0.5);
        // LEDSegment.FrontLeftStrip.setFadeAnimation(CANdleSubsystem.black, 0.5);
        LEDSegment.FrontRightStrip.setFadeAnimation(CANdleSubsystem.black, 0.5);
    }

    // public static void runDefaultTopAnimation() {
    //     LEDSegment.BackTopStrip.setFadeAnimation(LightsSubsystem.orange, 0.5);
    //     LEDSegment.FrontTopStrip.setFadeAnimation(LightsSubsystem.orange, 0.5);
    // }

    public static enum LEDSegment {

        // BatteryIndicator(0, 2, 0),
        // PressureIndicator(2, 2, 1),
        // MastEncoderIndicator(4, 1, -1),
        // BoomEncoderIndicator(5, 1, -1),
        // WristEncoderIndicator(6, 1, -1),
        // DriverStationIndicator(7, 1, -1),

        BackRightStrip(8, 29, 2), // Looking from back to front (arm faces back)
        // BackTopStrip(62, 33, 3),
        // BackLeftStrip(8, 29, 4),
        // FrontLeftStrip(8, 29, 5),
        // FrontTopStrip(203, 33, 6),
        
        FrontRightStrip(8, 29, 7);
        // MainStrip(8, 296, 2);

        // 33 on top
        // 13 past on right side
        public final int startIndex;
        public final int segmentSize;
        public final int animationSlot;

        // private enum constructor
        private LEDSegment(int startIndex, int segmentSize, int animationSlot) {
            this.startIndex = startIndex;
            this.segmentSize = segmentSize;
            this.animationSlot = animationSlot;
        }

        public void setColor(Color color) {
            clearAnimation();
            candle1.setLEDs(color.red, color.green, color.blue, 0, startIndex, segmentSize);
            candle2.setLEDs(color.red, color.green, color.blue, 0, startIndex, segmentSize);
            // candle3.setLEDs(color.red, color.green, color.blue, 0, startIndex, segmentSize);
            // candle4.setLEDs(color.red, color.green, color.blue, 0, startIndex, segmentSize);
        }

        private void setAnimation(Animation animation) {
            candle1.animate(animation, animationSlot);
            candle2.animate(animation, animationSlot);
            // candle3.animate(animation, animationSlot);
            // candle4.animate(animation, animationSlot);
        }

        public void fullClear() {
            clearAnimation();
            disableLEDs();
        }

        public void clearAnimation() {
            candle1.clearAnimation(animationSlot);
            candle2.clearAnimation(animationSlot);
            // candle3.clearAnimation(animationSlot);
            // candle4.clearAnimation(animationSlot);
        }

        public void disableLEDs() {
            setColor(black);
        }

        public void setFlowAnimation(Color color, double speed) {
            setAnimation(new ColorFlowAnimation(
                    color.red, color.green, color.blue, 0, speed, segmentSize, Direction.Forward, startIndex));
        }

        public void setFadeAnimation(Color color, double speed) {
            setAnimation(
                    new SingleFadeAnimation(color.red, color.green, color.blue, 0, speed, segmentSize, startIndex));
        }

        public void setBandAnimation(Color color, double speed) {
            setAnimation(new LarsonAnimation(
                    color.red, color.green, color.blue, 0, speed, segmentSize, BounceMode.Front, 3, startIndex));
        }

        public void setStrobeAnimation(Color color, double speed) {
            setAnimation(new StrobeAnimation(color.red, color.green, color.blue, 0, speed, segmentSize, startIndex));
        }

        public void setRainbowAnimation(double speed) {
            setAnimation(new RainbowAnimation(1, speed, segmentSize, false, startIndex));
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
}