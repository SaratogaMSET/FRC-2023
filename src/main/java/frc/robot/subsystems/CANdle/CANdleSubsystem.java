package frc.robot.subsystems.CANdle;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdleControlFrame;
import com.ctre.phoenix.led.CANdleStatusFrame;
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
    private static final CANdle candle1 = new CANdle(59, "649-Hammerhead-CANivore"); //Front Left
    private static final CANdle candle2 = new CANdle(60, "649-Hammerhead-CANivore"); //Front Right
    private static final CANdle candle3 = new CANdle(61, "649-Hammerhead-CANivore"); //Back Left
    private static final CANdle candle4 = new CANdle(62, "649-Hammerhead-CANivore"); //Back Right
    public static final Color black = new Color(0, 0, 0);
    private static double prev = 0.0;
   

    // Game piece colors
    private static final Color yellow = new Color(242, 60, 0);
    private static final Color purple = new Color(184, 0, 185);

    // Indicator colors
    private static final Color white = new Color(255, 230, 220);
    private static final Color green = new Color(56, 209, 0);
    private static final Color blue = new Color(8, 32, 255);
    private static final Color red = new Color(227, 26, 0);

    private static Color color = blue;

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

        candle1.setControlFramePeriod(CANdleControlFrame.CANdle_Control_2_ModulatedVBatOut, 300);
        candle2.setControlFramePeriod(CANdleControlFrame.CANdle_Control_2_ModulatedVBatOut, 300);
        candle3.setControlFramePeriod(CANdleControlFrame.CANdle_Control_2_ModulatedVBatOut, 300);
        candle4.setControlFramePeriod(CANdleControlFrame.CANdle_Control_2_ModulatedVBatOut, 300);

        candle1.setStatusFramePeriod(CANdleStatusFrame.CANdleStatusFrame_Status_1_General, 30000);
        candle2.setStatusFramePeriod(CANdleStatusFrame.CANdleStatusFrame_Status_1_General, 30000);
        candle3.setStatusFramePeriod(CANdleStatusFrame.CANdleStatusFrame_Status_1_General, 30000);
        candle4.setStatusFramePeriod(CANdleStatusFrame.CANdleStatusFrame_Status_1_General, 30000);
    }

    public void setColor(Color color){
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

    // public Command defaultCommand() {
    //     return runOnce(() -> {
    //         // LEDSegment.BatteryIndicator.fullClear();
    //         // LEDSegment.PressureIndicator.fullClear();
    //         // LEDSegment.MastEncoderIndicator.fullClear();
    //         // LEDSegment.BoomEncoderIndicator.fullClear();
    //         // LEDSegment.WristEncoderIndicator.fullClear();

    //         // LEDSegment.FrontLeftStrip.setColor(yellow);
    //         LEDSegment.BackRightStrip.setColor(yellow);
    //         LEDSegment.FrontRightStrip.setColor(yellow);
    //         // LEDSegment.BackLeftStrip.setColor(yellow);
    //         // LEDSegment.MainStrip.setColor(orange);

    //         // runDefaultMainStripAnimation();
    //     });
    // }

    public Command clearSegmentCommand(LEDSegment segment) {
        return runOnce(() -> {
            segment.clearAnimation();
            segment.disableLEDs();
        });
    }

    public Command indicateConeCommand() {
        CANdleSubsystem.color = CANdleSubsystem.yellow;
        CANdleSubsystem.prev = 0.0;
        // LEDSegment.BackLeftStrip.clearAnimation();
        // LEDSegment.BackRightStrip.clearAnimation();
        // LEDSegment.BackRightStrip.clearAnimation();
        // LEDSegment.FrontLeftStrip.clearAnimation();
        final Color newColor = new Color(CANdleSubsystem.color);
        return buildSideStripCommand(() -> {
            
            LEDSegment.BackRightStrip.setColor(newColor);
            LEDSegment.BackLeftStrip.setColor(newColor);
            LEDSegment.FrontLeftStrip.setFlowAnimation(newColor, 0.5);
            LEDSegment.FrontRightStrip.setFlowAnimation(newColor, 0.5);
        });
    }
    public Command indicateActiveSide(){
        LEDSegment.BackLeftStrip.clearAnimation();
        LEDSegment.BackRightStrip.clearAnimation();
        LEDSegment.FrontRightStrip.clearAnimation();
        LEDSegment.FrontLeftStrip.clearAnimation();

        LEDSegment.BackLeftStrip.disableLEDs();
        LEDSegment.BackRightStrip.disableLEDs();
        LEDSegment.FrontRightStrip.disableLEDs();
        LEDSegment.FrontLeftStrip.disableLEDs();
        final double newPrev = prev;
        // if(newPrev == 1.0){
            // final Color newColor = new Color(purple);
            return buildSideStripCommand(() -> {
            
                LEDSegment.BackRightStrip.setFlowAnimation(blue, 0.5);
                LEDSegment.BackLeftStrip.setFlowAnimation(blue, 0.5);
                LEDSegment.FrontLeftStrip.setColor(blue);
                LEDSegment.FrontRightStrip.setColor(blue);
 
            // });
        });
        // else{
        //     final Color newColor = new Color(yellow);
        //     return buildSideStripCommand(() -> {
            
        //         LEDSegment.BackRightStrip.setFlowAnimation(newColor, 0.5);
        //         LEDSegment.BackLeftStrip.setFlowAnimation(newColor, 0.5);
        //         LEDSegment.FrontLeftStrip.setColor(newColor);
        //         LEDSegment.FrontRightStrip.setColor(newColor);
 
        //     });
        // }
    }
    public Command indicateCubeCommand() {
        CANdleSubsystem.color = CANdleSubsystem.purple;
        CANdleSubsystem.prev = 1.0;

        // LEDSegment.BackLeftStrip.clearAnimation();
        // LEDSegment.BackRightStrip.clearAnimation();
        // LEDSegment.BackRightStrip.clearAnimation();
        // LEDSegment.FrontLeftStrip.clearAnimation();
        
        final Color newColor = new Color(CANdleSubsystem.color);
        return buildSideStripCommand(() -> {
            // System.out.println("Set Color to purple");
            // SmartDashboard.putNumberArray("Color On Cube Command", new double[]{CANdleSubsystem.color.red, CANdleSubsystem.color.green, CANdleSubsystem.color.blue});
            
            LEDSegment.BackRightStrip.setColor(newColor);
            LEDSegment.BackLeftStrip.setColor(newColor);
            LEDSegment.FrontLeftStrip.setFlowAnimation(newColor, 0.5);
            LEDSegment.FrontRightStrip.setFlowAnimation(newColor, 0.5);
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
        LEDSegment.BackRightStrip.setColor(CANdleSubsystem.color);
        LEDSegment.BackLeftStrip.setColor(CANdleSubsystem.color);
        LEDSegment.FrontLeftStrip.setFlowAnimation(CANdleSubsystem.color, 0.5);
        LEDSegment.FrontRightStrip.setFlowAnimation(CANdleSubsystem.color, 0.5);
    }

    // public static void runDefaultTopAnimation() {
    //     LEDSegment.BackTopStrip.setFadeAnimation(LightsSubsystem.orange, 0.5);
    //     LEDSegment.FrontTopStrip.setFadeAnimation(LightsSubsystem.orange, 0.5);
    // }

    public static enum LEDSegment {

        FrontLeftStrip(8, 30, 3, candle1),
        FrontRightStrip(8, 30, 7, candle2),
        BackLeftStrip(8, 30, 7, candle3),
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

        public Color(Color color){
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
        // SmartDashboard.putNumberArray("Color", new double[]{color.red, color.green, color.blue});
    }
}