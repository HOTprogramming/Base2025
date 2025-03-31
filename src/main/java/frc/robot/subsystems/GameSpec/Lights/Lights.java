package frc.robot.subsystems.GameSpec.Lights;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.subsystems.GameSpec.Lights.LEDsConstants.*;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;

public class Lights extends SubsystemBase {
    private final CANdle leftCANdle = new CANdle(LEFT_ID, "rio");
    private final CANdle rightCANdle = new CANdle(RIGHT_ID, "rio");

    int repeats;

    private Animation m_toAnimate = null;

    public enum AnimationTypes {
        ColorFlow,
        Fire,
        Larson,
        Rainbow,
        RgbFade,
        SingleFade,
        Strobe,
        Twinkle,
        TwinkleOff,
        AutoAlign,
        SetAll
    }
    private AnimationTypes m_currentAnimation;

    public Lights() {
        changeAnimation(AnimationTypes.SetAll);
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = false;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.RGB;
        configAll.brightnessScalar = 1;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        leftCANdle.configAllSettings(configAll);
        rightCANdle.configAllSettings(configAll);
    }

    public void incrementAnimation() {
        switch(m_currentAnimation) {
            case ColorFlow: changeAnimation(AnimationTypes.Fire); break;
            case Fire: changeAnimation(AnimationTypes.Larson); break;
            case Larson: changeAnimation(AnimationTypes.Rainbow); break;
            case Rainbow: changeAnimation(AnimationTypes.RgbFade); break;
            case RgbFade: changeAnimation(AnimationTypes.SingleFade); break;
            case SingleFade: changeAnimation(AnimationTypes.Strobe); break;
            case Strobe: changeAnimation(AnimationTypes.Twinkle); break;
            case Twinkle: changeAnimation(AnimationTypes.TwinkleOff); break;
            case TwinkleOff: changeAnimation(AnimationTypes.ColorFlow); break;
            case SetAll: changeAnimation(AnimationTypes.ColorFlow); break;
            case AutoAlign: changeAnimation(AnimationTypes.Strobe); break;
          default:
            break;
        }
    }
    public void decrementAnimation() {
        switch(m_currentAnimation) {
            case ColorFlow: changeAnimation(AnimationTypes.TwinkleOff); break;
            case Fire: changeAnimation(AnimationTypes.ColorFlow); break;
            case Larson: changeAnimation(AnimationTypes.Fire); break;
            case Rainbow: changeAnimation(AnimationTypes.Larson); break;
            case RgbFade: changeAnimation(AnimationTypes.Rainbow); break;
            case SingleFade: changeAnimation(AnimationTypes.RgbFade); break;
            case Strobe: changeAnimation(AnimationTypes.SingleFade); break;
            case Twinkle: changeAnimation(AnimationTypes.Strobe); break;
            case TwinkleOff: changeAnimation(AnimationTypes.Twinkle); break;
            case SetAll: changeAnimation(AnimationTypes.ColorFlow); break;
            case AutoAlign: changeAnimation(AnimationTypes.Strobe); break;
          default:
            break;
        }
    }
    public void setColors() {
        changeAnimation(AnimationTypes.SetAll);
    }

    public Command changeAnimation(AnimationTypes toChange) {
      return runOnce(() -> {
        m_currentAnimation = toChange;
        switch(toChange)
        {
            case ColorFlow:
                m_toAnimate = new ColorFlowAnimation(128, 20, 70, 0, 0.7, LEDS_PER_ANIMATION, Direction.Forward);
                break;
            case Fire:
                m_toAnimate = new FireAnimation(0.5, 0.7, LEDS_PER_ANIMATION, 0.7, 0.5);
                break;
            case Larson:
                m_toAnimate = new LarsonAnimation(0, 255, 46, 0, 1, LEDS_PER_ANIMATION, BounceMode.Front, 3, RightStart);
                break;
            case Rainbow:
                m_toAnimate = new RainbowAnimation(1, 5, LEDS_PER_ANIMATION);
                break;
            case RgbFade:
                m_toAnimate = new RgbFadeAnimation(0.7, 0.4, LEDS_PER_ANIMATION);
                break;
            case SingleFade:
                m_toAnimate = new SingleFadeAnimation(50, 2, 200, 0, 0.5, LEDS_PER_ANIMATION);
                break;
            case Strobe:
                m_toAnimate = new StrobeAnimation(240, 10, 180, 0, 0.5, LEDS_PER_ANIMATION);
                break;
            case Twinkle:
                m_toAnimate = new TwinkleAnimation(30, 70, 60, 0, 0.4, LEDS_PER_ANIMATION, TwinklePercent.Percent6, RightStart);
                break;
            case TwinkleOff:
                m_toAnimate = new TwinkleOffAnimation(70, 90, 175, 0, 0.8, LEDS_PER_ANIMATION, TwinkleOffPercent.Percent100);
                break;
            case AutoAlign:
                m_toAnimate = new StrobeAnimation(0, 255, 0, 0, 5, LEDS_PER_ANIMATION, RightStart);
            case SetAll:
                m_toAnimate = null;
                break;
        }

        rightCANdle.animate(m_toAnimate);
        leftCANdle.animate(m_toAnimate);

        System.out.println("Changed to " + m_currentAnimation.toString());
    });
  } 

    public Command animate() {
      return runOnce(() -> {
        rightCANdle.animate(m_toAnimate);
        leftCANdle.animate(m_toAnimate);
      });
    }

    @Override
    public void periodic() {

    }

    public void leftCameraLights(int r, int g, int b) {
      leftCANdle.setLEDs(r, g, b, 0, 8, 10);
    }

    public void rightCameraLights(int r, int g, int b) {
      rightCANdle.setLEDs(r, g, b, 0, 8, 10);
    }

    public Command autoAlignLights() {
      return runOnce(() -> changeAnimation(AnimationTypes.AutoAlign));
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

  public Command blink(AnimationTypes anim, int repeats, double time) { 
    return Commands.sequence(changeAnimation(anim), Commands.waitSeconds(time), changeAnimation(AnimationTypes.SetAll), Commands.waitSeconds(time));
  }
    
  public Command setGreen(){
   return runOnce(()-> {
      rightCANdle.setLEDs(0, 255, 0, 0, RightStart, count); 
      leftCANdle.setLEDs(0, 255, 0, 0, RightStart, count); 
    });
  }
  
  public Command setRed(){
    return runOnce(()-> {
      rightCANdle.setLEDs(225, 0, 0, 0, RightStart, count);
      leftCANdle.setLEDs(255, 0, 0, 0, LeftStart, count); 
    });
   }
  
   public Command setYellow(){
    return runOnce(()-> {
      rightCANdle.setLEDs(225, 150, 0, 0, RightStart, count);
      leftCANdle.setLEDs(225, 150, 0, 0, LeftStart, count);
    });
   }  

   public Command setPurple(){
    return runOnce(()-> {
      rightCANdle.setLEDs(0, 150, 255, 0, RightStart, count);
      leftCANdle.setLEDs(0, 150, 255, 0, LeftStart, count);
    });
   }  

   public Command setTeal(){
    return runOnce(()-> {
      rightCANdle.setLEDs(0, 128, 128, 0, RightStart, count);
      leftCANdle.setLEDs(0, 128, 128, 0, LeftStart, count);
    });
   }  

   public Command setOne(int ledNum, boolean good){
    return runOnce(()-> {
      rightCANdle.setLEDs(good ? 0 : 128, good ? 128 : 0, 0, 0, ledNum, 1);
    
    });
   }  

   }



