package frc.robot.subsystems.Lights;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode; 
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public class Lights implements Subsystem {
    
    CANdle candleLight;
          private int start = 0;
          private int start2 = 5; 
          private int start3 = 10;
          private int start4 = 15;
          private int startOne = 1;
          private int startTwo = 2;
          private int startThree = 3;
          private int startFour = 4;
          private int startFive = 5;
          private int startSix = 6;
          private int startSeven = 7;
          private int startEight = 8;
          private int startNine = 9;
          private int startTen = 10;
          private int start11 = 11;
          private int start12 = 12;
          private int start13 = 13;
          private int start14 = 14;
          private int start15 = 15;
          private int start16 = 16;
          private int start17 = 17;
          private int count = 5;
          private int countAll = 20;
          private int countOne = 1;



    
    public Lights (){      
        CANdleConfiguration configAll = new CANdleConfiguration();
       // changeAnimation(AnimationTypes.SetAll); 
         candleLight = new CANdle(51, "rio");
         
        configAll.statusLedOffWhenActive = false;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.RGB;
        configAll.brightnessScalar = 0.5;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        candleLight.configAllSettings(configAll);}
        private final int LedCount = 300;
        private Animation m_toAnimate = null;
        private final int LEDS_PER_ANIMATION = 30;
        private int m_candleChannel = 0;
        private boolean m_clearAllAnims = false;
        private boolean m_last5V = false;
        private boolean m_animDirection = false;
        private boolean m_setAnim = false;
       public Boolean helpfull;
  
    








  public Command setGreen(){
   return runOnce(()-> {candleLight.setLEDs(0, 255, 0, 0, start, count); });
    //return  candleLight.setLEDs(0, 255, 0, 0, start, count); //green
  }
  public Command setGreen2(){
    return runOnce(()-> {candleLight.setLEDs(0, 255, 0, 0, start2, count); });
     //return  candleLight.setLEDs(0, 255, 225, 0, start, count); //green
   }
   public Command setGreen3(){
    return runOnce(()-> {candleLight.setLEDs(0, 255, 0, 0, start3, count); });
     //return  candleLight.setLEDs(0, 255, 0, 0, start, count); //green
   }
   public Command setGreen4(){
    return runOnce(()-> {candleLight.setLEDs(0, 255, 0, 0, start4, count); });
     //return  candleLight.setLEDs(0, 255, 0, 0, start, count); //green
   }
   public Command setGreenAll(){
    return runOnce(()-> {candleLight.setLEDs(0, 255, 0, 0, start, countAll); });
     //return  candleLight.setLEDs(0, 255, 0, 0, start, count); //green
   }
   
     
  
  
  public Command setRed(){
    return runOnce(()-> {candleLight.setLEDs(225, 0, 0, 0, start, count); });
    
   }//red
   public Command setRed2(){
    return runOnce(()-> {candleLight.setLEDs(225, 0, 0, 0, start2, count); });
    
   }//red
   public Command setRed3(){
    return runOnce(()-> {candleLight.setLEDs(225, 0, 0, 0, start3, count); });
    
   }//red
   public Command setRed4(){
    return runOnce(()-> {candleLight.setLEDs(225, 0, 0, 0, start4, count); });
    
   }//red
   public Command setRedAll(){
    return runOnce(()-> {candleLight.setLEDs(225, 0, 0, 0, start, countAll); });
    
   }//red
   
   
   public Command setYellow(){
    return runOnce(()-> {candleLight.setLEDs(225, 150, 0, 0, start, count); });
   
   }//yellow
   public Command setYellow2(){
    return runOnce(()-> {candleLight.setLEDs(225, 150, 0, 0, start2, count); });
   
   }//yellow
   public Command setYellow3(){
    return runOnce(()-> {candleLight.setLEDs(225, 150, 0, 0, start3, count); });
   
   }//yellow
   public Command setYellow4(){
    return runOnce(()-> {candleLight.setLEDs(225, 150, 0, 0, start4, count); });
   
   }//yellow
   public Command setYellowAll(){
    return runOnce(()-> {candleLight.setLEDs(225, 150, 0, 0, start, countAll); });
   
   }//yellow
   
   
   public Command setWhite(){
    return runOnce(()-> {candleLight.setLEDs(225, 255, 225, 0, start, count); });
   }//white
   public Command setWhite2(){
    return runOnce(()-> {candleLight.setLEDs(225, 255, 225, 0, start2, count); });
   }//white
   public Command setWhite3(){
    return runOnce(()-> {candleLight.setLEDs(225, 255, 225, 0, start3, count); });
   }//white
   public Command setWhite4(){
    return runOnce(()-> {candleLight.setLEDs(225, 255, 225, 0, start4, count); });
   }//white
   public Command setWhiteAll(){
    return runOnce(()-> {candleLight.setLEDs(225, 255, 225, 0, start, countAll); });
   }//white



   public Command setBlue(){
    return runOnce(()-> {candleLight.setLEDs(0, 0, 225, 0, start, count); });
    //blue 
    }
    public Command setBlue2(){
        return runOnce(()-> {candleLight.setLEDs(0, 0, 225, 0, start2, count); });
        //blue    
    }
public Command setBlue3(){ 
    return runOnce(()-> {candleLight.setLEDs(0, 0, 225, 0, start3, count); });
            //blue
   }
   public Command setBlue4(){
    return runOnce(()-> {candleLight.setLEDs(0, 0, 225, 0, start4, count); });
    //blue
   }public Command setBlueAll(){
    return runOnce(()-> {candleLight.setLEDs(0, 0, 225, 0, start, countAll); });
    //blue
   }



   public Command setOff(){
    return runOnce(()-> {candleLight.setLEDs(0, 0, 0, 0, start, countAll); });
    //off?
   }
   public Command setPurple(){
    return runOnce(()-> {candleLight.setLEDs(170, 0, 255, 0, start, count); });
    //purple
   }
   public Command setPurple2(){
    return runOnce(()-> {candleLight.setLEDs(170, 0, 255, 0, start2, count); });
    //purple
   }  
   public Command setPurple3(){
    return runOnce(()-> {candleLight.setLEDs(170, 0, 255, 0, start3, count); });
    //purple
   }  
   public Command setPurple4(){
    return runOnce(()-> {candleLight.setLEDs(170, 0, 255, 0, start4, count); });
    //purple
   }  
   public Command setPurpleAll(){
    return runOnce(()-> {candleLight.setLEDs(170, 0, 255, 0, start, countAll); });
    //purple
   }  
     
   
   public Command setOrange(){
    return runOnce(()-> {candleLight.setLEDs(255, 110, 0, 0, start, count); });
    //orange
   }
   public Command setOrange2(){
    return runOnce(()-> {candleLight.setLEDs(255, 110, 0, 0, start2, count); });
    //orange
   }
   public Command setOrange3(){
    return runOnce(()-> {candleLight.setLEDs(255, 110, 0, 0, start3, count); });
    //orange
   }
   public Command setOrange4(){
    return runOnce(()-> {candleLight.setLEDs(255, 110, 0, 0, start4, count); });
    //orange
   }
   public Command setOrangeAll(){
    return runOnce(()-> {candleLight.setLEDs(255, 110, 0, 0, start, countAll); });
    //orange
   }


   public Command setAwsomeRed(){
    return runOnce(()-> {candleLight.setLEDs(225, 50, 0, 0, start, count); });
    //awsome red
   }
   public Command setAwsomeRed2(){
    return runOnce(()-> {candleLight.setLEDs(225, 50, 0, 0, start2, count); });
    //awsome red
   }
   public Command setAwsomeRed3(){
    return runOnce(()-> {candleLight.setLEDs(225, 50, 0, 0, start3, count); });
    //awsome red
   }
   public Command setAwsomeRed4(){
    return runOnce(()-> {candleLight.setLEDs(225, 50, 0, 0, start4, count); });
    //awsome red
   }
   public Command setAwsomeRedAll(){
    return runOnce(()-> {candleLight.setLEDs(225, 50, 0, 0, start, countAll); });
    //awsome red
   }

   public Command setTeal(){
    return runOnce(()-> {candleLight.setLEDs(0, 255, 120, 0, start, count); });
    //its teal dude
   }
   public Command setTeal2(){
    return runOnce(()-> {candleLight.setLEDs(0, 255, 120, 0, start2, count); });
    //its teal dude
   }
   public Command setTeal3(){
    return runOnce(()-> {candleLight.setLEDs(0, 255, 120, 0, start3, count); });
    //its teal dude
   }
   public Command setTeal4(){
    return runOnce(()-> {candleLight.setLEDs(0, 255, 120, 0, start4, count); });
    //its teal dude
   }
   public Command setTealAll(){
    return runOnce(()-> {candleLight.setLEDs(0, 255, 120, 0, start, countAll); });
    //its teal dude
   }
   
   
   
   
   
   
   public Command setTealOne(){
    return runOnce(()-> {candleLight.setLEDs(0, 255, 120, 0, startOne, countOne); });
    //its teal dude
   }

   public Command setTealThree(){
    return runOnce(()-> {candleLight.setLEDs(0, 255, 120, 0, startThree, countOne); });
    //its teal dude
   }
   public Command setTealFive(){
    return runOnce(()-> {candleLight.setLEDs(0, 255, 120, 0, startFive, countOne); });
    //its teal bro
   }
   public Command setTealSeven(){
    return runOnce(()-> {candleLight.setLEDs(0, 255, 120, 0, startSeven, countOne); });
    //its teal dude
   }
   public Command setTealNine(){
    return runOnce(()-> {candleLight.setLEDs(0, 255, 120, 0, startNine, countOne); });
    //its teal dudep
   }
   public Command setTeal11(){
    return runOnce(()-> {candleLight.setLEDs(0, 255, 120, 0, start11, countOne); });
    //its teal brosky
   }
   public Command setAwsomeRedTwo(){
    return runOnce(()-> {candleLight.setLEDs(225, 50, 0, 0, startTwo, count); });
    //nice red noble
   }
   public Command setAwsomeRedFour(){
    return runOnce(()-> {candleLight.setLEDs(225, 50, 0, 0, startFour, count); });
    //nice red noble
   }
   public Command setAwsomeRedSix(){
    return runOnce(()-> {candleLight.setLEDs(225, 50, 0, 0, startSix, count); });
    //nice red noble
   }
   public Command setAwsomeRedEight(){
    return runOnce(()-> {candleLight.setLEDs(225, 50, 0, 0, startEight, count); });
    //nice red noble
   }
   public Command setAwsomeRedTen(){
    return runOnce(()-> {candleLight.setLEDs(225, 50, 0, 0, startTen, count); });
    //nice red noble
   }
   
   
   public Command setTealTwo(){
    return runOnce(()-> {candleLight.setLEDs(225, 50, 0, 0, startTwo, count); });
    //nice red noble
   }
   public Command setTealFour(){
    return runOnce(()-> {candleLight.setLEDs(225, 50, 0, 0, startFour, count); });
    //nice red noble
   }
   public Command setTealSix(){
    return runOnce(()-> {candleLight.setLEDs(225, 50, 0, 0, startSix, count); });
    //nice red noble
   }
   public Command setTealEight(){
    return runOnce(()-> {candleLight.setLEDs(225, 50, 0, 0, startEight, count); });
    //nice red noble
   }
   public Command setTealTen(){
    return runOnce(()-> {candleLight.setLEDs(225, 50, 0, 0, startTen, count); });
    //nice red noble
   }
   
   
   public Command setAwsomeRedOne(){
    return runOnce(()-> {candleLight.setLEDs(0, 255, 120, 0, startOne, countOne); });
    //its teal dude
   }

   public Command setAwsomeRedThree(){
    return runOnce(()-> {candleLight.setLEDs(0, 255, 120, 0, startThree, countOne); });
    //its teal dude
   }
   public Command setAwsomeRedFive(){
    return runOnce(()-> {candleLight.setLEDs(0, 255, 120, 0, startFive, countOne); });
    //its teal bro
   }
   public Command setAwsomeRedSeven(){
    return runOnce(()-> {candleLight.setLEDs(0, 255, 120, 0, startSeven, countOne); });
    //its teal dude
   }
   public Command setAwsomeRedNine(){
    return runOnce(()-> {candleLight.setLEDs(0, 255, 120, 0, startNine, countOne); });
    //its teal dudep
   }
   public Command setAwsomeRed11(){
    return runOnce(()-> {candleLight.setLEDs(0, 255, 120, 0, start11, countOne); });
    //its teal brosky
   }


   













   }


//idk how to make blink

  
        
    




























