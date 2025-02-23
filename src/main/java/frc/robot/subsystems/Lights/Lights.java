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

import static frc.robot.subsystems.Lights.LEDsConstants.*;

public class Lights implements Subsystem {
    
    CANdle rightCANdle;
    CANdle leftCANdle;
          



    
    public Lights (){      
        CANdleConfiguration configAll = new CANdleConfiguration();
       // changeAnimation(AnimationTypes.SetAll); 
        rightCANdle = new CANdle(55, "rio");
        leftCANdle = new CANdle(54, "rio"); 

        configAll.statusLedOffWhenActive = false;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.RGB;
        configAll.brightnessScalar = 0.5;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        rightCANdle.configAllSettings(configAll);}
        private Animation m_toAnimate = null;
        

  public Command setGreen(){
   return runOnce(()-> {
      rightCANdle.setLEDs(0, 255, 0, 0, start, count); 
      leftCANdle.setLEDs(0, 255, 0, 0, start, count); 

    });
    //return  rightCANdle.setLEDs(0, 255, 0, 0, start, count); //green
  }
  public Command setGreen2(){
    return runOnce(()-> {rightCANdle.setLEDs(0, 255, 0, 0, start2, count); });
     //return  rightCANdle.setLEDs(0, 255, 225, 0, start, count); //green
   }
   public Command setGreen3(){
    return runOnce(()-> {rightCANdle.setLEDs(0, 255, 0, 0, start3, count); });
     //return  rightCANdle.setLEDs(0, 255, 0, 0, start, count); //green
   }
   public Command setGreen4(){
    return runOnce(()-> {rightCANdle.setLEDs(0, 255, 0, 0, start4, count); });
     //return  rightCANdle.setLEDs(0, 255, 0, 0, start, count); //green
   }
   public Command setGreenAll(){
    return runOnce(()-> {rightCANdle.setLEDs(0, 255, 0, 0, start, countAll); });
     //return  rightCANdle.setLEDs(0, 255, 0, 0, start, count); //green
   }
   
     
  
  
  public Command setRed(){
    return runOnce(()-> {
      rightCANdle.setLEDs(225, 0, 0, 0, start, count);
      leftCANdle.setLEDs(255, 0, 0, 0, start, count); 

    });
    
   }//red
   public Command setRed2(){
    return runOnce(()-> {rightCANdle.setLEDs(225, 0, 0, 0, start2, count); });
    
   }//red
   public Command setRed3(){
    return runOnce(()-> {rightCANdle.setLEDs(225, 0, 0, 0, start3, count); });
    
   }//red
   public Command setRed4(){
    return runOnce(()-> {rightCANdle.setLEDs(225, 0, 0, 0, start4, count); });
    
   }//red
   public Command setRedAll(){
    return runOnce(()-> {rightCANdle.setLEDs(225, 0, 0, 0, start, countAll); });
    
   }//red
   
   
   public Command setYellow(){
    return runOnce(()-> {
      rightCANdle.setLEDs(225, 150, 0, 0, start, count);
      leftCANdle.setLEDs(225, 150, 0, 0, start, count);
    });
   
   }//yellow
   public Command setYellow2(){
    return runOnce(()-> {rightCANdle.setLEDs(225, 150, 0, 0, start2, count); });
   
   }//yellow
   public Command setYellow3(){
    return runOnce(()-> {rightCANdle.setLEDs(225, 150, 0, 0, start3, count); });
   
   }//yellow
   public Command setYellow4(){
    return runOnce(()-> {rightCANdle.setLEDs(225, 150, 0, 0, start4, count); });
   
   }//yellow
   public Command setYellowAll(){
    return runOnce(()-> {rightCANdle.setLEDs(225, 150, 0, 0, start, countAll); });
   
   }//yellow
   
   
   public Command setWhite(){
    return runOnce(()-> {rightCANdle.setLEDs(225, 255, 225, 0, start, count); });
   }//white
   public Command setWhite2(){
    return runOnce(()-> {rightCANdle.setLEDs(225, 255, 225, 0, start2, count); });
   }//white
   public Command setWhite3(){
    return runOnce(()-> {rightCANdle.setLEDs(225, 255, 225, 0, start3, count); });
   }//white
   public Command setWhite4(){
    return runOnce(()-> {rightCANdle.setLEDs(225, 255, 225, 0, start4, count); });
   }//white
   public Command setWhiteAll(){
    return runOnce(()-> {rightCANdle.setLEDs(225, 255, 225, 0, start, countAll); });
   }//white



   public Command setBlue(){
    return runOnce(()-> {rightCANdle.setLEDs(0, 0, 225, 0, start, count); });
    //blue 
    }
    public Command setBlue2(){
        return runOnce(()-> {rightCANdle.setLEDs(0, 0, 225, 0, start2, count); });
        //blue    
    }
public Command setBlue3(){ 
    return runOnce(()-> {rightCANdle.setLEDs(0, 0, 225, 0, start3, count); });
            //blue
   }
   public Command setBlue4(){
    return runOnce(()-> {rightCANdle.setLEDs(0, 0, 225, 0, start4, count); });
    //blue
   }public Command setBlueAll(){
    return runOnce(()-> {rightCANdle.setLEDs(0, 0, 225, 0, start, countAll); });
    //blue
   }



   public Command setOff(){
    return runOnce(()-> {rightCANdle.setLEDs(0, 0, 0, 0, start, countAll); });
    //off?
   }
   public Command setPurple(){
    return runOnce(()-> {rightCANdle.setLEDs(170, 0, 255, 0, start, count); });
    //purple
   }
   public Command setPurple2(){
    return runOnce(()-> {rightCANdle.setLEDs(170, 0, 255, 0, start2, count); });
    //purple
   }  
   public Command setPurple3(){
    return runOnce(()-> {rightCANdle.setLEDs(170, 0, 255, 0, start3, count); });
    //purple
   }  
   public Command setPurple4(){
    return runOnce(()-> {rightCANdle.setLEDs(170, 0, 255, 0, start4, count); });
    //purple
   }  
   public Command setPurpleAll(){
    return runOnce(()-> {rightCANdle.setLEDs(170, 0, 255, 0, start, countAll); });
    //purple
   }  
     
   
   public Command setOrange(){
    return runOnce(()-> {rightCANdle.setLEDs(255, 110, 0, 0, start, count); });
    //orange
   }
   public Command setOrange2(){
    return runOnce(()-> {rightCANdle.setLEDs(255, 110, 0, 0, start2, count); });
    //orange
   }
   public Command setOrange3(){
    return runOnce(()-> {rightCANdle.setLEDs(255, 110, 0, 0, start3, count); });
    //orange
   }
   public Command setOrange4(){
    return runOnce(()-> {rightCANdle.setLEDs(255, 110, 0, 0, start4, count); });
    //orange
   }
   public Command setOrangeAll(){
    return runOnce(()-> {rightCANdle.setLEDs(255, 110, 0, 0, start, countAll); });
    //orange
   }


   public Command setAwsomeRed(){
    return runOnce(()-> {rightCANdle.setLEDs(225, 50, 0, 0, start, count); });
    //awsome red
   }
   public Command setAwsomeRed2(){
    return runOnce(()-> {rightCANdle.setLEDs(225, 50, 0, 0, start2, count); });
    //awsome red
   }
   public Command setAwsomeRed3(){
    return runOnce(()-> {rightCANdle.setLEDs(225, 50, 0, 0, start3, count); });
    //awsome red
   }
   public Command setAwsomeRed4(){
    return runOnce(()-> {rightCANdle.setLEDs(225, 50, 0, 0, start4, count); });
    //awsome red
   }
   public Command setAwsomeRedAll(){
    return runOnce(()-> {rightCANdle.setLEDs(225, 50, 0, 0, start, countAll); });
    //awsome red
   }

   public Command setTeal(){
    return runOnce(()-> {
      rightCANdle.setLEDs(0, 255, 120, 0, start, count);
      leftCANdle.setLEDs(0, 255, 120, 0, start, count); 

    });
    //its teal dude
   }
   public Command setTeal2(){
    return runOnce(()-> {rightCANdle.setLEDs(0, 255, 120, 0, start2, count); });
    //its teal dude
   }
   public Command setTeal3(){
    return runOnce(()-> {rightCANdle.setLEDs(0, 255, 120, 0, start3, count); });
    //its teal dude
   }
   public Command setTeal4(){
    return runOnce(()-> {rightCANdle.setLEDs(0, 255, 120, 0, start4, count); });
    //its teal dude
   }
   public Command setTealAll(){
    return runOnce(()-> {rightCANdle.setLEDs(0, 255, 120, 0, start, countAll); });
    //its teal dude
   }
   
   
   
   
   
   
   public Command setTealOne(){
    return runOnce(()-> {rightCANdle.setLEDs(0, 255, 120, 0, startOne, countOne); });
    //its teal dude
   }

   public Command setTealThree(){
    return runOnce(()-> {rightCANdle.setLEDs(0, 255, 120, 0, startThree, countOne); });
    //its teal dude
   }
   public Command setTealFive(){
    return runOnce(()-> {rightCANdle.setLEDs(0, 255, 120, 0, startFive, countOne); });
    //its teal bro
   }
   public Command setTealSeven(){
    return runOnce(()-> {rightCANdle.setLEDs(0, 255, 120, 0, startSeven, countOne); });
    //its teal dude
   }
   public Command setTealNine(){
    return runOnce(()-> {rightCANdle.setLEDs(0, 255, 120, 0, startNine, countOne); });
    //its teal dudep
   }
   public Command setTeal11(){
    return runOnce(()-> {rightCANdle.setLEDs(0, 255, 120, 0, start11, countOne); });
    //its teal brosky
   }
   public Command setAwsomeRedTwo(){
    return runOnce(()-> {rightCANdle.setLEDs(225, 50, 0, 0, startTwo, count); });
    //nice red noble
   }
   public Command setAwsomeRedFour(){
    return runOnce(()-> {rightCANdle.setLEDs(225, 50, 0, 0, startFour, count); });
    //nice red noble
   }
   public Command setAwsomeRedSix(){
    return runOnce(()-> {rightCANdle.setLEDs(225, 50, 0, 0, startSix, count); });
    //nice red noble
   }
   public Command setAwsomeRedEight(){
    return runOnce(()-> {rightCANdle.setLEDs(225, 50, 0, 0, startEight, count); });
    //nice red noble
   }
   public Command setAwsomeRedTen(){
    return runOnce(()-> {rightCANdle.setLEDs(225, 50, 0, 0, startTen, count); });
    //nice red noble
   }
   
   
   public Command setTealTwo(){
    return runOnce(()-> {rightCANdle.setLEDs(225, 50, 0, 0, startTwo, count); });
    //nice red noble
   }
   public Command setTealFour(){
    return runOnce(()-> {rightCANdle.setLEDs(225, 50, 0, 0, startFour, count); });
    //nice red noble
   }
   public Command setTealSix(){
    return runOnce(()-> {rightCANdle.setLEDs(225, 50, 0, 0, startSix, count); });
    //nice red noble
   }
   public Command setTealEight(){
    return runOnce(()-> {rightCANdle.setLEDs(225, 50, 0, 0, startEight, count); });
    //nice red noble
   }
   public Command setTealTen(){
    return runOnce(()-> {rightCANdle.setLEDs(225, 50, 0, 0, startTen, count); });
    //nice red noble
   }
   
   
   public Command setAwsomeRedOne(){
    return runOnce(()-> {rightCANdle.setLEDs(0, 255, 120, 0, startOne, countOne); });
    //its teal dude
   }

   public Command setAwsomeRedThree(){
    return runOnce(()-> {rightCANdle.setLEDs(0, 255, 120, 0, startThree, countOne); });
    //its teal dude
   }
   public Command setAwsomeRedFive(){
    return runOnce(()-> {rightCANdle.setLEDs(0, 255, 120, 0, startFive, countOne); });
    //its teal bro
   }
   public Command setAwsomeRedSeven(){
    return runOnce(()-> {rightCANdle.setLEDs(0, 255, 120, 0, startSeven, countOne); });
    //its teal dude
   }
   public Command setAwsomeRedNine(){
    return runOnce(()-> {rightCANdle.setLEDs(0, 255, 120, 0, startNine, countOne); });
    //its teal dudep
   }
   public Command setAwsomeRed11(){
    return runOnce(()-> {rightCANdle.setLEDs(0, 255, 120, 0, start11, countOne); });
    //its teal brosky
   }


   













   }


//idk how to make blink

  
        
    




























