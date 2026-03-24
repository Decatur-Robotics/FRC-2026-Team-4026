package frc.robot.subsystems.superstructure.leds;
import java.util.ArrayList;


import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Ports;
import frc.robot.util.TeamColor;
public class Leds extends SubsystemBase {

    // an led strip
    private AddressableLED led;
    // how many leds on the strip
    private int length;
    // a buffer with data on the led strip states
    private AddressableLEDBuffer buffer;
    // the current desired color of the leds
    private TeamColor currentColor;
    // the color when the leds are off
    private final TeamColor offColor;
    //number of remaining led flashes
    private int numFlashes;
    //timer for flashing leds
    private int periodsPassed;

    private boolean on;
    //array list for all the leds to add for pulsing
    ArrayList<TeamColor> ledsToAdd = new ArrayList<TeamColor>();
    // different modes for leds.  1 = normal flashing   2 = pulsing
    private int mode;

    private int red = 0;
    private int green = 0;
    private int blue = 0;
    private int fadeStage = 1;
    private final int rainbowSpeed = 2;

    private final int redFadeBottom = 200;
    private final int redFadeTop = 255;
    private final int blueFadeBottom = 140;
    private final int blueFadeTop = 240;
    private final int fadeSpeed = 5;
    private LEDPattern pattern;
    private final Timer ledTimer;
    public Leds(){
        ledTimer = new Timer();
        this.led = new AddressableLED(Ports.ADDRESSABLE_LED);

        this.length = LedsConstants.LENGTH;
        buffer = new AddressableLEDBuffer(length);

        led.setLength(length);
        led.setData(buffer);
        led.start();


        numFlashes = 0;
        periodsPassed = 0;

        on = true;
        offColor = LedsConstants.OFF_COLOR;
        currentColor = offColor;
        mode = 1;
    }

    public void updateData(){
        led.setData(buffer);
    }

    public void setAllPixels(TeamColor color){
        currentColor = color;
        mode = 1;
        for (int i = 0; i < length; i++){
            buffer.setRGB(i, color.r, color.g, color.b);
        }
        this.updateData();
    }

    public void flashAllPixels(TeamColor color, int numFlashes){
        setAllPixels(color);
        this.numFlashes = numFlashes;
    }
    // for pulsing moves all the leds foward one 
    public void stepAllPixels(){
        for( int i = length; i >0;){
            i--;
            if(i>0){
                buffer.setRGB(i, buffer.getRed(i-1), buffer.getGreen(i-1), buffer.getBlue(i-1));
            }
        }
    }

    public void pulseLEDS(TeamColor color, int amount){
        mode = 2;
        for (int i = 0; i < amount;i++){
            ledsToAdd.add(color);
        }
    }

    public void rainbowPulseLEDS(int length){
        pulseLEDS(LedsConstants.RED, length);
        pulseLEDS(LedsConstants.ORANGE, length);
        pulseLEDS(LedsConstants.YELLOW, length);
        pulseLEDS(LedsConstants.GREEN, length);
        pulseLEDS(LedsConstants.CYAN, length);
        pulseLEDS(LedsConstants.BLUE, length);
        pulseLEDS(LedsConstants.MAGENTA, length);
    }

    public void pulsingYellowLEDS(){
        pulseLEDS(LedsConstants.YELLOW, 3);
        pulseLEDS(LedsConstants.OFF_COLOR, 5);
    }

    public void rainbowLEDS(){
        mode = 3;
    }
    public void fadeRedLEDS(){
        mode = 4;
        red = 255;
        blue = 0;
        green = 0;
        fadeStage = 2;
    }

    public void fadeBlueLEDS(){
        mode = 5;
        red = 0;
        blue = 255;
        green = 0;
        fadeStage = 2;
    }

    public void shiftLEDS(){
        mode = 6;
    }

    //commands
    public Command setAllLedsCommand(TeamColor color){
        return runOnce(()-> setAllPixels(color));
    }

    public Command flashAllLedsCommand(TeamColor color, int numFlashes){
        return runOnce(()-> flashAllPixels(color,numFlashes));
    }

    public Command pulseLedsCommand(TeamColor color, int amount){
        return runOnce(()-> pulseLEDS(color, amount));
    }

    public Command rainbowPulseLEDSCommand(int length){
        return runOnce(()-> rainbowPulseLEDS(length));
    }

    public Command rainbowLEDSCommand(){
        return runOnce(()-> rainbowLEDS());
    }

    public Command fadeRedLEDSCommand(){
        return runOnce(()->fadeRedLEDS());
    }

    public Command fadeBlueLEDSCommand(){
        return runOnce(()->fadeBlueLEDS());
    }

    public Command shiftLEDSCommand(){
        return runOnce(()->shiftLEDS());
    }
    
    @Override
    public void periodic(){
        periodsPassed ++;
        if(DriverStation.getMatchTime() == 132){
            flashAllLedsCommand(LedsConstants.SHIFT_COLOR, 3);
        }
        if(DriverStation.getMatchTime() == 103){
            flashAllLedsCommand(LedsConstants.SHIFT_COLOR, 3);
        }
        if(DriverStation.getMatchTime() == 78){
            flashAllLedsCommand(LedsConstants.SHIFT_COLOR, 3);
        }
        if(DriverStation.getMatchTime() == 52){
            flashAllLedsCommand(LedsConstants.SHIFT_COLOR, 3);
        }
        if(DriverStation.getMatchTime() == 28){
            flashAllLedsCommand(LedsConstants.SHIFT_COLOR, 3);
        }
        
        if(mode == 1){

            if (periodsPassed == 10 && numFlashes >0){

                if(on){

                    for (int i = 0; i < length;i++){
                        buffer.setRGB(i, offColor.r, offColor.g, offColor.b);
                    }

                    this.updateData();
                    on = false;
                }
                else{

                    for(int i = 0; i< length;i++){
                        buffer.setRGB(i, currentColor.r, currentColor.g, currentColor.b);
                    }

                    this.updateData();
                    on = true;
                    numFlashes --;
                }
            }

        } else if (mode == 2){

           stepAllPixels();
           if(ledsToAdd.size() >0){

            buffer.setRGB(0,ledsToAdd.get(0).r,ledsToAdd.get(0).g,ledsToAdd.get(0).b);
            ledsToAdd.remove(0);
           } else{

            buffer.setRGB(0,0,0,0);

           }
          
           if(ledsToAdd.size() == 0){
                boolean ledsClear = true;
                for(int i = 0; i < length; i++){
                    if(buffer.getRed(i) !=0 || buffer.getBlue(i) != 0 || buffer.getGreen(i) != 0){
                        ledsClear = false;
                    }
                }
                if (ledsClear){
                    mode = 1;
                    numFlashes = 0;
                }
           }

        }  else if (mode == 3){
            if (fadeStage == 1){
                if (red < 10){
                    fadeStage = 2;
                } else{
                    red -= rainbowSpeed;
                    green += rainbowSpeed;
                }
            } 
            else if (fadeStage == 2){
                if (green < 10){
                    fadeStage = 3;
                } else{
                    green -= rainbowSpeed;
                    blue += rainbowSpeed;
                }
            }
            else if (fadeStage == 3){
                if ( blue < 10){
                    fadeStage = 1;
                } else {
                    blue -= rainbowSpeed;
                    red += rainbowSpeed;
                }
            }
            // if value of colors are out of range it fixes it
            if (red > 255){
                red = 255;
            }
            if (green > 255){
                green = 255;
            }
            if( blue > 255){
                blue = 255;
            }
            if (red < 0){
                red = 0;
            }
            if( green < 0){
                green = 0;
            }
            if (blue < 0){
                blue = 0;
            }

            for(int i = 0; i < length; i++){
                buffer.setRGB(i, red, green, blue);
            }
        }

        if(mode == 4){
            if (fadeStage == 1){
                if(red <= redFadeBottom){
                    fadeStage = 2;
                } else{
                    red -= fadeSpeed;
                }
            } else if (fadeStage == 2){
                if(red >= redFadeTop){
                    fadeStage = 1;
                } else{
                    red += fadeSpeed;
                }
            }
            if(red > 255){
                red =255;
            }
            if (red < 0){
                red = 0;
            }

            for(int i = 0; i < length; i++){
                buffer.setRGB(i, red, 0, 0);
            }
        }


        if(mode == 5){
            if (fadeStage == 1){
                if(blue <= blueFadeBottom){
                    fadeStage = 2;
                } else{
                    blue -= fadeSpeed;
                }
            } else if (fadeStage == 2){
                if(blue >= blueFadeTop){
                    fadeStage = 1;
                } else{
                    blue += fadeSpeed;
                }
            }
            if(blue > 255){
                blue =255;
            }
            if (blue < 0){
                blue = 0;
            }

            for(int i = 0; i < length; i++){
                buffer.setRGB(i, 0, 0, blue);
            }
        }

        if(mode == 6){
           //shift timer

           //Transition
           if(DriverStation.getMatchTime()>130 && DriverStation.getMatchTime() <= 140){
                double precentageLeft = (DriverStation.getMatchTime() - 130)/10;
                int ledsToSet = (int)Math.round(precentageLeft*length);
                setAllPixels(offColor);
                for(int i = 0; i < ledsToSet; i++){
                    buffer.setRGB(i, LedsConstants.SHIFT_COLOR.r, LedsConstants.SHIFT_COLOR.g, LedsConstants.SHIFT_COLOR.b);
                }
           } else if(DriverStation.getMatchTime()>105 && DriverStation.getMatchTime() <= 130){
                double precentageLeft = (DriverStation.getMatchTime() - 105)/25;
                int ledsToSet = (int)Math.round(precentageLeft*length);
                setAllPixels(offColor);
                for(int i = 0; i < ledsToSet; i++){
                    buffer.setRGB(i, LedsConstants.SHIFT_COLOR.r, LedsConstants.SHIFT_COLOR.g, LedsConstants.SHIFT_COLOR.b);
                }
            } else if(DriverStation.getMatchTime()>80 && DriverStation.getMatchTime() <= 105){
                double precentageLeft = (DriverStation.getMatchTime() - 80)/25;
                int ledsToSet = (int)Math.round(precentageLeft*length);
                setAllPixels(offColor);
                for(int i = 0; i < ledsToSet; i++){
                    buffer.setRGB(i, LedsConstants.SHIFT_COLOR.r, LedsConstants.SHIFT_COLOR.g, LedsConstants.SHIFT_COLOR.b);
                }
            } else if(DriverStation.getMatchTime()>55 && DriverStation.getMatchTime() <= 80){
                double precentageLeft = (DriverStation.getMatchTime() - 55)/25;
                int ledsToSet = (int)Math.round(precentageLeft*length);
                setAllPixels(offColor);
                for(int i = 0; i < ledsToSet; i++){
                    buffer.setRGB(i, LedsConstants.SHIFT_COLOR.r, LedsConstants.SHIFT_COLOR.g, LedsConstants.SHIFT_COLOR.b);
                }
            }else if(DriverStation.getMatchTime()>30 && DriverStation.getMatchTime() <= 55){
                double precentageLeft = (DriverStation.getMatchTime() - 30)/25;
                int ledsToSet = (int)Math.round(precentageLeft*length);
                setAllPixels(offColor);
                for(int i = 0; i < ledsToSet; i++){
                    buffer.setRGB(i, LedsConstants.SHIFT_COLOR.r, LedsConstants.SHIFT_COLOR.g, LedsConstants.SHIFT_COLOR.b);
                }
            } else if(DriverStation.getMatchTime()>0 && DriverStation.getMatchTime() <= 30){
                double precentageLeft = (DriverStation.getMatchTime() - 0)/25;
                int ledsToSet = (int)Math.round(precentageLeft*length);
                setAllPixels(offColor);
                for(int i = 0; i < ledsToSet; i++){
                    buffer.setRGB(i, LedsConstants.SHIFT_COLOR.r, LedsConstants.SHIFT_COLOR.g, LedsConstants.SHIFT_COLOR.b);
                }
            }
        }
        if (periodsPassed > 5) {
			periodsPassed = 0;
		}
        

    }


    public int getLength(){
        return length;
    }

    public TeamColor getCurrentColor(){
        return currentColor;
    }
    public void progressBar(){
        pattern = LEDPattern.progressMaskLayer(()-> DriverStation.getMatchTime()%25);
        pattern.applyTo(buffer);
    }
}
