package frc.robot.subsystems.superstructure.leds;
import java.util.ArrayList;


import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.Ports;
import frc.robot.util.TeamColor;
public class leds implements Subsystem {

    // an led strip
    private AddressableLED led;

    private double m;
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
    
    public leds(){
        this.led = new AddressableLED(Ports.ADDRESSABLE_LED);

        this.length = ledsConstants.LENGTH;
        buffer = new AddressableLEDBuffer(length);

        led.setLength(length);
        led.setData(buffer);
        led.start();


        numFlashes = 0;
        periodsPassed = 0;

        on = true;
        offColor = ledsConstants.OFF_COLOR;
        currentColor = offColor;
        mode = 1;
    }

    public void updateData(){
        led.setData(buffer);
    }

    public void setAllPixels(TeamColor color){
        currentColor = color;

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


    public Command setAllLedsCommand(TeamColor color){
        return runOnce(()-> setAllPixels(color));
    }

    public Command flashAllLedsCommand(TeamColor color, int numFlashes){
        return runOnce(()-> flashAllPixels(color,numFlashes));
    }

    public Command pulseLedsCommand(TeamColor color, int amount){
        return runOnce(()-> pulseLEDS(color, amount));
    }






    @Override
    public void periodic(){
        periodsPassed ++;

        if(mode == 1){

            if (periodsPassed == 5 && numFlashes >0){

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
}
