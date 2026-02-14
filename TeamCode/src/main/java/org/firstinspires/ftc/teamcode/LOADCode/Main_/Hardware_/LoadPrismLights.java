package org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;

public class LoadPrismLights {
    private GoBildaPrismDriver prism;

    public void init(OpMode opMode, String name){
        prism = opMode.hardwareMap.get(GoBildaPrismDriver.class, name);
        prism.setStripLength(24);
    }

    public void setStripsRainbow(){
        PrismAnimations.AnimationBase rainbow = new PrismAnimations.Rainbow();
        rainbow.setIndexes(0, 12);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, rainbow);
    }
    public void setStripsRed(){
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_1, new PrismAnimations.Solid(Color.RED, 12, 29));
    }
    public void setStripsBlue(){
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_1, new PrismAnimations.Solid(Color.BLUE, 12, 29));
    }

    public void clearStrips(){
        prism.clearAllAnimations();
    }
}
