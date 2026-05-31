package dev.weaponboy.nexus_pathing.PathGeneration.commands;

import java.util.ArrayList;

public class buildPath{

    public buildPath(SectionBuilder[] commands){

        for (int i = 0; i < commands.length; i++){
            commands[i].buildSection();
        }

    }

}
