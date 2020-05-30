//
// Created by parallels on 2/6/20.
//

#include "../include/my_frontier_exploration/inflation_obstacle_builder.h"


void inflation_obstacle_builder::ConfigInit(){
    const char ConfigFile[]= "./src/my_frontier_exploration/Configuration/config.txt";
    Config configSettings(ConfigFile);
    inflation_scale = configSettings.Read("inflation_scale",6);
}

void inflation_obstacle_builder::set_obstacle_space(){
    original_map = newest_map;
    inflation_map = newest_map;
    ConfigInit();
    for (int i = 0; i < original_map.data.size(); i++)
    {
        if (original_map.data[i] == 100 ){ //obstacle or unknown
            inflation_obstacle_builder::cell c((i+original_map.info.width)%original_map.info.width,
                    floor(i/original_map.info.width),i);
            obstacle_space.push_back(c);

        }

    }

}
void inflation_obstacle_builder::set_obstacle_edge_vec()
{

    for (int j = 0; j < obstacle_space.size(); j++) {
        bool have_free1 = (original_map.data[obstacle_space[j].index - 1] == 0) ? true : false;//check left
        bool have_free2 = (original_map.data[obstacle_space[j].index + 1] == 0) ? true : false;//check right
        bool have_free3 = (original_map.data[obstacle_space[j].index + original_map.info.width] == 0) ? true : false;//check up
        bool have_free4 = (original_map.data[obstacle_space[j].index - original_map.info.width] == 0) ? true : false;//check down
        bool have_free5 = (original_map.data[obstacle_space[j].index - original_map.info.width + 1] == 0) ? true : false;//check down right
        bool have_free6 = (original_map.data[obstacle_space[j].index - original_map.info.width - 1] == 0) ? true : false;//check down left
        bool have_free7 = (original_map.data[obstacle_space[j].index + original_map.info.width - 1] == 0) ? true : false;//check up left
        bool have_free8 = (original_map.data[obstacle_space[j].index + original_map.info.width + 1] == 0) ? true : false;//check up right
        if (have_free1 || have_free2 || have_free3 || have_free4 || have_free5 || have_free6 ||
            have_free7 || have_free8) {
            obstacle_edge_vec.push_back(obstacle_space[j]);
        }
    }

    return ;

}
void inflation_obstacle_builder::inflation() {
    for (int i = 0; i < obstacle_edge_vec.size(); i++) {
        vector<int> inflation_candidate;
        for (int j = -inflation_scale; j <= inflation_scale; j++) {
            for (int k = -inflation_scale; k <= inflation_scale; k++) {
                int x = obstacle_edge_vec[i].x  + j ;
                int y = obstacle_edge_vec[i].y  + k ;
                if(original_map.data[x + y * original_map.info.width] == 0 || original_map.data[x + y * original_map.info.width] == -1){
                    inflation_candidate.push_back(x + y * original_map.info.width);
                }
            }

        }
        for (int l = 0; l < inflation_candidate.size(); l++) {
            inflation_map.data[inflation_candidate[l]] = 50;
        }
    }
    obstacle_edge_vec.clear();
    obstacle_space.clear();
}
