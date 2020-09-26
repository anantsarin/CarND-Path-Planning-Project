double get_cost(float dist) {
	/* code */
	double cost  = (1 - exp(-1/dist));
	return cost;
}

	int getLaneBehaviour(vector<vector<double >> sensor_fusion, double car_s, int path_size, int lane, double same_lane_cost){
		int my_lane = lane;
		double cost = 0.0;
		vector<double> cost_total = 0.0;
		for(int j=0; j<2 ; j++){
			cost = 0.0;
			if(my_lane==0 || my_lane==2){
				lane = 1;
				j++;
			}
			else{
				lane = j*2;
			}
			for(int i =0; i<sensor_fusion.size(); i++){
				float d = sensor_fusion[i][6]; //ith cars
				if(d < (2+4*lane+2) && d > (2+4*lane-2)){
			      //my lane
					double vx = sensor_fusion[i][3];
		            double vy = sensor_fusion[i][4];
		            double check_speed = sqrt(vx*vx+vy*vy);
		            double check_car_s = sensor_fusion[i][5];

		            check_car_s += ((double)path_size*0.02*check_speed);
		            if((check_car_s > car_s)&& ((check_car_s - car_s)< 30)){
		              //remain in the lane 2
		            	cost += get_cost((check_car_s - car_s));
		            }
		            if((check_car_s < car_s)&& ((car_s - check_car_s)< 10)) {
		            	// there is a car at the back
		            	cost += get_cost((car_s - check_car_s));
		            }
			    }
			}
			cost_total.push_back(cost);
			if(lane==1){ // if destination is lane 1
				if(same_lane_cost < cost_total[0]){
					return my_lane;
				}
				else{
					return 1;
				}
			}
			else{
				//return 0 lane number
				int ans  = (cost_total[0] < cost_total[1]) ? ((cost_total[0] < same_lane_cost)?0:1):((cost_total[1] < same_lane_cost)?2:1) ;
				return ans;
			}
			}
		}
		// if()
		// if(right){
		// 	for(int i =0; i<sensor_fusion.size(); i++){
	 //            double vx = sensor_fusion[i][3];
	 //            double vy = sensor_fusion[i][4];
	 //            double check_speed = sqrt(vx*vx+vy*vy);
	 //            double check_car_s = sensor_fusion[i][5];

	 //            check_car_s += ((double)path_size*0.02*check_speed);
	 //            if((check_car_s > car_s)&& ((check_car_s - car_s)< 30)){
	 //              //remain in the lane 2
	 //            	cost_right += (1 - exp(-1/(check_car_s - car_s)));

	 //            }
	 //            if((check_car_s < car_s)&& ((car_s - check_car_s)< 10)) {
	 //            	// there is a car at the back
	 //            	cost_right += (1 - exp(-1/(car_s - check_car_s)));
	 //            }

		// 	}
// 		// }

// 	}
// 	for(int i =0; i<sensor_fusion.size(); i++){
// 		float d = sensor_fusion[i][6]; //ith cars
//             //////////////////////
// 	    int my_car_lane_dis = 2+4*lane;

// 	    if(d < (2+4*lane+2) && d > (2+4*lane-2)){
// 	      //my lane
// 	    }
// 	}



// 	if (lane == 2){
//                   next_lane = 1;
//                   //collision if safe to  switch
//                   for(int i =0; i<sensor_fusion.size(); i++){
//                     double vx = sensor_fusion[i][3];
//                     double vy = sensor_fusion[i][4];
//                     double check_speed = sqrt(vx*vx+vy*vy);
//                     double check_car_s = sensor_fusion[i][5];

//                     check_car_s += ((double)path_size*0.02*check_speed);
//                     if((check_car_s > car_s)&& ((check_car_s - car_s)< 30)){
//                       //remain in the lane 2
//                     	break;
//                     }
//                     if((check_car_s < car_s)&& ((car_s - check_car_s)< 10)) {
//                     	// there is a car at the back
//                     }

//                   }
//                 }
// }



// Check for cars ahead and behind given a lane
vector<double> getLaneBehaviour(vector<vector<double >> sensor_fusion, int lane, int prev_size, double car_end_s,
	double car_s, double car_v, double 30, double 30, bool print){

    double status_car_ahead = 0; // 0: no car ahead, 1: car ahead in range, 2: collision warning
    double status_car_behind = 0; // 0: no car behind, 1: car behind in range, 2: collision warning
    double dist_car_ahead = 10000;
    double dist_car_behind = 10000;
    double v_car_ahead = 49.5;
    double v_car_behind = 49.5;
    double id_car_ahead = -1;
    double id_car_behind = -1;

    for(int i = 0; i < sensor_fusion.size(); i++){

        // [id, x, y, v_x, v_y, s, d]
        vector<double> i_car = sensor_fusion[i];
        double i_car_d = i_car[6];

        if (i_car_d > (2 + 4 * lane - 2) && i_car_d < (2 + 4 * lane + 2)){

            // ith car is in the same lane as in the parameter lane
            double i_car_v_x = i_car[3];
            double i_car_v_y = i_car[4];
            double i_car_v = sqrt(i_car_v_x * i_car_v_x + i_car_v_y * i_car_v_y); // ith car speed in m/s
            double i_car_s = i_car[5];
            double i_car_end_s = i_car_s + (double) prev_size * 0.02 * i_car_v; // predicted ith car end s location

            if (car_v >= i_car_v * 2.24){
                // If the trailing car is travelling at a lower speed, hence, decrease the backward horizon
                30 = 20;
            }

            if (i_car_s > car_s && i_car_s < (car_s + 30/2)){
                // The i_th car is ahead and in collision warning distance
                status_car_ahead = 2;
                double i_dist_car_ahead = i_car_s - car_s;
                dist_car_ahead = i_dist_car_ahead;
                v_car_ahead = i_car_v * 2.24;
                id_car_ahead = i_car[0];
                break;

            }
            else if (i_car_s < car_s && car_s < (i_car_s + 30/2)){
                // The i_th car is behind and in collision warning distance
                status_car_behind = 2;
                double i_dist_car_behind = car_s - i_car_s;
                dist_car_behind = i_dist_car_behind;
                v_car_behind = i_car_v * 2.24;
                id_car_behind = i_car[0];
                break;

            }
            else if (i_car_end_s > car_end_s && i_car_end_s < (car_end_s + 30)){
                // The i_th car is ahead and in the path planning horizon
                status_car_ahead = 1;
                double i_dist_car_ahead = i_car_end_s - car_end_s;

                if (i_dist_car_ahead < dist_car_ahead){
                    dist_car_ahead = i_dist_car_ahead;
                    v_car_ahead = i_car_v * 2.24;
                    id_car_ahead = i_car[0];
                }
            }
            else if (i_car_end_s < car_end_s && car_end_s < (i_car_end_s + 30)){
                // The i_th car is behind and in the path planning horizon
                status_car_behind = 1;
                double i_dist_car_behind = car_end_s - i_car_end_s;

                if (i_dist_car_behind < dist_car_behind){
                    dist_car_behind = i_dist_car_behind;
                    v_car_behind = i_car_v * 2.24;
                    id_car_behind = i_car[0];
                }
            }
        }
    }

    if (print){
        /*if (status_car_ahead == 1){
            cout << " lane: " << lane << " dist car ahead: " << dist_car_ahead << " speed car ahead: " << v_car_ahead << endl;
        }
        if (status_car_behind == 1){
            cout << " lane: " << lane << " dist car behind: " << dist_car_behind << " speed car behind: " << v_car_behind << endl;
        }*/

        cout << " lane: " << lane << " car ahead " << status_car_ahead << " dist car ahead: "
         << dist_car_ahead << " speed car ahead: " << v_car_ahead << endl;
        cout << " lane: " << lane << " car behind " << status_car_behind << " dist car behind: "
        << dist_car_behind << " speed car behind: " << v_car_behind << endl;
    }

    return {status_car_ahead, status_car_behind, dist_car_ahead, dist_car_behind, v_car_ahead,
     v_car_behind, id_car_ahead, id_car_behind};
}