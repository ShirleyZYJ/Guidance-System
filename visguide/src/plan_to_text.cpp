#include "ros/ros.h"
#include "message_filters/subscriber.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "nav_msgs/Path.h"
#include <string>
#include <vector>
#include <math.h>
#include "iostream"
using namespace std;
#define PI 3.14

class new_pos {
    public:
    double x, y, z;
    double ox, oy, oz, ow;
};
double getSlope(double x1, double x2, double y1, double y2);
int sample_points(vector<new_pos> &gPath);
void callback(const nav_msgs::Path::ConstPtr & msgPath);
double getDist(double x1, double y1, double x2, double y2);
void dis_direct(vector<new_pos> &gPath);
void angle_correction();
void text_to_speech(vector<string> msg);
//nav_msgs::Path gPlan;
int counter = 0;
void callback(const nav_msgs::Path::ConstPtr & msgPath){
    //gPlan(msgPath);
    counter++;
    if(counter % 5 != 0) {    
    int i = 0;
    vector<new_pos> gPath;
    new_pos point;
	for(std::vector<geometry_msgs::PoseStamped>::const_iterator it= msgPath->poses.begin(); it!= msgPath->poses.end();  ++it) {
        //cout<<"Position  "<<(*it).pose.position.x<<"   "<<(*it).pose.position.y<<"   "<<(*it).pose.position.z<<endl;
        
        
        //Cloning for easier reference
        point.x = (*it).pose.position.x; point.y = (*it).pose.position.y; point.z = (*it).pose.position.z;
        point.ox = (*it).pose.orientation.x; point.oy = (*it).pose.orientation.y; point.oz = (*it).pose.orientation.z;
        point.ow = (*it).pose.orientation.z;
        gPath.push_back(point);
        ++i;
    }
    sample_points(gPath);
    cout<<"Total :"<<i<<endl;
    //cout<<msgPath.poses[0].pose<<endl;
    cout<<"-------------"<<endl;
    }
}

double getSlope(double x1, double x2, double y1, double y2) {
    double slope = (y2 - y1) / (x2 - x1);
    return slope;
}

int sample_points(vector<new_pos> &gPath) {
    int l = gPath.size();
    vector<new_pos> newgPath(3);
    //Parameters
    double threshold = 1;
    int prev_i = 0;
    int p_prev_i = 500;
    for(int i = 1000;i < l; i += 500) {
        double slope3 = (gPath[i].x, gPath[p_prev_i].x, gPath[i].y, gPath[p_prev_i].y);
        double slope2 = (gPath[i].x, gPath[prev_i].x, gPath[i].y, gPath[prev_i].y);
        double slope1 = (gPath[prev_i].x, gPath[p_prev_i].x, gPath[prev_i].y, gPath[p_prev_i].y);
        if(abs(slope2 - slope1) <= threshold) {
            prev_i = i;
        } else {
            //p_prev_i = prev_i;
            //prev_i = i;
            newgPath[0] = gPath[p_prev_i];
            newgPath[1] = gPath[prev_i];
            newgPath[2] = gPath[i];
            dis_direct(newgPath);
            return 0;
        }
    }
    double d1 = getDist(gPath[0].x, gPath[l - 1].x, gPath[0].y, gPath[l - 1].y);
    string text_msg = "Go forward " + to_string(int(round(d1))) + " meters";
    cout<<text_msg<<endl;
    //text_to_speech(text_msg);
    return 0;
}
double getDist(double x1, double x2, double y1, double y2) {
    double dist = sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2));
    return dist;
}
void dis_direct(vector<new_pos> &gPath) {
    //Distance to go
    double d1 = getDist(gPath[0].x, gPath[1].x, gPath[0].y, gPath[1].y);
    double d2 = getDist(gPath[2].x, gPath[1].x, gPath[2].y, gPath[1].y);
    double d3 = getDist(gPath[0].x, gPath[2].x, gPath[0].y, gPath[2].y);
    //cout<<gPath[0].x<<"  "<<gPath[1].x<<"  "<<gPath[2].x<<"  "<<gPath[0].y<<"  "<<gPath[1].y<<"  "<<gPath[2].y<<endl;
    cout<<"Distance:  "<<d1<<"  "<<d2<<"  "<<d3<<endl;
    //Angle to rotate
    double alpha = acos((d2*d2 + d3*d3 - d1*d1) / (2 * d2 * d3)); alpha = alpha * 180.0 / PI;
    double beta = acos((d3*d3 + d1*d1 - d2*d2) / (2 * d3 * d1)); beta = beta * 180.0 / PI;
    cout<<endl<<"Angles:  "<<alpha<<"  "<<beta<<endl;
    double angle_to_rotate = (alpha + beta);
    d1 = d1 + d2 * cos(angle_to_rotate);

    //Direction to rotate
    string direction;
    double slope01 = (gPath[1].y - gPath[0].y) / (gPath[1].x - gPath[0].x);
    double slope12 = (gPath[2].y - gPath[0].y) / (gPath[2].x - gPath[0].x);
    if(slope01 >= slope12) {
        direction = "right";
    } else {
        direction = "left";
        //angle_to_rotate = 180 - angle_to_rotate;
    }

    //Generate message
    string text_msg = "Go forward " + to_string(int(round(d1))) + " units and turn " + to_string(int(angle_to_rotate)) + " degrees to " + direction;
    cout<<text_msg<<endl;

    //Get voice command
    //text_to_speech(text_msg);
}

void angle_correction() {

}
void text_to_speech(string text_msg) {

}

int main(int argc, char **argv){
	ros::init(argc, argv, "Text_Comm");
	ros::NodeHandle n;
	cout<<"checkpoint1"<<endl;
	//message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> PathSub(n, "/move_base/NavfnROS/plan", 10);
    //PathSub.registerCallback(callback);
	ros::Subscriber PathSub = n.subscribe("/move_base/NavfnROS/plan", 100, callback);
	ros::Rate loop_rate(10);
	
	ros::spin();
	return 0;
}



