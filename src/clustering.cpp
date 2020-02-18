#include <iostream>
#include <algorithm>
#include <vector>
#include <string>
#include <fstream>
#include <cmath>
#include <map>
#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;

const int NOISE = -2;
const int NOT_CLASSIFIED = -1;

class Point {
public:
    double x, y;
    int ptsCnt, cluster;
    double getDis(const Point & ot) {
        return sqrt((x-ot.x)*(x-ot.x)+(y-ot.y)*(y-ot.y));
    }
};

class DBCAN {
public:
    int n, minPts;
    double eps;
    vector<Point> points;
    int size;
    vector<vector<int> > adjPoints;
    vector<bool> visited;
    vector<vector<int> > cluster;
    int clusterIdx;
    
    DBCAN(int n, double eps, int minPts, vector<Point> points) {
//        this->n = n;
        this->eps = eps;
        this->minPts = minPts;
        this->points = points;
        this->size = (int)points.size();
        adjPoints.resize(size);
        this->clusterIdx=-1;
    }
    void run () {
        checkNearPoints();
        
		for(int i=0;i<size;i++) {
            if(points[i].cluster != NOT_CLASSIFIED) continue;
            
            if(isCoreObject(i)) {
                dfs(i, ++clusterIdx);
            } else {
                points[i].cluster = NOISE;
            }
        }
        
        cluster.resize(clusterIdx+1);
        for(int i=0;i<size;i++) {
            if(points[i].cluster != NOISE) {
                cluster[points[i].cluster].push_back(i);
            }
        }
    }
    
    void dfs (int now, int c) {
        points[now].cluster = c;
        if(!isCoreObject(now)) return;
        
        for(auto&next:adjPoints[now]) {
            if(points[next].cluster != NOT_CLASSIFIED) continue;
            dfs(next, c);
        }
    }
    
    void checkNearPoints() {
        for(int i=0;i<size;i++) {
            for(int j=0;j<size;j++) {
                if(i==j) continue;
                if(points[i].getDis(points[j]) <= eps) {
                    points[i].ptsCnt++;
                    adjPoints[i].push_back(j);
                }
            }
        }
    }
    // is idx'th point core object?
    bool isCoreObject(int idx) {
        return points[idx].ptsCnt >= minPts;
    }
    
    vector<vector<int> > getCluster() {
        return cluster;
    }
};


class InputReader {
private:
    ifstream fin;
    vector<Point> points;
	vector<float> idx_float;
	vector<float> x_float;
	vector<float> y_float;
public:
    InputReader(string filename) {
        fin.open(filename);
        if(!fin) {
            cout << filename << " file could not be opened\n";
            exit(0);
        }
        parse();
    }
    void parse() {
        int idx;
        double x, y;
        
		while(!fin.eof()) {
            fin >> idx >> x >> y;
            points.push_back({x,y,0, NOT_CLASSIFIED});

			idx_float.push_back(idx);
			x_float.push_back(x);
			y_float.push_back(y);

        }
        points.pop_back();
        
		idx_float.pop_back();
		x_float.pop_back();
		y_float.pop_back();

    }
	void clear(){
		idx_float.clear();
		x_float.clear();
		y_float.clear();

	}

    vector<Point> getPoints() {
        return points;
    }

	vector<float> getIDX(){
		return idx_float;
	}
	vector<float> getX(){
		return x_float;
	}
	vector<float> getY(){
		return y_float;
	}
};



class OutputPrinter {
private:
    ofstream fout;
    vector<vector<int> > cluster;
    vector<int> idx;
    string filename;
    int n;


public:
    OutputPrinter(int n, string filename, vector<vector<int> > cluster) {
        this->n = n;
        this->cluster = cluster;
        
        // remove ".txt" from filename
        if(filename.size()<4){
            cout << filename << "input file name's format is wrong\n";
            exit(0);
        }
        for(int i=0;i<4;i++) filename.pop_back();
        this->filename = filename;
        
        // sort by size decending order
        sort(cluster.begin(), cluster.end(), [&](const vector<int> i, const vector<int> j) {
            return (int)i.size() > (int)j.size();
        });
    }
    void print() {
        for(int i=0;i<n;i++) {
//            fout.open(filename+"_cluster_"+to_string(i)+".txt");
            
            for(int j=0;j<cluster[i].size();j++) {
//                fout << cluster[i][j] << endl;
				idx.push_back(cluster[i][j]);
			}
            
//            fout.close();
        }
//		for(int i=0;i<idx.size();i++){
//			cout << idx[i] << " ";
//		}
//		cout << endl;
	}

	vector<int> getClusteringID(){
		return idx;
	}
};

int main(int argc, char **argv) {
	ros::init(argc,argv,"dbscan_node");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<visualization_msgs::Marker>("/dbscan_pub",1);

	while(ros::ok()){
   
    vector<int> idx;
	vector<float> idx_float;
	vector<float> x_float;
	vector<float> y_float;
	
	int id = 0;

	visualization_msgs::Marker mk;
	geometry_msgs::Point pt;


// using input3.txt
	string inputFileName = "input3.txt";
	string n = "4";
	string eps = "3";
	string minPts = "5";

/*
// using input1.txt
	string inputFileName = "input1.txt";
	string n = "8";
	string eps = "10";
	string minPts = "6";
*/	
	InputReader inputReader(inputFileName);
    
    DBCAN dbScan(stoi(n), stod(eps), stoi(minPts), inputReader.getPoints());
    dbScan.run();
	
    OutputPrinter outputPrinter(stoi(n), inputFileName, dbScan.getCluster());
    outputPrinter.print();
    
	idx = outputPrinter.getClusteringID();
	idx_float = inputReader.getIDX();
	x_float = inputReader.getX();
	y_float = inputReader.getY();

	mk.header.frame_id = "/mk_frame";
	mk.type = visualization_msgs::Marker::SPHERE_LIST;
    mk.header.stamp = ros::Time::now();

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	mk.scale.x = 1.1;
	mk.scale.y = 1.1;
	mk.scale.z = 1.1;

	// Set the color -- be sure to set alpha to something non-zero!
	mk.color.r = 0.0f;
	mk.color.g = 1.0f;
	mk.color.b = 0.0f;
	mk.color.a = 1.0;

	for(int i=0;i<idx.size();i++){
//		cout << x_float[i] << " "<< y_float[i] <<endl;
		id = idx[i];
		pt.x = x_float[id];
		pt.y = y_float[id];
		mk.points.push_back(pt);
	}

	inputReader.clear();
	
	pub.publish(mk);
	printf("------------\n");
	ros::Rate r(0.1);
	ros::spinOnce();
	}

    return 0;
}

