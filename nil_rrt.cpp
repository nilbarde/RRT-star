#include<iostream>
#include<vector>
#include<stdlib.h>
#include<time.h>
#include<math.h>
#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
// #include <typeinfo>

using namespace std;

void print(string x)
{
    cout<<x<<endl;
}

std::vector < std::vector < int > > mapObst;
int mapW = 40;
int mapH = 40;
int mapRes = 0.05;

float stepSize= 4;
float radius= 4;
float goalRadius= 3;

ros::Publisher pubLocalMap, pubPath;

struct Point
{
	double x;
	double y;
};

struct Node
{
	float cost;
	Point P; 
	Node* parent;
};

vector<Point> nowPath;
Point pointStart = {mapW/2,0};
Point pointEnd = {mapW-2,mapH-2};

class RRT
{
    public:
    Node* start = new Node;
    vector<Node*> allNodes;
    Node* stop = new Node;

	RRT()
	{
        print("-------------------");
		(*start).P.x = pointStart.x;		
		(*start).P.y = pointStart.y;
		// pointStart.x = (*start).P.x;		
		// pointStart.y = (*start).P.y;
		start->parent= NULL;
		start->cost= 0;

		(*stop).P.x = pointEnd.x;		
		(*stop).P.y = pointEnd.y;
		stop->parent= NULL;
		stop->cost= 0;

		allNodes.push_back(start);
		
		stop->parent= NULL;
	}

    Node* getRandNode()
    {
    	 
        //Point rPoint = {(rand() % mapH),(rand() % mapW)};
        float thetaMax = 45;
        int n = 2*thetaMax/0.1;
        float theta[n];
        //float theta = (float) rand() / (float) RAND_MAX;
        for(int i=0; i<n; i++)
        {
        	theta[i] = -thetaMax + i*0.1;
        }

        vector<float> weights(n);
        int j;
        for(int i=0; theta[i] != 0; i++)
        {
        	weights[i] = exp(i);
        	j = i+1;
        } 
        for(int i=j; i<n; i++)
        {
        	weights[i] = weights[j-1];
        	j--;
        }

        std::random_device rd;
    	std::mt19937 gen(rd());

    	std::discrete_distribution<> d(weights.begin(), weights.end());
    	float Theta = theta[d(gen)];
    	float rMax;
    	if(abs(Theta)>)



        Node* rNode = new Node;
        rNode-> P = rPoint;
        rNode-> parent = NULL;
        rNode-> cost = 0;
        return rNode;
    }

	Node* newNode(Node* nearest, Node* N)
	{
		Node* new_node = new Node;
		float s = stepSize;
		Point m;
		if(checkLineJoin(nearest, N))
		{
			if((distance(N->P, nearest->P)< s))
			{
				new_node->P= N->P;
				new_node->cost= nearest->cost+ distance(N->P, nearest->P);
				new_node->parent= nearest;
				if (!(isObstacle(new_node)))
                {
                	allNodes.push_back(new_node);
                    return new_node;
                }
                else 
                {
                    return NULL;
                }
			}
			else
			{
				m.x= 1/(distance(N->P, nearest->P))*(s*((N->P).x) + (distance(N->P, nearest->P) - s)*((nearest->P).x));
				m.y= 1/(distance(N->P, nearest->P))*(s*((N->P).y) + (distance(N->P, nearest->P) - s)*((nearest->P).y));
				new_node->P= m;
				new_node->cost= nearest->cost + s;
				new_node->parent= nearest;
				if (!(isObstacle(new_node)))
                {
                	allNodes.push_back(new_node);
                    return new_node;
                }
                else 
                {
                    return NULL;
                }
			}
		}
	    return NULL;
		
	}

	Node* getNearestNode(Node* N)
	{
		Node* nearest = new Node;
		float d= 1000000;
		float m;
		//vector<Node*> all_= near_nodes(N);

		for(int i=0; i< allNodes.size(); i++)
		{
			m= distance(allNodes[i]->P, N->P);
			if(m<d)
			{
				d = m;
				nearest= allNodes[i];
			}	
		}
		return nearest;
	}

	Node* bestParent(Node* N)
	{
		vector<Node*> q= getNearNodes(N);
		Node* best= new Node;
		best->cost = 10000000;
		float best_cost = best->cost;
		float now_cost = 0;
		for(int i= 0; i<q.size(); i++)
		{
			if(!(q[i]== N))
			{
				now_cost = (q[i]->cost + distance(q[i]->P, N->P));
				if(now_cost < best_cost)
				{
					if((checkLineJoin(q[i], N)))
					{
						best= q[i];
						best_cost = now_cost;
					}
				}
			}
		}
		if(!(q.size()==1))
		{
			N->cost= best_cost;
			N->parent= best;
			return best;
		}
		else
		{
			return N->parent;
		}	
	}

    vector<Node*> getNearNodes(Node* N)
    {
        vector<Node*> nearNodes;
        for(int i=0;i<allNodes.size();i++)
        {
            if(distance(allNodes[i]->P, N->P) < radius)
            {
                nearNodes.push_back(allNodes[i]);
            }
        }
        return nearNodes;
    }

	vector<Node*> shortestPath(Node* first, Node* final)
	{
		vector<Node*> path;

		Node* a= final;
		// cout<<a->P.x<<endl;
		int i=1;
        while(!(a->parent== NULL))
        {
            path.push_back(a);
            a= a->parent;
            
            i++;
        }
        path.push_back(first);
		return path;
	}

	bool goalCheck(Node* N, Point goal)
	{
		if(distance(N->P, goal) < goalRadius)
		{
			stop->parent= N;
			stop->P= goal;
			stop->cost= N->cost+ distance(N->P, goal);
			return true;
		}
		return false;
	}

    bool isObstacle(Node* N)
    {
        // return 0;
        int i = N->P.x;
        int j = N->P.y;
		// cout<<mapObst[i][j]<<endl;
        if(mapObst[i][j]==1)
        {
            return 1;
        }
        return 0;
    }

	bool checkLineJoin(Node* a, Node* b)
	{
		Point A= a->P;
		Point B= b->P;
		int l = 20;
		for (int i=1;i<l;i++)
		{
			// cout<<i<<endl;
			Node* N_1= new Node;
			float c_z = (float)(i)/l;
			Point n_1= {((c_z)*A.x+(1-c_z)*B.x),((c_z)*A.y+(1-c_z)*B.y)};
			N_1->P= n_1;
			if(!(isObstacle(N_1)))
			{
				return 1;
			}
		}
		return 0;
	}

	void rewire(Node* N)
	{
		vector<Node*> w= getNearNodes(N);

		if(!(stop->parent== NULL) && (distance(stop->P, N->P)<radius))
			w.push_back(stop);
		Node* bestparent_n = bestParent(N);
		for(int i=0; i<w.size(); i++)
		{
			if(bestparent_n== w[i])
			{
				w.erase(w.begin()+ i);
				break;
			}
		}
		for(int i=0; i<w.size(); i++)
		{
			if((N->cost+ distance(N->P, w[i]->P)) < w[i]->cost)
			{
				if((checkLineJoin(N, w[i])))
				{
					w[i]->parent = N;
					w[i]->cost = N->cost+ distance(N->P, w[i]->P);
				}
			}
		}

	}

	float distance(Point a, Point b)
	{
		return sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y));
	}
};

vector<Point> findPath()
{
	RRT rrt;	
	bool c= 0;

	int iter_max= 15000;
	vector<Node*> Path_nodes;
	// vector<Point> random_nodes;
	Node* rand_n= new Node;
	Node* nearest_1= new Node;
	Node* new_n= new Node;

	int check= 1;

	for(int i= 1; i<iter_max; i++)
	{
		rand_n = rrt.getRandNode();
		// random_nodes.push_back(rand_n->P);
		nearest_1 = rrt.getNearestNode(rand_n);
		new_n = rrt.newNode(nearest_1, rand_n);

		if(!(new_n==NULL))
		{
			c = !(rrt.stop->parent==NULL);
            rrt.bestParent(new_n);
            rrt.rewire(new_n);
            if(check==1)
            {
                if(rrt.goalCheck(new_n, pointEnd))
                {
                    check=0;
                }
            }
			if(c==1)
			{
				Path_nodes = rrt.shortestPath(rrt.start, rrt.stop);
			}
		}
	}

	for(int i=0; i< Path_nodes.size(); i++)
	{
        cout<<Path_nodes[i]->P.x<<" "<<Path_nodes[i]->P.y<<endl;
		// Path.push_back(Path_nodes[i]->P);
	}
	vector<Point> Path;
	for(int i=0;i<Path_nodes.size();i++)
	{
		Path.push_back(Path_nodes[i]->P);
	}
	return Path;
}

void mapInit()
{
	mapObst.clear();
    std::vector<int> v;
    for (int i=0; i<mapW; i++) v.push_back(0);
    for (int i=0; i<mapH; i++) mapObst.push_back(v);
}

void mapObstDraw()
{
    for(int i=15;i<25;i++)
    {
        for(int j=15;j<25;j++)
        {
            mapObst[i][j] = 1;
        }
    }
    for (int i=0;i<mapH;i++)
    {
        for(int j=0;j<mapW;j++)
        {
            cout<<mapObst[i][j]<<" ";
        }
        cout<<endl;
    }
    // for(int i=20;i<30;i++)
    // {
    //     for(int j=20;j<30;j++)
    //     {
    //         mapObst[i][j] = 1;
    //     }
    // }
}

void mapPublish()
{
	nav_msgs::OccupancyGrid mapGrid;
	mapGrid.header.stamp = ros::Time::now();
	mapGrid.header.frame_id = "/ego_vehicle";
	mapGrid.info.resolution = mapRes;
	mapGrid.info.origin.position.x = 0.0;
	mapGrid.info.origin.position.y = 0.0;
	mapGrid.info.origin.position.z = 0.0;
	mapGrid.info.origin.orientation.x = 0.0;
	mapGrid.info.origin.orientation.y = 0.0;
	mapGrid.info.origin.orientation.z = 0.0;
	mapGrid.info.origin.orientation.w = 1.0;
	mapGrid.info.width = mapW;
	mapGrid.info.height = mapH;
	mapGrid.info.map_load_time = ros::Time::now();

	for(int i=0;i<mapH;i++)
	{
		for(int j=0;j<mapW;j++)
		{
			mapGrid.data.push_back(mapObst[j][i]*100);
		}
	}

    pubLocalMap.publish(mapGrid);
    cout<<"publishing\n";
}

void pathPublish(vector<Point> Path)
{
    nav_msgs::Path poses;
    poses.header.frame_id = "map";
    poses.poses.clear();

    for(int i=0; i<Path.size(); i++)
    {
        geometry_msgs::PoseStamped vertex;
        vertex.pose.position.x= Path[i].x;
        vertex.pose.position.y= Path[i].y;
        vertex.pose.position.z= 0;

        poses.poses.push_back(vertex);
    }

    pubPath.publish(poses);

}

void callbackRoad(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	if(!msg->data.empty())
	{

		int i,j;
		mapW = msg->info.width;
		mapH = msg->info.height;
		mapRes = msg->info.resolution;
		mapInit();

		for(i=0;i<mapH;i++)
		{
			for(j=0;j<mapW;j++)
			{
				if(msg->data[i*mapW+j]>=40)
				{
					mapObst[i][j] = 1;
				}
			}
		}
	}
}

int main(int argc, char **argv)
{
    print("started hardik");
	mapInit();
	// mapObstDraw();
	cout<<"("<<pointStart.x<<", "<<pointStart.y<<")"<<endl;

	ros::init(argc,argv,"rrt");

	ros::NodeHandle mapPubNode, LRSubNode;

	//pubLocalMap = mapPubNode.advertise<nav_msgs::OccupancyGrid>("/scan/local_map",1);
	ros::Subscriber subRoad = LRSubNode.subscribe<nav_msgs::OccupancyGrid>("/scan/blown_local_map",1,callbackRoad);
	pubPath = mapPubNode.advertise<nav_msgs::Path>("PATH", 1);

	ros::Rate RosLoopRate(10);
	while(ros::ok())
	{
		ros::spinOnce();//check for incoming messages
		RosLoopRate.sleep();
        pathPublish(findPath());
        // mapPublish();
	}
    cout<<"doing great\n";

	return 0; 
}