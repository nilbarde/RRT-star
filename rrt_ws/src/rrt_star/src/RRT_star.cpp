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

bool checkMap = false;
float stepsize= 3;
float radius= 10;
float goal_radius= 4;
int RoadMapW, RoadMapH;
ros::Publisher chatter_pub;

float arr[2][4]= {{20, 60, 20, 50}, {50, 80, 50, 80}};
int s= sizeof(arr)/sizeof(arr[0]);


struct Point
{
	double x;
	double y;
};

struct Node
{

	float cost;
	Point P; 
	Node *parent;

};

int goalx = 5;
int goaly = 35;
int begx = 20;
int begy = 0;


Point gridSize = {RoadMapW, RoadMapH};

Point goal= {goalx, goaly};
Point beg= {begx, begy};

//int arr_1[gridx][gridy];
vector<vector<int> > obstacle_map;

/*for(int i=0; i<abs(goal.x-beg.x+1); i++)
	for(int j=0; j<abs(goal.y-beg.y+1))
		arr_1[i][j]= 0;
for(int i=0; i<s; i++)
{
	for(int j=arr[i][0]; j<=arr[i][1]; j++)
		for(int k=arr[i][2]; k<=arr[i][3]; j++)
			arr_1[i][j]= 1;
}*/



// Point x_centre= {(goal.x+beg.x)/2, (goal.y+beg.y)/2};


// float c_min= sqrt((goal.x-beg.x)*(goal.x-beg.x) + (goal.y-beg.y)*(goal.y-beg.y));
// float C[2][2]= {{(goal.x-beg.x)/c_min, (goal.x-beg.x)/c_min}, {(goal.y-beg.y)/c_min, (goal.y-beg.y)/c_min}} ;


class RRT
{
public:
	Node* start = new Node;
	// start = &dd;
	vector<Node*> all_nodes;
	Node* final = new Node;
	Node sstart;
	RRT()
	{
		
		
		//cout<<"aa "<<(*start).P.x<<endl;
		//cout<<start->P.x<<endl;
		
		//cout<<sstart.P.x<<endl;
		(*start).P.x = beg.x;
		
		(*start).P.y = beg.y;
		
		start->parent= NULL;
		
		start->cost= 0;
		
		all_nodes.push_back(start);
		
		final->parent= NULL;
		
	}
	Node* generate_randnode()
	{

		// rand();
		// int asdfghjkhgfdsa = 1;
		Point gridSize = {RoadMapW, RoadMapH};
		//cout<<4444444<<endl;
		//cout<<gridSize.x<<","<<gridSize.y<<endl;
		Point p= {(rand() % int(gridSize.x)) +0, ((rand() % int(gridSize.y+0)))+0};
		bool a= true;
		//cout<<5555555<<endl;
		/*cout<<"sssssss"<<endl;
		for(int i= 0; i< all_nodes.size(); i++)
		{
			//cout<<"sssssss"<<endl;
			if(((p.x= (all_nodes[i]->P).x) && (p.y= (all_nodes[i]->P).y)))
				a= false;
				
		}*/
		if(a)
		{
			Node* rand_node= new Node;
			rand_node-> P= p;
			rand_node-> parent= NULL;
			rand_node->cost= 0;
			return rand_node;
		}
		else
			generate_randnode();
	}
		


	/*float** multiply_matrices(float a[2][2], float b[2][2])
	{
		// float** c;
		float** c= new float*[2];
		c[0]= new float[2];
		c[1]= new float[2];
		c[0][0]= a[0][0]*b[0][0] + a[0][1]*b[1][0];
		c[0][1]= a[0][0]*b[0][1] + a[0][1]*b[1][1];
		c[1][0]= a[1][0]*b[0][0] + a[1][1]*b[1][0];
		c[1][1]= a[1][0]*b[0][1] + a[1][1]*b[1][1];
		return c;
	}
	float* multiply_matr_vec(float a[2][2], float b[2])
	{
		float c_1[2];
		c_1[0]= a[0][0]*b[0] + a[0][1]*b[1];
		c_1[1]= a[1][0]*b[0] + a[1][1]*b[1];
		return c_1;
	}
	Node* sample(float c_best)
	{
		float r= (float) rand()/RAND_MAX;
		float theta= (float) rand()/RAND_MAX* 2*22/7;
		float s_1[2];
		s_1[0]= r*cos(theta);
		s_1[1]= r*sin(theta);
		float L[2][2]= {{0,0}, {0,0}};
		L[0][0]= c_best/2;
		L[1][1]= 0.5*sqrt(c_best*c_best - c_min*c_min);
		float** k=multiply_matrices(C, L);
		
		float* p= multiply_matr_vec(&&k, s_1);
		Point n;
		n.x= p[0]+x_centre.x;
		n.y= p[1]+x_centre.y;
		Node* rand_1;
		rand_1->P= n;
		if((obstacle_check(rand_1)) && n.x<= gridSize.x && n.y<= gridSize.y)
		{
			rand_1->parent= NULL;
			return rand_1;
		}
		else
			sample(c_best);
	}*/

	vector<Node*> near_nodes(Node* N)
	{
		//cout<<12345<<endl;
		vector<Node*> nearby_nodes;
		int e=1;
		for(int i=0; i< all_nodes.size(); i++)
		{
			if(distance(all_nodes[i]->P, N->P)< radius)
			{
				// cout<<"boom "<<all_nodes[i]->P.x<<" "<<all_nodes[i]->P.y<<endl;
				nearby_nodes.push_back(all_nodes[i]);
			}
			if(all_nodes[i]==N)
			{
				//cout<<i;
				e=i;
			}
		}
		//cout<<endl<<3<<endl;
		//nearby_nodes.erase(nearby_nodes.begin()+ e);
		//cout<<nearby_nodes.size()<<endl<<"11236432875260746597284343798247897432894890952"<<endl;
		return nearby_nodes;
	}



	// bool obstacle_check_1(Node* N)
	// {
	// 	Point p= N->P;
	// 	int a, b;
	// 	a= p.x;
	// 	b= p.y;
	// 	int bb = 1, aa = 1;
	// 	if(b>99)
	// 	{
	// 		b = 99;
	// 	}
	// 	for(int i=0; i<s; i++)
	// 	{
	// 		if((p.x>=(arr[i][0]-0)&&p.x<=(arr[i][1]+0)) && (p.y>=(arr[i][2]-0)&&p.y<=(arr[i][3]+0)))
	// 		{
	// 			bb = 0;
	// 			// return false;
	// 		}
	// 	}
	// 	if(arr_1[a][b]==1)
	// 	{
	// 		aa = 0;
	// 	}
	// 	if(aa!=bb)
	// 	{

	// 		cout<<"booooooom "<<aa<<bb<<endl;
	// 	}
	// 	if(!(a<100 and a>=0 && b>=0 && b<100))
	// 	{
	// 		cout<<"asdfghj "<<a<<" "<<b<<"\n";
	// 	}
	// 	return bb;
	// }

	// bool obstacle_check_2(Node* N)
	// {
	// 	Point p= N->P;
	// 	for(int i=0; i<s; i++)
	// 	{
	// 		if((p.x>=(arr[i][0]-1)&&p.x<=(arr[i][1]+1)) && (p.y>=(arr[i][2]-1)&&p.y<=(arr[i][3]+1)))
	// 			return false;
	// 	}
	// 	return true;
	// }

	bool obstacle_check(Node* N)
	{
		Point p= N->P;
		int a, b;
		a= p.x;
		b= p.y;
		if(obstacle_map[a][b]>=40)
		{
			cout<<"%%%%%%%%%%%%%%%%%%%%"<<endl;
			return false;
		}
		return true;
	}


	bool linejoin_check(Node* a, Node*b)
	{
		Point A= a->P;
		Point B= b->P;
		Node* N_1= new Node;
		Node* N_2= new Node;
		Node* N_3= new Node;
		Point n_1= {0.5*(A.x+B.x), 0.5*(A.y+B.y)};
		Point n_2= {0.5*(A.x+n_1.x), 0.5*(A.y+n_1.y)};
		Point n_3= {0.5*(B.x+n_1.x), 0.5*(B.y+n_1.y)};
		N_1->P= n_1;
		N_2->P= n_2;
		N_3->P= n_3;
		if(!(obstacle_check(N_1)))
			return 0;
		else if(!(obstacle_check(N_2)))
			return 0;
		else if(!(obstacle_check(N_3)))
			return 0;
		else
			return 1;
	}


	Node* nearest_node(Node* N)
	{
		Node* nearest= new Node;
		float d= 1000000;
		float m;
		//vector<Node*> all_= near_nodes(N);

		for(int i=0; i< all_nodes.size(); i++)
		{
			m= distance(all_nodes[i]->P, N->P);
			if(m<d)
			{
				d= m;
				nearest= all_nodes[i];
			}	
		}
		return nearest;
	}


	Node* new_node(Node* nearest, Node* N)
	{
		Node* new_node= new Node;
		float s= stepsize;
		Point m;
		//cout<<distance(nearest->P, N->P)<<endl;
		if(linejoin_check(nearest, N))
		{
			if((distance(N->P, nearest->P)< s))
			{
				new_node->P= N->P;
				new_node->cost= nearest->cost+ distance(N->P, nearest->P);
				new_node->parent= nearest;
				all_nodes.push_back(new_node);
				return new_node;
			}
			else
			{
				m.x= 1/(distance(N->P, nearest->P))*(s*((N->P).x) + (distance(N->P, nearest->P) - s)*((nearest->P).x));
				m.y= 1/(distance(N->P, nearest->P))*(s*((N->P).y) + (distance(N->P, nearest->P) - s)*((nearest->P).y));
				new_node->P= m;
				new_node->cost= nearest->cost + s;
				new_node->parent= nearest;
				all_nodes.push_back(new_node);
				return new_node;
			}
		}
		//cout<<nearest->P.x<<endl<<nearest->P.y<<endl;
		//cout<<"00000000000000"<<endl<<m.x<<endl<<m.y<<endl;
		int a= 1;
			



		/*while(a && s> 0.01*stepsize)
		{
			if(!obstacle_check(new_node))
			{
				
				(new_node->P).x= 0.9*(new_node->P).x+ 0.1*(nearest->P).x;
				(new_node->P).y= 0.9*(new_node->P).y+ 0.1*(nearest->P).y;
				s*= 0.9;   
				//cout<<new_node->P.x<<endl<<new_node->P.y<<endl<<endl;
			}
			else
			{
				a= 0;
				new_node->cost= nearest->cost+ s;
				new_node->parent= nearest;
				all_nodes.push_back(new_node);
				return new_node;
			}
		}*/






		/*if(a==0)
		{
			new_node->cost= nearest->cost+ s;
			new_node->parent= nearest;
			all_nodes.push_back(new_node);
			return new_node;
		}*/

			//cout<<a<<"%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"<<endl;
			return NULL;
		
	}


	float distance(Point a, Point b)
	{
		return sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y));
	}




	Node* best_parent(Node* N)
	{
		vector<Node*> q= near_nodes(N);
		//cout<<"6666666666"<<q.size()<<endl;
		Node* best= new Node;
		best->cost = 10000000;
		float best_cost = best->cost;
		float now_cost = best->cost;
		// cout<<"innnnnnnnnnnnnnn"<<endl;
		for(int i= 0; i<q.size(); i++)
		{
			//cout<<"cost"<<" "<<q[i]->cost<<endl;
			if(!(q[i]== N))
			{
				//cout<<"shoot "<<q[i]->cost<<endl;
				//+ distance(q[i]->P, N->P)
				now_cost = (q[i]->cost + distance(q[i]->P, N->P));
				if(now_cost < best_cost)
				{
					//cout<<"in here"<<q[i]->P.x<<" "<<q[i]->P.y<<" "<<linejoin_check(q[i], N)<<endl;
					
					if((linejoin_check(q[i], N)))
					{
						best= q[i];
						// cout<<"asdfghjkl"<<endl;
						best_cost = now_cost;
						// best->cost = best_cost;
					}
				}
			}
		}
		// cout<<"fu"<<endl;
		if(!(q.size()==1))
		{
			// cout<<"in here"<<best->P.x<<" "<<best->P.y<<endl;
			// cout<<"8888888888888"<<endl;
			N->cost= best_cost;
			N->parent= best;
			return best;
		}
		else
		{
			// cout<<"1111111111111"<<endl;
			return N->parent;
		}	
	}

	void rewire(Node* N)
	{
		vector<Node*> w= near_nodes(N);


		if(!(final->parent== NULL) && (distance(final->P, N->P)<radius))
			w.push_back(final);
		//cout<<w.size();
		Node* bestparent_n= best_parent(N);


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
				

				//cout<<"@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@";
				if((linejoin_check(N, w[i])))
				{
					w[i]->parent= N;
					w[i]->cost= N->cost+ distance(N->P, w[i]->P);
				}

			}
		}

	}

	bool goal_check(Node* N, Point goal)
	{
		if(distance(N->P, goal) < goal_radius)
		{
			final->parent= N;
			final->P= goal;
			final->cost= N->cost+ distance(N->P, goal);
			return true;
		}
		return false;
	}

	vector<Node*> shortest_path(Node* start, Node* final)
	{
		vector<Node*> path;
		
		Node* a= final;
		int i=1;
			while(!(a->parent== NULL))
			{
				//cout<<i<<endl;

				path.push_back(a);
				
				//cout<<path.size()<<endl;
				if((a->parent==start))
					path.push_back(start);
				a= a->parent;
				
				i++;
				
			}
			
			
			//path.push_back(start->P);
		return path;
	}


};



vector<Node*> find_path()
{
	//cout<<3333333333333<<endl;
	RRT rrt;	
	bool c= 0;

	int iter_max= 3000;
	vector<Node*> Path_nodes;
	vector<Point> random_nodes;
	Node* rand_n= new Node;
	Node* nearest_1= new Node;
	Node* new_n= new Node;

	int check= 1;


	for(int i= 1; i<iter_max; i++)
	{
		
		//cout<<333333333564333<<endl;

		rand_n= rrt.generate_randnode();
		//cout<<3333333333333<<endl;
		random_nodes.push_back(rand_n->P);
		nearest_1= rrt.nearest_node(rand_n);
		
		new_n= rrt.new_node(nearest_1, rand_n);
		if(!(new_n==NULL))
		{
			

			c= !(rrt.final->parent== NULL);
			//cout<<c<<endl;

			if(c==1)
			{
				
				rrt.best_parent(new_n);
				rrt.rewire(new_n);
				if(check==1)
				{
					if(rrt.goal_check(new_n, goal))
					{
						check=0;
					}
				
				}
				Path_nodes= rrt.shortest_path(rrt.start, rrt.final);
				
			}
			else
			{
				rrt.best_parent(new_n);				
				rrt.rewire(new_n);
				if(check==1)
				{
					if(rrt.goal_check(new_n, goal))
					{
						check=0;
					}
				}
			}
		}


		

	}
	float q= 0;

	vector<Point> Path;

	for(int i=0; i< Path_nodes.size(); i++)
	{
		Path.push_back(Path_nodes[i]->P);
	}

	 //for(int i=0;i<Path.size();i++)
	 //{
	 	//cout<<Path[i].x<< ", "<< Path[i].y<< endl;

	 //}

	// cout<<5555555<<endl;

	// vector<Node*> all_1= rrt.all_nodes;
	 
	//  cout<<rrt.linejoin_check(rrt.start, rrt.final)<<endl<<666<<endl;;

	 

	 // for(int i=0;i<Path.size();i++)
	 // {
	 // 	cout<<Path[i].x<< ", "<< Path[i].y<< endl<<Path_nodes[i]->cost<<endl;
	 // 	if(i>0)
	 // 		cout<<rrt.distance(Path[i], Path[i-1])<<endl<<endl;

	 // }

	  cout<<endl<<rrt.final->cost<<endl;
	 
	 
	 
	 
	 int s=1;
	 return Path_nodes;

}


void publisher(vector<Point> Path)
{
	
	RRT a_1;
	ros::NodeHandle v;
		  	 //ros::Publisher chatter_pub= v.advertise< nav_msgs/Path >("PATH", 1000);
		  	ros::Publisher chatter_pub = v.advertise<nav_msgs::Path>("PATH", 1);
		  	 // ros::Publisher map_pub = v.advertise<nav_msgs::OccupancyGrid>("MyGrid", 1);

		  	nav_msgs::Path poses;
		  	poses.header.frame_id = "map";
		  
		  	// nav_msgs::OccupancyGrid MyGrid;
		  
		  	// MyGrid.header.stamp = ros::Time::now();
		  	// MyGrid.header.frame_id = "map";
		  	// MyGrid.info.resolution = 1.0;
		  	// MyGrid.info.origin.position.x = 0.0;
		  	// MyGrid.info.origin.position.y = 0.0;
		  	// MyGrid.info.origin.position.z = 0.0;
		  	// MyGrid.info.origin.orientation.x = 0.0;
		  	// MyGrid.info.origin.orientation.y = 0.0;
		  	// MyGrid.info.origin.orientation.z = 0.0;
		  	// MyGrid.info.origin.orientation.w = 0.0;
		  	// MyGrid.info.width = RoadMapW;
		  	// MyGrid.info.height = RoadMapH;
		  	// MyGrid.info.map_load_time = ros::Time::now();
		  


		  	// bool b= true;
		  	// // int xcvb = 0;
		  	// for(int i=0;i<RoadMapW;i++)
		  	// {
		  	// 	for(int j=0;j<RoadMapH;j++)
		  	// 	{

		  	// 		Node* e = new Node;
		  	// 		//cout<<"$$$$"<<endl;
		  	// 		Point abc = {i, j};
		  	// 		e->P.x= i;
		  	// 		e->P.y= j;
		  	// 		//cout<<99999<<endl;
		  
		  	// 			if(!a_1.obstacle_check(e))
		  	// 			{
		  	// 				// xcvb++;
		  	// 				MyGrid.data.push_back(100);
		  	// 				break;
		  	// 			}
		  			
		  	// 		if(b)
		  	// 		{
		  	// 			// xcvb++;
		  	// 			MyGrid.data.push_back(0);
		  	// 		}
		  	// 	}
		  	// }



		  	// cout<<xcvb<<" asd"<<endl;
		  	 // while (ros::ok())
		  	 {
		  	 	poses.poses.clear();
		  		// cout<<count<<endl;
		  
		  		for(int i=0; i<Path.size(); i++)
		  		{
		  			geometry_msgs::PoseStamped vertex;
		  			vertex.pose.position.x= Path[i].x;
		  			vertex.pose.position.y= Path[i].y;
		  			vertex.pose.position.z= 0;
		  
		  		
		  
		  		
		  			poses.poses.push_back(vertex);
		  		}
		  
		  		chatter_pub.publish(poses);
		  		//map_pub.publish(MyGrid);
		  
		  
		      	//count++;
		  		
		  	 }  
}



void RoadCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	//cout<<666666<<endl;
	if(!msg->data.empty())
	{
		checkMap = true;

		int i,j;
		RoadMapW = msg->info.width;
		RoadMapH = msg->info.height;
		//begx = RoadMapW/2;
		//begy = 0;
		//goalx = 20;
		//goaly = 20;

		std::vector<int> v;
		for (i=0; i<RoadMapW; i++) v.push_back(0);
		for (i=0; i<RoadMapH; i++) obstacle_map.push_back(v);

			//cout << "2\n";

		for(i=0;i<RoadMapH;i++)//putting values
		{
			for(j=0;j<RoadMapW;j++)
			{
				// int i1;
				// i1=RoadMapH-1-i;
				// cout << "data: " << msg->data[i*RoadMapW+j] << endl;
				if(msg->data[i*RoadMapW+j]>=40)
				{
					obstacle_map[i][j] = int(msg->data[i*RoadMapW+j]);
					// cout<<int(msg->data[i*RoadMapW+j])<<endl;
				}
			}
		}

			// for (int i : msg->data)
			// 	if(i!=0)
			// 	{cout << "data: "<<i << "YAY"<< endl;
			// 	}

		// for(int i = 0; i<RoadMapW; i++)
		// {
		// 	for(int j= 0; j<RoadMapH;j++)
		// 	{
		// 		cout<<obstacle_map[i][j]<<", ";
		// 	}
		// 	cout<<endl;
		// }

		vector<Point> Path;
 	//cout<<"###"<<checkMap<<endl;
	}

  
	if(checkMap == true )
		{

		  	//cout<<2222;
		  	vector<Node*> Path_nodes= find_path();
		  	//cout<<"%%%%%%%%%%%%%%%%%"<<endl;
		  	//cout<<555<<endl<<Path_nodes.size()<<endl;
		  	
		  	for(int i=0; i< Path_nodes.size(); i++)
		  	{
		  		Path.push_back(Path_nodes[i]->P);
		  	}
		}

		 


		publisher(Path);
	

		//cout<<RoadMapW<<","<<RoadMapH<<endl<<endl;
		//cout << "3\n";
}


int main(int argc, char **argv)
{
	/*for(int i=0; i<=gridx; i++)
		for(int j=0; j<=gridy;j++)
			arr_1[i][j]= 0;
	for(int i=0; i<s; i++)
	{
		for(int j=arr[i][0]-1; j<arr[i][1]+1; j++)
		{
			for(int k=arr[i][2]-1; k<arr[i][3]+1; k++)
			{
				if(j>=0 and j<100 and k>=0 and k<100)
					arr_1[j][k]= 1;
			}
		}
	}*/

	RRT a_1;

	//cout<<checkMap<<endl;
	ros::init(argc, argv, "rrt");

	ros::NodeHandle LRSubNode, MapPubNode;
	ros::Subscriber SubRoad = LRSubNode.subscribe<nav_msgs::OccupancyGrid>("occ_map", 1, RoadCallback);

	chatter_pub = MapPubNode.advertise<nav_msgs::Path>("PATH", 1);;

	
 //  	vector<Point> Path;
 // 	//cout<<"###"<<checkMap<<endl;
  
	// if(checkMap == true )
	// 	{
	// 	  	//cout<<2222;
	// 	  	vector<Node*> Path_nodes= find_path();
	// 	  	//cout<<555<<endl<<Path_nodes.size()<<endl;
		  	
	// 	  	for(int i=0; i< Path_nodes.size(); i++)
	// 	  	{
	// 	  		Path.push_back(Path_nodes[i]->P);
	// 	  	}
	// 	  }
	// 	  		int count= 0;
		  
		  	 
		  
	// 	  	 ros::NodeHandle v;
	// 	  	 //ros::Publisher chatter_pub= v.advertise< nav_msgs/Path >("PATH", 1000);
	// 	  	 ros::Publisher chatter_pub = v.advertise<nav_msgs::Path>("PATH", 1);
	// 	  	 ros::Publisher map_pub = v.advertise<nav_msgs::OccupancyGrid>("MyGrid", 1);
		  
	// 	  	nav_msgs::Path poses;
	// 	  	poses.header.frame_id = "map";
		  
	// 	  	nav_msgs::OccupancyGrid MyGrid;
		  
	// 	  	MyGrid.header.stamp = ros::Time::now();
	// 	  	MyGrid.header.frame_id = "map";
	// 	  	MyGrid.info.resolution = 1.0;
	// 	  	MyGrid.info.origin.position.x = 0.0;
	// 	  	MyGrid.info.origin.position.y = 0.0;
	// 	  	MyGrid.info.origin.position.z = 0.0;
	// 	  	MyGrid.info.origin.orientation.x = 0.0;
	// 	  	MyGrid.info.origin.orientation.y = 0.0;
	// 	  	MyGrid.info.origin.orientation.z = 0.0;
	// 	  	MyGrid.info.origin.orientation.w = 0.0;
	// 	  	MyGrid.info.width = RoadMapW;
	// 	  	MyGrid.info.height = RoadMapH;
	// 	  	MyGrid.info.map_load_time = ros::Time::now();
		  
	// 	  	bool b= true;
	// 	  	// int xcvb = 0;
	// 	  	for(int i=0;i<RoadMapW;i++)
	// 	  	{
	// 	  		for(int j=0;j<RoadMapH;j++)
	// 	  		{
	// 	  			Node* e;
	// 	  			e->P.x= i;
	// 	  			e->P.y= j;
		  
	// 	  				if(!a_1.obstacle_check(e))
	// 	  				{
	// 	  					// xcvb++;
	// 	  					MyGrid.data.push_back(100);
	// 	  					break;
	// 	  				}
		  			
	// 	  			if(b)
	// 	  			{
	// 	  				// xcvb++;
	// 	  				MyGrid.data.push_back(0);
	// 	  			}
	// 	  		}
	// 	  	}
	// 	  	// cout<<xcvb<<" asd"<<endl;
	// 	  	 // while (ros::ok())
	// 	  	 {
	// 	  	 	poses.poses.clear();
	// 	  		 cout<<count<<endl;
		  
	// 	  		for(int i=0; i<Path.size(); i++)
	// 	  		{
	// 	  			geometry_msgs::PoseStamped vertex;
	// 	  			vertex.pose.position.x= Path[i].x;
	// 	  			vertex.pose.position.y= Path[i].y;
	// 	  			vertex.pose.position.z= 0;
		  
		  		
		  
		  		
	// 	  			poses.poses.push_back(vertex);
	// 	  		}
		  
	// 	  		chatter_pub.publish(poses);
	// 	  		map_pub.publish(MyGrid);
		  
		  
	// 	      	count++;
		  		
	// 	  	 }  
		  	 
				ros::Rate loop_rate(10);
				ros::spin();
		  
				loop_rate.sleep();

	return 0;
}