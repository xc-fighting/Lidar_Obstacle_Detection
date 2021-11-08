/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// binary search tree with even level split by x
// odd level split by y
// Structure to represent node of kd tree
struct Node
{
	//point[0] represent x, point[1] represent y
	std::vector<float> point;
	//unique id of the node
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree() {
		delete root;
	}
    
	//better to use iterative way
	void insertNode( float x, float y, int id ) {

		 std::vector<float> coordinate(2,0);
		 coordinate[0] = x;
		 coordinate[1] = y;
		 Node* newNode = new Node(coordinate, id);

         if( this->root == nullptr ) {
			 
			 this->root = newNode;
			 return;
		 }

		 Node* prev = nullptr;
		 Node* cur = root;
		 int level = 0;
		 int dir = -1;
		 while( cur != nullptr ) {
			 prev = cur;
             if( level % 2 == 0 ) {
				 //compare with x
				 if( x < cur->point[0] ) {
					 cur = cur->left;
					 dir = 0;
				 }
				 else {
					 cur = cur->right;
					 dir = 1;
				 }
			 }
			 else {
                if( y < cur->point[1] ) {
					cur = cur->left;
					dir = 0;
				}
				else {
					cur = cur->right;
					dir = 1;
				}
			 }
			 level++;
		 }
		
		 if( dir == 0 ) {
			 prev->left = newNode;
		 }
		 else {
			 prev->right = newNode;
		 }
         return;

	}
	void insert(std::vector<float> point, int id) {
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
        insertNode(point[0], point[1], id);
	}
    
	//solution 1, nearly search every node in the tree
	void searchNodes(std::vector<int>& candidateSet,Node* cur,
	                 int depth,std::vector<float> target, 
					 float distanceTol) {
		if( cur == nullptr ) {
			return;
		}
		float xmin = target[0] - distanceTol;
		float xmax = target[0] + distanceTol;
		float ymin = target[1] - distanceTol;
		float ymax = target[1] + distanceTol;
		float x2 = (cur->point[0] - target[0]) * (cur->point[0] - target[0]);
		float y2 = (cur->point[1] - target[1]) * (cur->point[1] - target[1]);
		float dis = sqrt(x2+y2);
		if( dis <= distanceTol ) {
			candidateSet.push_back(cur->id);
			searchNodes(candidateSet, cur->right, depth+1, target, distanceTol);
			searchNodes(candidateSet, cur->left, depth+1, target, distanceTol);
		}
		else {
              if( depth % 2 == 0 ) {
			    // check for x 
			   if( cur->point[0] < xmin ) {
				   searchNodes(candidateSet,cur->right,depth+1,target,distanceTol);
			   }
			   else if( cur->point[0] > xmax ) {
				searchNodes(candidateSet,cur->left, depth+1,target, distanceTol);
			   }
			
		    }
		   else {
            // check for x 
			if( cur->point[1] < ymin ) {
				searchNodes(candidateSet,cur->right,depth+1,target,distanceTol);
			}
			else if( cur->point[1] > ymax ) {
				searchNodes(candidateSet,cur->left, depth+1,target, distanceTol);
			}
			
		  }
		}
       
		return;
	}
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol) {
		std::vector<int> ids;
		searchNodes(ids,root,0,target,distanceTol);
		return ids;
	}
	

};




