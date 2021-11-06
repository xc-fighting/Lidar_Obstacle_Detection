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
                if( x < cur->point[1] ) {
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

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol) {
		std::vector<int> ids;
		return ids;
	}
	

};




