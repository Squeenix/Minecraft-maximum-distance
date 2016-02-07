#include "stlastar.h"
#include <iostream>
#include <string>
#include <vector>
#include <stdio.h>
#include <stdlib.h>

#define DEBUG_LISTS 0
#define DEBUG_LIST_LENGTHS_ONLY 0

using namespace std;


//pretty much just represents a 3d vector with 1 auxillary variable at this point but xcode sucks at refactoring
struct edge {
    int x;
    int y;
    int z;
    int cost;
    edge(int x, int y, int z,int cost){
        this->x = x;
        this->y = y;
        this->z = z;
        this->cost = cost;
    }
};

const int desiredLayers = 30;

const int maxX = 10+desiredLayers*2;
const int maxY = desiredLayers+3;
const int maxZ = 10+desiredLayers*2;

const int aLength = maxX * maxY *maxZ;

const int centerX = maxX /2;
const int centerY = 0;
const int centerZ = maxZ /2;

const int startX = 5;
const int endX = maxX - 4;

const int startY = 0;
const int endY = maxY - 3;

const int startZ = 5;
const int endZ = maxZ - 4;

bool *indices = (bool*)malloc(sizeof(bool)*maxX*maxY*maxZ);
std::vector<edge> *edges = (std::vector<edge>*) malloc(sizeof(std::vector<edge>)*maxX*maxY*maxZ);

int flatten(int x, int y, int z){
    return x + y* maxX + z*maxX*maxY;
}

int ff = 0;

const int g1 = centerX;
const int g2 = centerY;
const int g3 = centerZ;

bool closer(int x1, int x2, int x3, int o1, int o2, int o3){

    int origDist = abs(x1-g1)+abs(x2-g2)+abs(x3-g3);
    int newDist = abs(x1+o1-g1)+abs(x2+o2-g2)+abs(x3+o3-g3);
    return origDist >= newDist;
}

class PathSearchNode
{
public:
    
    int x;
    int y;
    int z;
    
    PathSearchNode();
    PathSearchNode(int x, int y, int z);
    
    float GoalDistanceEstimate( PathSearchNode &nodeGoal );
    bool IsGoal( PathSearchNode &nodeGoal );
    bool GetSuccessors( AStarSearch<PathSearchNode> *astarsearch, PathSearchNode *parent_node );
    float GetCost( PathSearchNode &successor );
    bool IsSameState( PathSearchNode &rhs );
    
    void PrintNodeInfo();
};
PathSearchNode::PathSearchNode(){
}
PathSearchNode::PathSearchNode(int x, int y, int z){
    this->x = x;
    this->y = y;
    this->z = z;
}

bool PathSearchNode::IsSameState( PathSearchNode &rhs )
{
    if(x == rhs.x && y ==rhs.y && z == rhs.z){
        return true;
    }
    return(false);
}

float PathSearchNode::GoalDistanceEstimate( PathSearchNode &nodeGoal )
{
    return abs(nodeGoal.x - x) + abs(nodeGoal.z -z) + abs(nodeGoal.y -y);
}

// check if "this" node is the goal node
bool PathSearchNode::IsGoal( PathSearchNode &nodeGoal )
{
    if(x == centerX && y == centerY && z == centerZ){
        return(true);
    }
    return(false);
}

// generates the successor nodes of "this" node
bool PathSearchNode::GetSuccessors( AStarSearch<PathSearchNode> *astarsearch, PathSearchNode *parent_node )
{
    std::vector<edge> costs = edges[flatten(x, y, z)];
    for(int i=0; i<costs.size(); i++)
    {
        edge e = costs.at(i);
        PathSearchNode newNode = PathSearchNode(e.x, e.y, e.z);
        astarsearch->AddSuccessor(newNode);
    }
    return true;
}

// the cost of going from "this" node to the "successor" node
float PathSearchNode::GetCost( PathSearchNode &successor )
{
    int cost = 99999;
    vector<edge> costs = edges[flatten(x, y, z)];
    for (int i = 0; i < costs.size(); i++) {
        edge e = costs.at(i);
        if(e.x == successor.x && e.y ==successor.y && e.z == successor.z){
            if(e.cost < cost){
                cost = e.cost;
            }
        }
    }
    
    
    return cost;
    
}

// prints out information about the node
void PathSearchNode::PrintNodeInfo()
{
}

int main( int argc, const char* argv[] )
{
    
        for(int i = 0; i<  maxX * maxY * maxZ; i++){
            indices[i] = false;
            vector<edge> v;
            edges[i] = v;
        }
        
        indices[flatten(centerX, centerY, centerZ)] = true;
    
        while(true){
        //Start loop for placing blocks
        for(int i = 0; i < aLength; i++){
            edges[i].clear();
        }
        
        //too lazy to make a vector3
        vector<edge> adjacents;
        for(int x = startX; x < endX; x++){
            for(int y = startY; y < endY; y++){
                for(int z = startZ; z < endZ; z++){


                    int coutAdjacent = 0;
                    int cout = 0;
                
                    
                    int in;
                    int tv = flatten(x, y, z);
                    //The big question is why I didn't make method to do this.
                    
                    
                    
                    //DOWN
                    in = flatten(x, y-1, z);
                    if(y - 1 >= 0 && indices[in] && closer(x, y, z, 0, -1, 0)){
                        edge e(x, y-1, z, 1);
                        edges[tv].push_back(e);
                        coutAdjacent++;
                    }
                    
                    //DIAGONAL DOWN
                    in = flatten(x, y-1, z+1);
                    if(y - 1 >= 0 && indices[in] && closer(x, y, z, 0, -1, 1)){
                        edge e(x, y-1, z+1, 2);
                        edges[tv].push_back(e);
                        coutAdjacent++;
                    }
                    in = flatten(x, y-1, z-1);
                    if(y - 1 >= 0 && indices[in] && closer(x, y, z, 0, -1, -1)){
                        edge e(x, y-1, z-1, 2);
                        edges[tv].push_back(e);
                        coutAdjacent++;
                    }
                    in = flatten(x+1, y-1, z);
                    if(y - 1 >= 0 && indices[in] && closer(x, y, z, 1, -1, 0)){
                        edge e(x+1, y-1, z, 2);
                        edges[tv].push_back(e);
                        coutAdjacent++;
                    }
                    in = flatten(x-1, y-1, z);
                    if(y - 1 >= 0 && indices[in] && closer(x, y, z, -1, -1, 0)){
                        edge e(x-1, y-1, z, 2);
                        edges[tv].push_back(e);
                        coutAdjacent++;
                    }
                    
                    
                    //DIAGONAL CORNER DOWN
                    in = flatten(x+1, y-1, z+1);
                    if(y - 1 >= 0 && indices[in] && closer(x, y, z, 1, -1, 1)){
                        edge e(x+1, y-1, z+1, 3);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    in = flatten(x-1, y-1, z-1);
                    if(y - 1 >= 0 && indices[in] && closer(x, y, z, -1, -1, -1)){
                        edge e(x-1, y-1, z-1, 3);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    in = flatten(x+1, y-1, z-1);
                    if(y - 1 >= 0 && indices[in] && closer(x, y, z, 1, -1, -1)){
                        edge e(x+1, y-1, z-1, 3);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    in = flatten(x-1, y-1, z+1);
                    if(y - 1 >= 0 && indices[in] && closer(x, y, z, -1, -1, 1)){
                        edge e(x-1, y-1, z+1, 3);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    
                    //MINECART BULLSHIT
                    in = flatten(x+2, y, z);
                    if(indices[in] && closer(x, y, z, 2, 0, 0)){
                        edge e(x+2, y, z, 3);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    in = flatten(x-2, y, z);
                    if(indices[in] && closer(x, y, z, -2, 0, 0)){
                        edge e(x-2, y, z, 3);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    in = flatten(x, y, z+2);
                    if(indices[in] && closer(x, y, z, 0, 0, 2)){
                        edge e(x, y, z+2, 3);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    in = flatten(x, y, z-2);
                    if(indices[in] && closer(x, y, z, 0, 0, -2)){
                        edge e(x, y, z-2, 3);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    
                    
                    in = flatten(x+3, y, z);
                    if(indices[in] && closer(x, y, z, 3, 0, 0)){
                        edge e(x+3, y, z, 4);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    in = flatten(x-3, y, z);
                    if(indices[in] && closer(x, y, z, -3, 0, 0)){
                        edge e(x-3, y, z, 4);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    in = flatten(x, y, z+3);
                    if(indices[in] && closer(x, y, z, 0, 0, 3)){
                        edge e(x, y, z+3, 4);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    in = flatten(x, y, z-3);
                    if(indices[in] && closer(x, y, z, 0, 0, -3)){
                        edge e(x, y, z-3, 4);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    
                    
                    in = flatten(x+4, y, z);
                    if(indices[in] && closer(x, y, z, 4, 0, 0)){
                        edge e(x+4, y, z, 5);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    in = flatten(x-4, y, z);
                    if(indices[in] && closer(x, y, z, -4, 0, 0)){
                        edge e(x-4, y, z, 5);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    in = flatten(x, y, z+4);
                    if(indices[in] && closer(x, y, z, 0, 0, 4)){
                        edge e(x, y, z+4, 5);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    in = flatten(x, y, z-4);
                    if(indices[in] && closer(x, y, z, 0, 0, -4)){
                        edge e(x, y, z-4, 5);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    
                    in = flatten(x+5, y, z);
                    if(indices[in] && closer(x, y, z, 5, 0, 0)){
                        edge e(x+5, y, z, 6);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    in = flatten(x-5, y, z);
                    if(indices[in] && closer(x, y, z, -5, 0, 0)){
                        edge e(x-5, y, z, 6);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    in = flatten(x, y, z+5);
                    if(indices[in] && closer(x, y, z, 0, 0, 5)){
                        edge e(x, y, z+5, 6);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    in = flatten(x, y, z-5);
                    if(indices[in] && closer(x, y, z, 0, 0, -5)){
                        edge e(x, y, z-5, 6);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    
                    //DIAGONALS
                    in = flatten(x+1, y, z+1);
                    if(indices[in] && closer(x, y, z, 1, 0, 1)){
                        edge e(x+1, y, z+1, 3);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    in = flatten(x+1, y, z+2);
                    if(indices[in] && closer(x, y, z, 1, 0, 2)){
                        edge e(x+1, y, z+2, 4);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    in = flatten(x+1, y, z+3);
                    if(indices[in] && closer(x, y, z, 1, 0, 3)){
                        edge e(x+1, y, z+3, 5);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    in = flatten(x+1, y, z+4);
                    if(indices[in] && closer(x, y, z, 1, 0, 4)){
                        edge e(x+1, y, z+4, 6);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    in = flatten(x+2, y, z+1);
                    if(indices[in] && closer(x, y, z, 2, 0, 1)){
                        edge e(x+2, y, z+1, 4);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    in = flatten(x+3, y, z+1);
                    if(indices[in] && closer(x, y, z, 3, 0, 1)){
                        edge e(x+3, y, z+1, 5);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    in = flatten(x+4, y, z+1);
                    if(indices[in] && closer(x, y, z, 4, 0, 1)){
                        edge e(x+4, y, z+1, 6);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    
                    in = flatten(x+2, y, z+2);
                    if(indices[in] && closer(x, y, z, 2, 0, 2)){
                        edge e(x+2, y, z+2, 5);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    in = flatten(x+2, y, z+3);
                    if(indices[in] && closer(x, y, z, 2, 0, 3)){
                        edge e(x+2, y, z+3, 6);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    in = flatten(x+3, y, z+2);
                    if(indices[in] && closer(x, y, z, 3, 0, 2)){
                        edge e(x+3, y, z+2, 6);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    
                    //DIAGONALS
                    in = flatten(x-1, y, z+1);
                    if(indices[in] && closer(x, y, z, -1, 0, 1)){
                        edge e(x-1, y, z+1, 3);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    in = flatten(x-1, y, z+2);
                    if(indices[in] && closer(x, y, z, -1, 0, 2)){
                        edge e(x-1, y, z+2, 4);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    in = flatten(x-1, y, z+3);
                    if(indices[in] && closer(x, y, z, -1, 0, 3)){
                        edge e(x-1, y, z+3, 5);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    in = flatten(x-1, y, z+4);
                    if(indices[in] && closer(x, y, z, -1, 0, 4)){
                        edge e(x-1, y, z+4, 6);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    in = flatten(x-2, y, z+1);
                    if(indices[in] && closer(x, y, z, -2, 0, 1)){
                        edge e(x-2, y, z+1, 4);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    in = flatten(x-3, y, z+1);
                    if(indices[in] && closer(x, y, z, -3, 0, 1)){
                        edge e(x-3, y, z+1, 5);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    in = flatten(x-4, y, z+1);
                    if(indices[in] && closer(x, y, z, -4, 0, 1)){
                        edge e(x-4, y, z+1, 6);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    
                    in = flatten(x-2, y, z+2);
                    if(indices[in] && closer(x, y, z, -2, 0, 2)){
                        edge e(x-2, y, z+2, 5);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    in = flatten(x-2, y, z+3);
                    if(indices[in]&& closer(x, y, z, -2, 0, 3)){
                        edge e(x-2, y, z+3, 6);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    in = flatten(x-3, y, z+2);
                    if(indices[in]&& closer(x, y, z, -3, 0, 2)){
                        edge e(x-3, y, z+2, 6);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    
                    //DIAGONALS
                    in = flatten(x-1, y, z-1);
                    if(indices[in]&& closer(x, y, z, -1, 0, -1)){
                        edge e(x-1, y, z-1, 3);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    in = flatten(x-1, y, z-2);
                    if(indices[in]&& closer(x, y, z, -1, 0, -2)){
                        edge e(x-1, y, z-2, 4);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    in = flatten(x-1, y, z-3);
                    if(indices[in]&& closer(x, y, z, -1, 0, -3)){
                        edge e(x-1, y, z-3, 5);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    in = flatten(x-1, y, z-4);
                    if(indices[in]&& closer(x, y, z, -1, 0, -4)){
                        edge e(x-1, y, z-4, 6);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    in = flatten(x-2, y, z-1);
                    if(indices[in]&& closer(x, y, z, -2, 0, -1)){
                        edge e(x-2, y, z-1, 4);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    in = flatten(x-3, y, z-1);
                    if(indices[in]&& closer(x, y, z, -3, 0, -1)){
                        edge e(x-3, y, z-1, 5);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    in = flatten(x-4, y, z-1);
                    if(indices[in]&& closer(x, y, z, -4, 0, -1)){
                        edge e(x-4, y, z-1, 6);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    
                    in = flatten(x-2, y, z-1);
                    if(indices[in]&& closer(x, y, z, -2, 0, -1)){
                        edge e(x-2, y, z-2, 5);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    in = flatten(x-2, y, z-3);
                    if(indices[in]&& closer(x, y, z, -2, 0, -3)){
                        edge e(x-2, y, z-3, 6);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    in = flatten(x-3, y, z-2);
                    if(indices[in]&& closer(x, y, z, -3, 0, -2)){
                        edge e(x-3, y, z-2, 6);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    
                    //DIAGONALS
                    in = flatten(x+1, y, z-1);
                    if(indices[in]&& closer(x, y, z, 1, 0, -1)){
                        edge e(x+1, y, z-1, 3);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    in = flatten(x+1, y, z-2);
                    if(indices[in]&& closer(x, y, z, 1, 0, -2)){
                        edge e(x+1, y, z-2, 4);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    in = flatten(x+1, y, z-3);
                    if(indices[in]&& closer(x, y, z, 1, 0, -3)){
                        edge e(x+1, y, z-3, 5);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    in = flatten(x+1, y, z-4);
                    if(indices[in]&& closer(x, y, z, 1, 0, -4)){
                        edge e(x+1, y, z-4, 6);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    in = flatten(x+2, y, z-1);
                    if(indices[in]&& closer(x, y, z, 2, 0, -1)){
                        edge e(x+2, y, z-1, 4);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    in = flatten(x+3, y, z-1);
                    if(indices[in]&& closer(x, y, z, 3, 0, -1)){
                        edge e(x+3, y, z-1, 5);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    in = flatten(x+4, y, z-1);
                    if(indices[in]&& closer(x, y, z, 4, 0, -1)){
                        edge e(x+4, y, z-1, 6);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    
                    in = flatten(x+2, y, z-2);
                    if(indices[in]&& closer(x, y, z, 2, 0, -2)){
                        edge e(x+2, y, z-2, 5);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    in = flatten(x+2, y, z-3);
                    if(indices[in]&& closer(x, y, z, 2, 0, -3)){
                        edge e(x+2, y, z-3, 6);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    in = flatten(x+3, y, z-2);
                    if(indices[in]&& closer(x, y, z, 3, 0, -2)){
                        edge e(x+3, y, z-2, 6);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    
                    
                    //TO THE SIDE
                    in = flatten(x+1, y, z);
                    if(indices[in]){
                        //Can pop chest without getting the block above it
                        if(x+1 == centerX && y == centerY && z ==centerZ){
                            edge e(x+1, y, z, 1);
                            edges[tv].push_back(e);
                        }
                        else{
                            edge e(x+1, y, z, 2);
                            edges[tv].push_back(e);
                        }
                        coutAdjacent++;
                    }
                    in = flatten(x-1, y, z);
                    if(indices[in]){
                        if(x-1 == centerX && y == centerY && z ==centerZ){
                            edge e(x-1, y, z, 1);
                            edges[tv].push_back(e);
                        }
                        else{
                            edge e(x-1, y, z, 2);
                            edges[tv].push_back(e);
                        }
                        coutAdjacent++;
                    }
                    in = flatten(x, y, z+1);
                    if(indices[in]){
                        if(x == centerX && y == centerY && z+1 ==centerZ){
                            edge e(x, y, z+1, 1);
                            edges[tv].push_back(e);
                        }
                        else{
                            edge e(x, y, z+1, 2);
                            edges[tv].push_back(e);
                        }
                        coutAdjacent++;
                    }
                    in = flatten(x, y, z-1);
                    if(indices[in]){
                        if(x == centerX && y == centerY && z-1 ==centerZ){
                            edge e(x, y, z-1, 1);
                            edges[tv].push_back(e);
                        }
                        else{
                            edge e(x, y, z-1, 2);
                            edges[tv].push_back(e);
                        }
                        coutAdjacent++;
                    }
                    
                    //Diagonal poppin' chests
                    in = flatten(centerX, centerY, centerZ);
                    if(x-1 == centerX && y == centerY && z-1 ==centerZ){
                        edge e(x-1, y, z-1, 2);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    if(x+1 == centerX && y == centerY && z+1 ==centerZ){
                        edge e(x+1, y, z+1, 2);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    if(x+1 == centerX && y == centerY && z-1 ==centerZ){
                        edge e(x+1, y, z-1, 2);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    if(x-1 == centerX && y == centerY && z+1 ==centerZ){
                        edge e(x-1, y, z+1, 2);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    
                    //Bendin' down n' poppin chests
                    if(x+2 == centerX && y == centerY && z+1 ==centerZ){
                        edge e(x+2, y, z+1, 3);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    if(x+2 == centerX && y == centerY && z-1 ==centerZ){
                        edge e(x+2, y, z-1, 3);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    if(x+2 == centerX && y == centerY && z ==centerZ){
                        edge e(x+2, y, z, 2);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    
                    if(x-2 == centerX && y == centerY && z+1 ==centerZ){
                        edge e(x-2, y, z+1, 3);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    if(x-2 == centerX && y == centerY && z-1 ==centerZ){
                        edge e(x-2, y, z-1, 3);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    if(x-2 == centerX && y == centerY && z ==centerZ){
                        edge e(x-2, y, z, 2);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    
                    
                    if(x+1 == centerX && y == centerY && z+2 ==centerZ){
                        edge e(x+1, y, z+2, 3);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    if(x-1 == centerX && y == centerY && z+2 ==centerZ){
                        edge e(x-1, y, z+2, 3);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    if(x == centerX && y == centerY && z+2 ==centerZ){
                        edge e(x, y, z+2, 2);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    
                    if(x+1 == centerX && y == centerY && z-2 ==centerZ){
                        edge e(x+1, y, z-2, 3);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    if(x-1 == centerX && y == centerY && z-2 ==centerZ){
                        edge e(x-1, y, z-2, 3);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    if(x == centerX && y == centerY && z-2 ==centerZ){
                        edge e(x, y, z-2, 2);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    
                    
                    //Digging a tunnel and peeking under
                    if(x+4 == centerX && y == centerY && z ==centerZ){
                        edge e(x+4, y, z, 5);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    if(x-4 == centerX && y == centerY && z ==centerZ){
                        edge e(x-4, y, z, 5);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    if(x == centerX && y == centerY && z+4 ==centerZ){
                        edge e(x, y, z+4, 5);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    if(x== centerX && y == centerY && z-4 ==centerZ){
                        edge e(x, y, z-4, 5);
                        edges[tv].push_back(e);
                        cout++;
                    }
                    
                    
                    
                    
                    
                    if(coutAdjacent + cout >= 1){
                        if(coutAdjacent >= 1 && !indices[flatten(x, y, z)]){
                            edge e(x, y, z, 0);
                            adjacents.push_back(e);
                        }
                    }
                }
            }
        }
        
        vector<edge> results;
        AStarSearch<PathSearchNode> astarsearch;
        
        int minCost = 9999999;
            
        
        for(int i = 0; i <adjacents.size(); i++){
            edge e = adjacents.at(i);
            PathSearchNode nodeStart(e.x,e.y,e.z);
            
            PathSearchNode nodeEnd(centerX, centerY, centerZ);
            
            astarsearch.SetStartAndGoalStates( nodeStart, nodeEnd );
            
            unsigned int SearchState;
            unsigned int SearchSteps = 0;
            
            do{
                SearchState = astarsearch.SearchStep();
                SearchSteps++;
            } while( SearchState == AStarSearch<PathSearchNode>::SEARCH_STATE_SEARCHING );
            if(SearchState == AStarSearch<PathSearchNode>::SEARCH_STATE_SUCCEEDED ){
                int cost = (int)astarsearch.GetSolutionCost();
                if(cost < minCost){
                    minCost = cost;
                    PathSearchNode *start = astarsearch.GetSolutionStart();
                    while(start != NULL){
                        //printf("(%i,%i,%i)\n\n", start->x,start->y,start->z);
                        start = astarsearch.GetSolutionNext();
                    }
                    printf("\n\n");
                }
                edge s(e.x,e.y,e.z,cost);
                results.push_back(s);
                astarsearch.FreeSolutionNodes();
            }
            astarsearch.EnsureMemoryFreed();
        }
            ff = 0;
            for(int i = 0; i < results.size(); i++){
                if(results.at(i).cost == minCost){
                    ff++;
                }
            }
            int money = 1;
            for(int y = startY; y < endY; y++){
                int numLayer = 0;
                printf("\n");
                for (int z = startZ; z < endZ; z++) {
                    for (int x = startX; x < endX; x++) {
                        if(indices[flatten(x,y,z)] == true){
                            if(x == centerX && y == centerY && z == centerZ){
                                printf("O");
                            }
                            else{
                                money++;
                                printf("X");
                            }
                            numLayer++;
                        }
                        else{
                            printf("-");
                        }
                    }
                    printf("%i\n", z- startZ);
                }
                printf("\n");
            }
            printf("%d layers \n", minCost);
            printf("Costs: %d blocks", money);
        //printf("Left of cost %d: %lu\n", minCost, ff++);
        if(minCost == desiredLayers){
            
            break;
        }
        for(int i = 0; i < results.size(); i++){
            edge r = results[i];
            if(r.cost == minCost){
                indices[flatten(r.x, r.y, r.z)] = true;
                //if(minCost ==desiredLayers-1) break;
            }
        }
    }
    
    return 0;
    
}
