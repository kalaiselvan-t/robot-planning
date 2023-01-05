#ifndef R_MAP_H
#define R_MAP_H

#include <iostream>
#include <vector>
#include <random>
#include <algorithm>

using namespace std;

// Data Structs

struct Node
{
    float x;
    float y;
};

struct Nodes
{
    vector<Node> list;

    void print()
    {
        for 
            (size_t i = 0; i < list.size(); i++)
        {
            cout << "x: " << list[i].x << " y: " << list[i].y << endl;
        }
    }
};


// Initialisations
extern int no_samples;
extern int max_border_x;
extern int max_border_y;

extern random_device rd;

extern Nodes roadmap_nodes;

// Function Declarations

float round_up(float inp, int places);

Node create_random_node();

void test_link();

#endif
