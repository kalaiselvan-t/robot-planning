#include "../include/roadmap/r_map.h"

// Data Struct Initialisations
int no_samples = 100;
int max_border_x = 10;
int max_border_y = 10;

random_device rd;
mt19937 gen(rd());
uniform_real_distribution<float> dis_x(0,max_border_x);
uniform_real_distribution<float> dis_y(0, max_border_y);

Nodes roadmap_nodes;

// Function Declarations

float round_up
    (float inp, int places)
{
    int no = pow(10,places);

    return floorf(inp * no) / no;
}

Node create_random_node
    ()
{
    Node node;
    node.x = round_up(dis_x(gen),2);
    node.y = round_up(dis_y(gen),2);

    return node;
}

void test_link
    ()
{
    cout << "Connected\n";
}