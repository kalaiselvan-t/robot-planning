#include "../include/roadmap/r_map.h"

int main()
{
    while 
        (static_cast<int>(roadmap_nodes.list.size()) < no_samples)
    {
        roadmap_nodes.list.push_back(create_random_node());
    }

    roadmap_nodes.print();
    
    return 0;
}