#include "../include/clipper/clipper.hpp"
#include <iostream>
#include <vector>

#define MIN(x,y) (x < y ? x : y)
#define MAX(x,y) (x > y ? x : y)

using namespace std;

using namespace ClipperLib;

struct Node
{
    float x;
    float y;
};

vector<Node> checkobs(vector<std::vector<Node>> polygon_org, vector<Node> vecNodes, int n_p)
{
    int flag=0;
    vector<Node> vecNodesEx;
    for (auto &ii : vecNodes)
    {
        flag=0;
        
        for (int j=0; j < n_p && flag!=1; j++)
        {
            vector<Node> polygon =  polygon_org[j];
            int N =polygon.size();
        
            int counter = 0;
            int i;
            double xinters;
            Node p1,p2;

            p1 = polygon[0];
            for (i=1;i<=N;i++) {
                p2 = polygon[i % N];
                if (ii.y > MIN(p1.y,p2.y)) {
                if (ii.y <= MAX(p1.y,p2.y)) {
                    if (ii.x <= MAX(p1.x,p2.x)) {
                    if (p1.y != p2.y) {
                        xinters = (ii.y-p1.y)*(p2.x-p1.x)/(p2.y-p1.y)+p1.x;
                        if (p1.x == p2.x || ii.x <= xinters)
                        counter++;
                    }
                    }
                }
                }
                p1 = p2;
            }
            if (counter % 2 != 0)//inside
            {
                flag=1;
                cout<<"In";
                //std::cout<<ii.x;
            }
        }
        if (flag==0)
            vecNodesEx.push_back(ii);
    }
    return vecNodesEx;
}

int main()
{
    cInt ab;
    Path subj;
    Path a;
    Paths solution;
    subj << 
    IntPoint(3,0) << IntPoint(5,3) << IntPoint(7,0);
    ClipperOffset co;
    co.AddPath(subj, jtMiter, etClosedPolygon);
    co.Execute(solution, 1.0);

    unsigned long long cc = 12;
    float c = static_cast<float>(cc);

    a = solution[0];

    for (size_t i = 0; i < a.size(); i++)
    {
        cout << a[i] << endl;
    }
    
    cout << c << endl;

    return 0;
}