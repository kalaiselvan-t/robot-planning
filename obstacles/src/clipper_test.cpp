/*
Clipper is not working as intended
Having problems with InflatePaths function
*/





#include <clipper2/clipper.h>
#include <vector>

using namespace std;
using namespace Clipper2Lib;

int main()
{
    Paths64 out_p;
    Point64 p1(0,0);
    Point64 p2(100,0);
    Point64 p3(100,100);
    Point64 p4(100,0);
    Path64 p = {p1,p2,p3,p4};
    Paths64 inp_p = {p};
    // vector<Point64> p_pts = {p1,p2,p3,p4};

    // out_p = InflatePaths(inp_p,10.0,JoinType::Round,EndType::Polygon);



}