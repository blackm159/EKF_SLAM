#include "turtlelib/svg.hpp"
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include <iosfwd> // contains forward definitions for iostream objects
#include <cmath> // contains math functions
#include <iostream> // contains iostream objects
#include <fstream>
#include <cstdlib> 
#include <vector>

using namespace std;

int main() {

    turtlelib::SVG svg;

    turtlelib::Transform2D T_ab;
    turtlelib::Transform2D T_ba;
    turtlelib::Transform2D T_bc;
    turtlelib::Transform2D T_cb;
    turtlelib::Transform2D T_ac;
    turtlelib::Transform2D T_ca;

    cout<<"Welcome to frame_main!\n";
    cout<<"Enter transform T_{a,b}:\n";
    cin>>T_ab;
    // cout<<T_ab<<"\n";

    cout<<"Enter transform T_{b,c}:\n";
    cin>>T_bc;
    // cout<<T_bc<<"\n";

    cout<<"T_{a,b}: "<<T_ab<<"\n";
    svg.Draw(T_ab, "T_{a,b}");

    T_ba = T_ab.inv();
    cout<<"T_{b,a}: "<<T_ba<<"\n";
    svg.Draw(T_ba, "T_{b,a}");

    cout<<"T_{b,c}: "<<T_bc<<"\n";
    svg.Draw(T_bc, "T_{b,c}");

    T_cb = T_bc.inv();
    cout<<"T_{c,b}: "<<T_cb<<"\n";
    svg.Draw(T_cb, "T_{c,b}");

    T_ac = T_ab*T_bc;
    cout<<"T_{a,c}: "<<T_ac<<"\n";
    svg.Draw(T_ac, "T_{a,c}");

    T_ca = T_ac.inv();
    cout<<"T_{c,a}: "<<T_ca<<"\n";
    svg.Draw(T_ca, "T_{c,a}");


    turtlelib::Point2D p_a;
    turtlelib::Point2D p_b;
    turtlelib::Point2D p_c;

    cout<<"Enter point p_a:\n";
    cin>>p_a;
    cout<<"p_a: "<<p_a<<"\n";
    svg.Draw(p_a, "purple");

    p_b = T_ba(p_a);
    cout<<"p_b: "<<p_b<<"\n";
    svg.Draw(p_b, "brown");

    p_c = T_ca(p_a);
    cout<<"p_c: "<<p_c<<"\n";
    svg.Draw(p_c, "orange");


    turtlelib::Vector2D v_a;
    turtlelib::Vector2D v_b;
    turtlelib::Vector2D v_c;
    turtlelib::Vector2D v_bhat;

    turtlelib::Point2D head;
    turtlelib::Point2D tail;
    tail.x = 0.0;
    tail.y = 0.0;

    cout<<"Enter vector v_a:\n";
    cin>>v_b;

    v_bhat.x = v_b.x / sqrt(pow(v_b.x, 2.0) + pow(v_b.y, 2.0));
    v_bhat.y = v_b.y / sqrt(pow(v_b.x, 2.0) + pow(v_b.y, 2.0));
    cout<<"v_bhat: "<<v_bhat<<"\n";
    head.x = v_bhat.x;
    head.y = v_bhat.y;
    svg.Draw(head, tail, "brown");

    v_a = T_ab(v_b);
    cout<<"v_a: "<<v_b<<"\n";
    head.x = v_a.x;
    head.y = v_a.y;
    svg.Draw(head, tail, "purple");

    cout<<"v_b: "<<v_b<<"\n";

    v_c = T_cb(v_b);
    cout<<"v_c: "<<v_c<<"\n";
    head.x = v_c.x;
    head.y = v_c.y;
    svg.Draw(head, tail, "orange");


    turtlelib::Twist2D V_a;
    turtlelib::Twist2D V_b;
    turtlelib::Twist2D V_c;

    cout<<"Enter twist V_b:\n";
    cin>>V_b;

    V_a = T_ab(V_b);
    cout<<"V_a: "<<V_a<<"\n";

    cout<<"V_b: "<<V_b<<"\n";

    V_c = T_cb(V_b);
    cout<<"V_c: "<<V_c<<"\n";


    svg.Export("/tmp/frames.svg");


    return 0;
}