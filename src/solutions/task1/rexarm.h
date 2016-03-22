#include <vector>
#include <lcmtypes/dynamixel_command_list_t.hpp>
#include <lcmtypes/dynamixel_command_t.hpp>


#define D1 .118
#define D2 .984
#define D3 .989
#define D4 .981
#define H .821
#define hz 15
#define NUM_SERVOS 6

//using namespace eecs467;
class Rexarm
{
//private:
    

public:
   	std::vector<double> thetas;
  
    //Default constructor 
    Rexarm();
    
    void setThetas(double theta, double radius);
    void run();
};


