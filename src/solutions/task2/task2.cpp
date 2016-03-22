#include "Gui.h"

using namespace std;

int main(int argc, char** argv) {
	//Retrieve input argument as an int
    // if (argc==1) {
    // 	return 1;
    // }

    VxWindow window(argc, argv);
    eecs467_init (argc, argv);
    int myID = 2; //atoi(argv[1]);
    int setup_result = window.setup(myID);
    if (setup_result!=0) {
        cout << "Error code: " << setup_result << endl;
        return 1;
    }	
    
    window.run();

    return 0;
}
