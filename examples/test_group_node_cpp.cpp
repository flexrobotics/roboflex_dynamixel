#include <iostream>
#include <cmath>
#include "roboflex_dynamixel/dynamixel.h"
#include "roboflex_dynamixel/dynamixel_controller.h"

using namespace roboflex;
using namespace roboflex::dynamixelgroup;
using namespace roboflex::dynamixelnodes;

const int DynamixelID = 10;

class MyDynamixelRemoteController: public DynamixelRemoteController {
public:
    MyDynamixelRemoteController(): DynamixelRemoteController(), t0(0) {}

    DXLIdsToValues readwrite_loop_function(const DynamixelGroupState& state) {
        auto values = state.values;
        auto mymotorvalues = values[DynamixelID];

        if (t0 == 0) {
            t0 = state.timestamp.t0;

            p0 = mymotorvalues[DXLControlTable::PresentPosition];

            // won't compile - something about the const
            //p0 = state.values[DynamixelID][DXLControlTable::PresentPosition];
        }

        double dt = state.timestamp.t0 - t0;
        int new_pos = int(p0 + 500 * sin(dt));
        std::cout << "NEW_POS: " << new_pos << std::endl;
        std::cout << "CUR_POS: " <<  mymotorvalues[DXLControlTable::PresentPosition] << std::endl;
        return {{DynamixelID, {{DXLControlTable::GoalPosition, new_pos}}}};
    }

protected:
    double t0;
    int p0;
};

int main() {
    
    auto controller = MyDynamixelRemoteController();

    auto dynamixel_node = DynamixelGroupNode(
        DynamixelGroupController::PositionController(
            "/dev/ttyUSB0",
            4000000,
            {DynamixelID}
        )
    );

    dynamixel_node > controller;
    controller > dynamixel_node;

    dynamixel_node.start();
    //controller.start();

    sleep(6.35);

    dynamixel_node.stop();

    std::cout << "DONE" << std::endl;
    return 0;
}