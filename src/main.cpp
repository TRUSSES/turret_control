#include "TurretController.h"
#include <unistd.h>
#include <iostream>

int main(int argc, char ** argv){
    std::cout << "Turret Initialization\n";
    TurretController turret;

    // Initialize the turret controller (sets up GPIO, CAN socket, etc.)
    turret.initTurret();

    // Zero the turret system (both spiral zipper and pitch)
    turret.zeroTurret();

    // Set goal parameters:
    // goal_dist: desired spiral zipper extension in meters,
    // pitch: desired turret pitch in degrees,
    // yaw: desired turret yaw in degrees (if applicable).
    // float goal_dist = 0.1;  // For example, extend to 0.5 meters.
    // float desired_pitch = -50.0;  // For example, set turret pitch to -30 degrees.
    // float desired_yaw = 0.0;  // Yaw remains unchanged here.

    // // Loop until the system has reached the desired state.
    // bool reached = false;
    // while (!reached) {
    //     reached = turret.actuateTurretCable(goal_dist, desired_pitch, desired_yaw);
    //     usleep(10000);  // Delay for 10ms to allow for sensor updates.
    // }
    // std::cout << "Goal position reached using actuateTurretCable.\n";

    // Close the CAN socket before exiting.
    turret.closeSocket();
    return 0;
}


// #include "TurretController.h"
// #include <iostream>
// #include <cmath>
// #include <unistd.h> // for sleep()

// using namespace std;

// int main(int argc, char ** argv){
//     cout << "Turret Init" << endl;

//     TurretController turret;

//     // Initialize turret controller
//     turret.initTurret();

//     sleep(2);
//     turret.zeroZipper();

//     sleep(2);

//     const float tolerance = 0.01f; // Adjust based on your precision requirements

//     while (true) {
//         float zipper_length;
//         cout << "Enter desired zipper length (enter a negative value to exit): ";
//         cin >> zipper_length;

//         // Check if the input is valid
//         if (!cin) {
//             cout << "Invalid input. Exiting program." << endl;
//             break;
//         }
        
//         // Exit the loop if a negative value is entered
//         if (zipper_length < 0) {
//             cout << "Exiting program." << endl;
//             break;
//         }

//         // Actuate until the desired zipper length is reached within tolerance
//         while (fabs(turret.getActuatorLength() - zipper_length) > tolerance) {
//             turret.actuateZipperLength(zipper_length);
//             //cout << "Current length of zipper = " << turret._sz_encoder_count << endl;
//             sleep(0.02); // small delay to allow encoder update and reduce CPU load
//         }

//         turret.setMotorOutput(1500); // explicitly stop the motor after reaching position
//         cout << "Target zipper length reached: " << turret.getActuatorLength() << endl;
//     }

//     turret.closeSocket();
//     return 0;
// }
