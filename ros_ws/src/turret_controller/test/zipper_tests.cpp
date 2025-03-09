/**
 * @file main.cpp
 * @brief Tester code for specific features
 *        of ZipperActuator.cpp implementation.
 *
 * @author Akshay Ram <paks@seas.upenn.edu>
 * @author Maggie Du <maggiedu@seas.upenn.edu>
 * @date 07/09/2024
 */

#include "FullTurretControl.h"

int main(){
    FullTurretControl turret = FullTurretControl();
    turret.setupCAN();
    turret.setupSPI();
    turret.setupLoadCells();
    turret.initZipper();
    turret.zeroZipper();

    turret.actuateZipperLength(0.1);
    sleep(2);

    initscr();
    timeout(100);
    keypad(stdscr, TRUE);
    noecho();

    int next_key = -1;

    while (true) {
        turret.getLoadCellReadings();
        
        if (next_key == -1) {
            next_key = getch();
        }
        if (next_key != -1) {
            switch (next_key) {
                case 'z':
                    turret.zeroZipper();
                    break;
                case 'r':
                    turret.setMotorOutput(1200);
                    break;
                case 'e':
                    turret.setMotorOutput(1800);
                    break;
                case 's':
                    turret.stopZipper();
                    break;
                case 'l':
                    turret.sendToMotor(2, 0, 2, 1, 2, 0);
                    break;
                case 'k':
                    turret.sendToMotor(2, 0, -2, 1, 2, 0);
                    break;
                case 'o':
                    turret.sendToMotor(41, 0, -2, 1, 2, 0);
                    break;
                case 'p':
                    turret.sendToMotor(41, 0, 2, 1, 2, 0);
                    break;
                default:
                    break;
            }
            next_key = getch();
        }
    }

    turret.exitMotorMode(2);
    turret.exitMotorMode(41);

    turret.closeSocket();
    gpioTerminate();

    return 0;
}