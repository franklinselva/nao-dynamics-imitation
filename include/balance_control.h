#ifndef BALANCE_CONTROL_H
#define BALANCE_CONTROL_H

#include <robot.h>
#include <Jacobian.h>

/**
 * @brief Holds the implementation of balance control and imitation of the nao robot using CoM based static balance
 * 
 */

class balanceControl : public robot
{
public:
    balanceControl(std::string robot_IP, int robot_port, int mode);
    balanceControl();

    void balance();
    void not_balanced();
    void DS();

    void check_balance_and_move();
    void begin_imitation(float feetdistance, float distancepiedR, float distancepiedL, float rotation_tete);
};

#endif // BALANCE_CONTROL_H
