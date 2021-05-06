#ifndef BALANCE_CONTROL_H
#define BALANCE_CONTROL_H

#include <robot.h>

class balanceControl : public robot
{
    public:
        balanceControl();
        ~balanceControl();

        void balance();
        void not_balanced();
        void DS();

        void check_balance_and_move();
        void imitation_bis(float feetdistance, float distancepiedR, float distancepiedL, float rotation_tete);

};

#endif // BALANCE_CONTROL_H
