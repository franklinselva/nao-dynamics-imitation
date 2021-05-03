#ifndef DYNAMICS_PARAM_H
#define DYNAMICS_PARAM_H

/// Include robot softbank
#include <qi/os.hpp>
#include <alerror/alerror.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/almemoryproxy.h>
#include <alproxies/alrobotpostureproxy.h>
#include <alproxies/altexttospeechproxy.h>
#include <alcommon/albroker.h>
#include <boost/shared_ptr.hpp>

#include <Jacobian.h>
#include <vector>

class dPARAM
{
    /// This file concentrates on the dynamic parameters of NAO robot v5 H25
    public:
        dPARAM();
        ~dPARAM();

        std::vector<float> getMass();
        float getTotalMass();
        std::vector<Eigen::Vector3d> getCOM();
        std::vector<Eigen::MatrixXd> getInertia();

        float getMass(AL::ALValue segment);
        Eigen::Vector3d getCOM(AL::ALValue segment);
        Eigen::Matrix3f getInertia(AL::ALValue segment);

        void defMass();
        void defCoM();
        void defInertiaMatrix();

    private:
        std::vector<float> Mass;
        float TotalMass;
        std::vector<Eigen::Vector3d> CentreOfMass;
        std::vector<Eigen::Matrix3f> InertiaMatrix;
};

#endif
