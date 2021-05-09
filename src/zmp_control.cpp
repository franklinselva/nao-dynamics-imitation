/**
 * @file zmp_control.cpp
 * @author Selvakumar H S (franklinselva10@gmail.com)
 * @brief Implementation of ZMP Control based on the references from "Kajita et al. 2003; 
 * Biped walking pattern generation by using preview control of zero-moment point"
 * @version 0.1
 * @date 2021-05-07
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <zmp_control.h>

#define INF 10000

/**
 * @brief get ZMP position based on the current CoM
 * The model is assumed to be 3D-LIPM as referenced in Kajita. et. al. 2003 
 * 
 * @return Eigen::Vector3f the position of ZMP
 */
Eigen::Vector3f ZMPControl::getZMP()
{
    //Get current CoM from the robot state
    ZMPControl::pCoM = Nao->GetXYZ_CoM(robot::m_q_current);

    ZMPControl::aCoM = g * (pCoM - pZMP) / pCoM[2];

    ZMPControl::pZMP[0] = pCoM[0] - pCoM[2] * aCoM[0] / ZMPControl::g;
    ZMPControl::pZMP[1] = pCoM[1] - pCoM[2] * aCoM[1] / ZMPControl::g;
    ZMPControl::pZMP[2] = 0;

    return ZMPControl::pZMP;
}

/**
 * @brief Checks for the ZMP within the support polygon. Note that this function is approximately
 * the same as the check_balance_and_save() function in balanceControl class. 
 * Reference taken for point within polygon from https://www.eecs.umich.edu/courses/eecs380/HANDOUTS/PROJ2/InsidePoly.html
 * 
 * @return true if the ZMP is within the support polygon
 * @return false if the ZMP goes outside the support polygon
 */
bool ZMPControl::withinSP()
{
    robot::m_withinSP = false;
    std::vector<float> q = robot::m_robot_joint_values;
    std::vector<Eigen::Vector2d> footVertex, lFootVertex, rFootVertex; //Vector of foot vertices
    const uint nVertices = 8;

    pLFoot = Nao->GetP_LFoot(q);
    pLFoot = Nao->GetP_LFoot(q);

    lFootVertex = ZMPControl::calculateFootCoordinates(pLFoot, 0);
    rFootVertex = ZMPControl::calculateFootCoordinates(pRFoot, 0);
    Eigen::Vector2d pExtreme{INF, pZMP[1]}; // The point acts as an ray with point to ZMP

    int count = 0, i = 0;

    do
    {
        int next = (i + 1) & nVertices;

        if (doIntersect(footVertex[i], footVertex[next], pZMP, pExtreme))
        {
            if (orientation(footVertex[i], pZMP, footVertex[next]) == 0)
            {
                robot::m_withinSP = onSegment(footVertex[i], pZMP, footVertex[next]);
                return robot::m_withinSP;
            }
            count++;
        }
        i = next;
    } while (i != 0);

    robot::m_withinSP = count & 1; //Odd number bool return
    return robot::m_withinSP;
}

/**
 * @brief Checks for left, right or double support and stores the value in: m_isDS, m_isLS, m_isRS
 * 
 */
void ZMPControl::checkSupportPhase()
{
    ///NAO only imitate angle if he is not moving
    if (m_mode == 2)
    {
        /// Check if the CoM is in the balance polygon and imitate.
        robot::m_isDS = false;
        robot::m_isRS = false;
        robot::m_isLS = false;

        if (m_FeetHeight(0) < 10 && m_FeetHeight(1) < 10)
        {
            robot::m_isDS = true;
        }
        else if (m_FeetHeight(0) > 10)
        {
            robot::m_isLS = true;
        }
        else if (m_FeetHeight(1) > 10)
        {
            robot::m_isRS = true;
        }
    }
}

void ZMPControl::balance()
{
}

/**
 * @brief Check for dynamic robot balance based on mode(1/2);
 * Focuses on single and double support for mode 2;
 * Starts the imitation if mode 1 is selected
 * 
 */
void ZMPControl::check_balance_and_move()
{
    ZMPControl::checkSupportPhase();
    if (robot::m_mode == 2 && (robot::m_isDS || robot::m_isLS || robot::m_isRS))
    {
        ZMPControl::balance();
    }
    else if (robot::m_mode == 1)
    {
        // Start Body Imitation
        m_motion->setAngles(m_robot_joint_names, m_robot_joint_values, m_motors_speed);
        m_q_current = m_robot_joint_values;
    }
}

/**
 * @brief Starts the imitation of the human actor based on the mode selected (1/2); 
 * Note that dynamci balance control is implemented with this function.
 * 
 * @param feetdistance The euclidean distance betweem the two feet
 * @param distancepiedR The euclidean distance from right foot to torso projection
 * @param distancepiedL The euclidean distance from left foot to torso projection
 * @param dRotation Rotation difference along z axis from Xsens Analyze
 */
void ZMPControl::begin_imitation(float feetdistance, float distancepiedR, float distancepiedL, float dRotation)
{
    ///update m_velocity if feet are far or head is turned
    robot::update_velocity(feetdistance, distancepiedR, distancepiedL, dRotation);
    robot::m_imitate = true;

    ///Check if the robot is balanced then proceed to imitate without falling
    ZMPControl::check_balance_and_move();
}

/**
 * @brief Calculates the foot vertex coordinates in 2D projection
 * 
 * @param footCenter Eigen::Vector3d - Center of the position of left foot or right foot 
 * @param footAngle float(rad) - FootAngle of the left or right foot
 * @return std::vector<Eigen::Vector2d> - Returns the vertex coordinates in order "Top Right, Top Left, Bottom Right, Bottom Left"
 */
std::vector<Eigen::Vector2d> ZMPControl::calculateFootCoordinates(Eigen::Vector3d footCenter, float footAngle)
{
    Eigen::Vector2d TR_vertex, TL_vertex, BL_vertex, BR_vertex;
    Eigen::Vector2d footCenter2D{footCenter[0], footCenter[1]};

    TR_vertex[0] = footCenter2D[0] + ((footWidth / 2) * cos(footAngle) - (footLength / 2) * sin(footAngle));
    TR_vertex[1] = footCenter2D[1] + ((footWidth / 2) * sin(footAngle) + (footLength / 2) * cos(footAngle));

    TL_vertex[0] = footCenter2D[0] - ((footWidth / 2) * cos(footAngle) - (footLength / 2) * sin(footAngle));
    TL_vertex[1] = footCenter2D[1] - ((footWidth / 2) * sin(footAngle) + (footLength / 2) * cos(footAngle));

    BL_vertex[0] = footCenter2D[0] - ((footWidth / 2) * cos(footAngle) + (footLength / 2) * sin(footAngle));
    BL_vertex[1] = footCenter2D[1] - ((footWidth / 2) * sin(footAngle) - (footLength / 2) * cos(footAngle));

    BR_vertex[0] = footCenter2D[0] + ((footWidth / 2) * cos(footAngle) + (footLength / 2) * sin(footAngle));
    BR_vertex[1] = footCenter2D[1] + ((footWidth / 2) * sin(footAngle) - (footLength / 2) * cos(footAngle));

    return std::vector<Eigen::Vector2d>{TR_vertex, TL_vertex, BR_vertex, BL_vertex};
}

/**
 * @brief Given the collinear points, the function checks for q in line pr
 * 
 * @param p Eigen::Vector2d - Coordinates of the line 
 * @param q Eigen::Vector2d - Point to be tested
 * @param r Eigen::Vector2d - Coordinates of the line 
 * @return true The point q is on the line pr
 * @return false The point q is not on the line pr
 */
bool ZMPControl::onSegment(Eigen::Vector2d p, Eigen::Vector2d q, Eigen::Vector2d r)
{
    if (q[0] <= std::max(p[0], r[0]) && q[0] >= std::min(p[0], r[0]) &&
        q[1] <= std::max(p[1], r[1]) && q[1] >= std::min(p[1], r[1]))
        return true;
    return false;
}

/**
 * @brief To find orientation of ordered triplet (p, q, r).
 * 
 * @param p Eigen::Vector2d
 * @param q Eigen::Vector2d
 * @param r Eigen::Vector2d
 * @return int - 0 if points are collinear; 1 if the points are in clockwise order; 2 if the points are in anticlockwise order
 */
int ZMPControl::orientation(Eigen::Vector2d p, Eigen::Vector2d q, Eigen::Vector2d r)
{
    int val = (q[1] - p[1]) * (r[0] - q[0]) -
              (q[0] - p[0]) * (r[1] - q[1]);

    if (val == 0)
        return 0;             // colinear
    return (val > 0) ? 1 : 2; // clock or counterclock wise
}

/**
 * @brief This function checks for intersection of two lines given its 2D coordinates
 * 
 * @param p1 Eigen::Vector2d - Coordinates of line 1
 * @param q1 Eigen::Vector2d - Coordinates of line 1
 * @param p2 Eigen::Vector2d - Coordinates of line 2
 * @param q2 Eigen::Vector2d - Coordinates of line 2
 * @return true The two given lines do intersect
 * @return false The two lines doesn't intersect
 */
bool ZMPControl::doIntersect(Eigen::Vector2d p1, Eigen::Vector2d q1, Eigen::Vector2d p2, Eigen::Vector2d q2)
{
    // Find the four orientations needed for general and
    // special cases
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);

    // General case
    if (o1 != o2 && o3 != o4)
        return true;

    // Special Cases
    // p1, q1 and p2 are colinear and p2 lies on segment p1q1
    if (o1 == 0 && onSegment(p1, p2, q1))
        return true;

    // p1, q1 and p2 are colinear and q2 lies on segment p1q1
    if (o2 == 0 && onSegment(p1, q2, q1))
        return true;

    // p2, q2 and p1 are colinear and p1 lies on segment p2q2
    if (o3 == 0 && onSegment(p2, p1, q2))
        return true;

    // p2, q2 and q1 are colinear and q1 lies on segment p2q2
    if (o4 == 0 && onSegment(p2, q1, q2))
        return true;

    return false; // Doesn't fall in any of the above cases
}