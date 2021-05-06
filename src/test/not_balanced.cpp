

void robot::not_balanced(Eigen::VectorXd q_limits_min, Eigen::VectorXd q_limits_max)
{
    /// *********************************************** ///
    /// Move the joints to achieve a balanced position. ///
    /// *********************************************** ///

    std::cout << "\033[1;31m NOT BALANCED \033[0m" << std::endl;
    std::cout <<std::endl;

    //int modifiedConfiguration[] = {0,1,2,3,5,9,10,11,12,13,14,15,16,17,18,19,20,21,23}; /// Without LHipYawPitch(ID:8), LHand(7), Rhand(25)
                                                                                      /// RWristYaw(24), LWristYaw(6), LElbowYaw(4), RElbowYaw(22)

    int modifiedConfiguration[] = {16,17,11,12,19,14,6,7,4,3,8,2};

    int lenght = m_q_current_mod.size();

    Eigen::Vector2d CoM_current, CoM_desired;

    std::vector<float> dq_minimization(lenght,0);
    Eigen::VectorXd ZAngleDiff(lenght), ZLimits(lenght);

    Eigen::Vector2d dCoM;
    Eigen::MatrixXd JacobianXY_CoM_RFoot(2,lenght);

    Eigen::VectorXd q_limits_mean(lenght), q_limits_delta(lenght);
    q_limits_mean = (q_limits_max+q_limits_min)/2.0;
    q_limits_delta = q_limits_max-q_limits_min;

    Eigen::VectorXd dq(lenght),dq1(lenght),dq2(lenght);

    float weightAngleDiff = -0.02; /// Weight of a minimization task (-)
    float weightLimit = -0.1; /// Weight of a minimization task (-)

    int iIteration = 1, maxIterations = 10;
    float max_error_CoM = 0.1;
    int Sp_CoM = 10; // Threshold (milimeters) of how big the step of the IK can be in task space (dXp_CoM)

    bool ClampingOn = true, SolutionFound = false, ReducedX = true;

    Eigen::VectorXd q_results(lenght);

    Eigen::MatrixXd q_results_bySteps;

    //std::vector<float> q(lenght,0);

    std::cout << "desired joint configuration: " << std::endl;
    std::cout << m_q_desired_mod.transpose() << std::endl;

    std::cout << "current joint configuration: " << std::endl;
    std::cout << m_q_current_mod.transpose() << std::endl;

    CoM_desired(0) = m_pos_CoM[0];
    CoM_desired(1) = m_pos_CoM[1];

    if( m_pos_CoM[0] > m_security_XY_CoM[0]) ///m_pos_CoM_x > CoM_max_x
    {
        CoM_desired[0] = m_security_XY_CoM[0];
    }
    if(m_pos_CoM[0] < m_security_XY_CoM[1]) ///m_pos_CoM_x < CoM_min_x
    {
        CoM_desired[0] = m_security_XY_CoM[1];
    }
    if(m_pos_CoM[1] > m_security_XY_CoM[2]) ///m_pos_CoM_y > CoM_max_y
    {
        CoM_desired[1] = m_security_XY_CoM[2];
    }
    if(m_pos_CoM[1] < m_security_XY_CoM[3]) ///m_pos_CoM_y < CoM_min_y
    {
        CoM_desired[1] = m_security_XY_CoM[3];
    }

//    CoM_desired(0) = m_pos_CoM_mean[0];
//    CoM_desired(1) = m_pos_CoM_mean[1];

    Eigen::VectorXd q_current = m_q_current_mod;
    Eigen::VectorXd q_desired = m_q_desired_mod;
    std::vector<float> q(lenght,0);

    while(1)
    {
        std::cout << std::endl << "Beginning the iteration loop..." << std::endl << std::endl;

        for(int i=0;i<lenght;i++)
        {
            q[i] = q_current[i];
        }

        std::cout << "Current joint configuration : " << std::endl << q << std::endl;
        CoM_current = GetXY_CoM_RFoot(q);
        JacobianXY_CoM_RFoot = GetJacobianXY_CoM_RFoot(q);
        JacobianEndEff =
        /// Reduce XY_CoM_desired to the allowed step -- only once per solve
        if(ReducedX){
            ReducedX = false;

            float dCoM_X	  =  CoM_desired[0]-CoM_current(0);
            float dCoM_Y	  =  CoM_desired[1]-CoM_current(1);

            // This will only work for mm. Radians will be lower than Sp
            if(abs(dCoM_X)>Sp_CoM) {
                dCoM_X = dCoM_X/abs(dCoM_X)*Sp_CoM;
            }
            if(abs(dCoM_Y)>Sp_CoM) {
                dCoM_Y = dCoM_Y/abs(dCoM_Y)*Sp_CoM;
            }
            //CoM_desired[0] = dCoM_X+CoM_current(0);
            //CoM_desired[1] = dCoM_Y+CoM_current(1);

            //std::cout << "Desired XY CoM: " << std::endl << CoM_desired << std::endl;
            //std::cout << "Current XY CoM: " << std::endl << CoM_current << std::endl;
            //dCoM(0) = (CoM_desired[0] - CoM_current[0]);
            //dCoM(1) = (CoM_desired[1] - CoM_current[1]);
        }

        /// Difference between current CoM projection and desired one
        /// CoM_desired is different for each not balanced behaviour
        std::cout << "Desired XY CoM: " << std::endl << CoM_desired << std::endl;
        std::cout << "Current XY CoM: " << std::endl << CoM_current << std::endl;
        dCoM(1) = (CoM_desired[1] - CoM_current[1]);
        dCoM(0) = (CoM_desired[0] - CoM_current[0]);

        // CHECK IF SOLUTION WAS FOUND
        // Limits
        bool LimitsOK = true;
        for(int i=0; i<lenght; i++)
            if (q_current[i] < q_limits_min(i) || q_current[i] > q_limits_max(i)){
                LimitsOK = false;
                std::cout << " i: " << i;
            }
        if (!LimitsOK) std::cout << std::endl;

        // Balance
        std::cout << "|CoM_desired_x - CoM_current_x| = " << std::endl << fabs(dCoM(0)) << std::endl;
        std::cout << "|CoM_desired_y - CoM_current_y| = " << std::endl << fabs(dCoM(1)) << std::endl;
        bool BalanceOK = true;
        if ((fabs(dCoM(0))>max_error_CoM)||(fabs(dCoM(1))>max_error_CoM))
        {
            BalanceOK = false;
        }

        std::cout << std::endl;
        std::cout << "Balance: " << BalanceOK << std::endl;
        std::cout << "Limits: " << LimitsOK << std::endl;
        std::cout << std::endl;
        // Complete check
        if(LimitsOK && BalanceOK) {
            SolutionFound = true;
            m_q_current_mod = q_current;
            std::cout << "\033[1;32mConversion!!! In " << iIteration << " iterations. \033[0m" << std::endl; // in dark green
            break;
        }

        /// The limit of iterations has been reached
        if(iIteration>maxIterations) {
            /// The results stay the same as the previous configuration.
            std::cout << "\033[1;31mNo conversion after " << iIteration << " iterations.\033[0m";
            SolutionFound = false;

            if(!LimitsOK) {
                std::cout << "\033[1;31m LIMITS FAIL \033[0m";
            } else {
                std::cout << "\033[1;32m LIMITS OK \033[0m";
            }
            if(!BalanceOK) {
                std::cout << "\033[1;31m BALANCE FAIL \033[0m";
            } else {
                std::cout << "\033[1;32m BALANCE OK \033[0m";
            }

            break;
        }

        /// Minimize joint differences
        /// Difference between desired joint angles and current ones
        for (int i = 0; i<lenght;i++) {
            if (std::count(modifiedConfiguration,modifiedConfiguration+12,i)==1)
            {
                dq_minimization[i] = q_current[i]  - q_desired[i];
            }
            else
            dq_minimization[i] = 0;
        }

        /// Low priority: Optimization tasks -- AngleDifferenciation & JointLimits
        for (int i=0; i<lenght; i++)
        {
            ZAngleDiff(i) = 2.0*weightAngleDiff*dq_minimization[i];
            ZLimits(i) = (2.0*weightLimit*(q_current[i]-q_limits_mean(i)))/(pow(q_limits_delta(i),2));
        }

        /// Choose priority -- tracking / balance
        /// To choose, exchange J1 and J2, X1 and X2, dimension 1 and dimension2.
        /// It's also possible to experiment only with a specific end-effector, by using their Jacobians defined above and adjusting dX and dimension accordingly
        int dimension1 = JacobianXY_CoM_RFoot.rows();
        int dimension2 = JacobianEndEff.rows();

        Eigen::VectorXd q_current_temp(lenght),q_current_temp2(lenght);

        /// Priority 1 -> BALANCE
        Eigen::MatrixXd J_CoM  =  JacobianXY_CoM_RFoot;
        Eigen::VectorXd dX_CoM = dCoM;
        /// Priority 1 -> Track EndEffector
        Eigen::MatrixXd J_Eff  = JacobianEndEff;
        Eigen::VectorXd dX_Eff = dX;



        /// ESSAI --> Converge vers 0???
        int lambda = 1;
        //dX_CoM = -lambda*dX_CoM;

        /// Clamping loop according to Baerlocher
        /// Clamps joints that are outside limits to their limits and solves IK with the rest of joints
        bool Clamping = true;
        while (Clamping){
            Clamping = false;

            std::cout << std::endl << "Begin the clamping loop..." << std::endl << std::endl;

            std::cout << "CoM_desired - CoM_current = " << std::endl << dCoM << std::endl;

            /// Get pseudo inverses
            /// These matrices are sparse and wide. The right pinv seems to be working, but I don't know how robust it is.
//            Eigen::MatrixXd J_CoM_i = pinvRight2(J_CoM);


            ///Projection matrix.
//            Eigen::MatrixXd P_CoM = Eigen::MatrixXd::Identity(lenght,lenght) - J_CoM_i*J_CoM;
//
//            dq2 = J_CoM_i*dX_CoM;
//            dq1 = dq2 + P_CoM*ZLimits;
//            dq = dq1 + P_CoM*ZAngleDiff;

            std::cout << std::endl << "Première méthode: " << std::endl << std::endl;

            Eigen::MatrixXd J_CoM2=J_CoM;
            CompleteOrthogonalDecomposition<Ref<MatrixXd> > COD(J_CoM);
            dq = COD.solve(dX_CoM);

            // Keep the value without dq added in case the Eigen library goes crazy...? Did I solve this by putting zeros in the Jacobians?
            q_current_temp = q_current;

            Eigen::VectorXd V = J_CoM2*dq;

            q_current_temp2 = q_current + dq;

            std::cout << "q_current: " << std::endl << q_current_temp.transpose() << std::endl << std::endl;

            std::cout << std::endl << "|J_CoM*dq - dX_CoM| = " << (V - dX_CoM).norm() << std::endl;

            std::cout << "dq= " << std::endl << dq.transpose() << std::endl << std::endl;

            std::cout << std::endl << "J_CoM*dq = " << V << std::endl;
            std::cout << std::endl << "dX_CoM = " << dX_CoM << std::endl;

            std::cout << "new q_current (q_d): " << std::endl << q_current_temp2.transpose() << std::endl << std::endl;

            for(int i = 0; i<lenght;i++)
            {
                q[i] =  q_current_temp2(i);
            }

            Eigen::Vector2d new_CoM_pos = GetXY_CoM_RFoot(q);

            std::cout << "CoM with new q_current: " << std::endl << new_CoM_pos << std::endl << std::endl;

            Eigen::Vector2d new_dCoM = (CoM_desired - new_CoM_pos);

            std::cout << "dX_CoM: " << std::endl << new_dCoM << std::endl << std::endl;

            /// -------------------------------------------------------------------------------------------------///

            std::cout << std::endl << "Deuxième méthode: " << std::endl << std::endl;

            Eigen::MatrixXd J_CoM3=J_CoM2;
            Eigen::Vector2d dX_CoM2, dX_CoM3;
            Eigen::VectorXd q_d(lenght);
            dX_CoM3 = J_CoM3*q_current_temp;
            dX_CoM2 = dX_CoM + dX_CoM3;
            CompleteOrthogonalDecomposition<Ref<MatrixXd> > COD2(J_CoM2);
            q_d = COD2.solve(dX_CoM2);

            Eigen::VectorXd V2 = J_CoM3*q_d;

            Eigen::VectorXd V3 = J_CoM3*q_d - J_CoM3*q_current_temp;

            std::cout << std::endl << "|J_CoM*q_d - J_CoM*q_current - dX_CoM| = " << (V2 - dX_CoM2).norm() << std::endl;

            std::cout << std::endl << "dq = " << (q_d-q_current_temp).transpose() << std::endl;

            std::cout << std::endl << "J_CoM*q_d - J_CoM*q_current = " << V3 << std::endl;

            std::cout << "new new q_current (q_d): " << std::endl << q_d.transpose() << std::endl << std::endl;

            for(int i = 0; i<lenght;i++)
            {
                q[i] =  q_d(i);
            }

            Eigen::Vector2d new_CoM_pos2 = GetXY_CoM_RFoot(q);

            std::cout << "CoM with new new q_current: " << std::endl << new_CoM_pos2 << std::endl << std::endl;

            Eigen::Vector2d new_dCoM2 = (CoM_desired - new_CoM_pos2);

            std::cout << "new dX_CoM: " << std::endl << new_dCoM2 << std::endl << std::endl;


            /// ON CHOISIT LA PREMIERE METHODE. POURQUOI????
            //q_current = q_current_temp2;
            //q_current_temp2 = q_d;
            q_current = q_current_temp2;

            // Baerlocher
            // Check if result is within joint limits
            if(ClampingOn) {

                double margin = 0.01;
                for(int i=0; i<lenght; i++) {
                    if (q_current_temp2[i] < q_limits_min(i) || q_current_temp2(i) > q_limits_max(i)){
                            std::cout << "Joint over its limits" << std::endl;
                            Clamping = true;
                            // Back to previous solution
                            //qq_current[i] = q_current_temp(i);
                            //q_current = q_current_temp;
                            // Fix the trouble angle to its limit
                            if (q_current_temp2(i) < q_limits_min(i) ) {
                                q_current[i] = q_limits_min(i)+margin;
                                std::cout << "Joint " << i+1 << " (" << i << ") clamped to its min limit." << std::endl;
                            }
                            if (q_current_temp2(i) > q_limits_max(i) ) {
                                q_current[i] = q_limits_max(i)-margin;
                                std::cout << "Joint " << i+1 << " (" << i << ") clamped to its max limit." << std::endl;
                            }
                            // Zero column i of the Jacobian
                            for (int j=0; j<dimension1; j++) J_CoM(j,i) = 0;
                    }
                    else
                        std::cout << "Joints in their limits!!!" << std::endl;
                }
            }

        //q_current_mod = q_current_temp2;
        } /// end clamping loop

        std::cout << std::endl << "Clamping loop ended" << std::endl << std::endl;

        if (iIteration == 1) {
            q_results_bySteps = dq.transpose();
        }
        else {
            // Append this dq
            int numberRows = q_results_bySteps.rows();
            q_results_bySteps.conservativeResize(numberRows+1,Eigen::NoChange);
            q_results_bySteps.row(numberRows) = dq.transpose();
        }

        // This makes sure no crazy results come out, because sometimes nans and infs come and destroy everything
        for(int i=0; i<lenght; i++) {
            if( q_current[i] != q_current[i] )	// check if its NaN
                q_current[i] = q_current_temp(i);
            if( q_current[i] > 1.0e2)// thats a very large upper limit, no?
            {
                std::cout << "inf " << i << std::endl;
                q_current[i] = q_current_temp(i);
            }
        }

        // Making sure the results are in the desired quadrant -- it seems to always be --  get rid of temp2?
        // Limit to 2*pi
        for(int i=0; i<lenght; i++)
            q_current[i] = fmod((double) q_current[i], 2*M_PI);
        // Limit to between -pi and pi
        for(int i=0; i<lenght; i++) {
            if(( q_current[i] >= M_PI ) && ( q_current[i] <= 2*M_PI))
                    q_current[i] = q_current[i] - 2*M_PI;
            else if(( q_current[i] <- M_PI ) && ( q_current[i] >- 2*M_PI ))
                q_current[i] = q_current[i] + 2*M_PI;
        }

        std::cout << "Iteration n° " << iIteration << std::endl;
        std::cout << std::endl << std::endl;
        iIteration=iIteration+1;
        std::cout << "new q_current: " << std::endl << q_current << std::endl << std::endl;

    } /// end of while loop
    if(SolutionFound)
    {
        std::cout << "\033[1;32m SOLUTION FOUND \033[0m" << std::endl << std::endl;
        m_q_results_byStepsFinal = q_results_bySteps;
        q_results_bySteps.conservativeResize(1,lenght);
    }

    std::cout << std::endl << std::endl;

}

void robot::DKM()
{

    MatrixXd JacobianP_LHand(3,23), JacobianP_RHand(3,23), JacobianP_LAnkle(3,23), JacobianZ_LFoot(1,23), JacobianXY_CoM(2,23), new_JacobianEndEff(11,23);
    MatrixXd JacobianZ_LToe(1,23);
    MatrixXd JacobianR_LFoot(3,23);
	JacobianP_LHand = GetJacobianP_LHand_RFoot(std::vector<float> &q);
	JacobianP_RHand = GetJacobianP_RHand_RFoot(std::vector<float> &q);
	JacobianP_LAnkle = GetJacobianP_LAnkle_RFoot(std::vector<float> &q);
    JacobianZ_LFoot = GetJacobianZ_LFoot_RFoot(std::vector<float> &q);
    JacobianZ_LToe = GetJacobianZ_LToe_RFoot(std::vector<float> &q);
	JacobianXY_CoM  = GetJacobianXY_CoM_RFoot  (std::vector<float> &q);

	// Concatenate end-effector Jacobians and task vector
	for(int i=0; i<23; i++) {
        new_JacobianEndEff(0,i) = JacobianP_LHand(0,i);
		new_JacobianEndEff(1,i) = JacobianP_LHand(1,i);
		new_JacobianEndEff(2,i) = JacobianP_LHand(2,i);
		new_JacobianEndEff(3,i) = JacobianP_RHand(0,i);
		new_JacobianEndEff(4,i) = JacobianP_RHand(1,i);
        new_JacobianEndEff(5,i) = JacobianP_RHand(2,i);
        new_JacobianEndEff(6,i) = JacobianP_LAnkle(0,i);
        new_JacobianEndEff(7,i) = JacobianP_LAnkle(1,i);
        new_JacobianEndEff(8,i) = JacobianP_LAnkle(2,i);
        new_JacobianEndEff(9,i) = JacobianZ_LFoot(0,i);
        new_JacobianEndEff(10,i) = JacobianZ_LToe(0,i);
	}

    m_JacobianCoM = JacobianXY_CoM;
	m_JacobianEndEff = new_JacobianEndEff;
}

void robot::DGM()
{
    Vector3d P_LHand_current, P_RHand_current, P_LAnkle_current, R_LFoot_current;
    double Z_LFoot_current, Z_LToe_current;
	Vector2d new_XY_CoM_current;

	P_LHand_current = GetP_LHand_RFoot(std::vector<float> &q);
	P_RHand_current = GetP_RHand_RFoot(std::vector<float> &q);
	P_LAnkle_current = GetP_LAnkle_RFoot(std::vector<float> &q);
    Z_LFoot_current = GetZ_LFoot_RFoot(std::vector<float> &q);
    Z_LToe_current = GetZ_LToe_RFoot(std::vector<float> &q);
	new_XY_CoM_current = GetXY_CoM_RFoot(std::vector<float> &q);

    int sizeX = 11;
    VectorXd new_X_current(sizeX);
    new_X_current(0) = P_LHand_current(0);
	new_X_current(1) = P_LHand_current(1);
	new_X_current(2) = P_LHand_current(2);
	new_X_current(3) = P_RHand_current(0);
	new_X_current(4) = P_RHand_current(1);
    new_X_current(5) = P_RHand_current(2) ;
    new_X_current(6) = P_LAnkle_current(0);
    new_X_current(7) = P_LAnkle_current(1);
    new_X_current(8) = P_LAnkle_current(2);
    new_X_current(9) = Z_LFoot_current;
    new_X_current(10) = Z_LToe_current;

	m_X_current = new_X_current;
	m_XY_CoM_current = new_XY_CoM_current;

    // Reduce X_desired and XY_CoM_desired to the allowed step -- only once per solve
    if(ReducedX){
        ReducedX = false;
        VectorXd dX_ = X_desired - X_current;

        double dCoM_X	  =  XY_CoM_desired(0)-new_XY_CoM_current(0);
        double dCoM_Y	  =  XY_CoM_desired(1)-new_XY_CoM_current(1);


        // This will only work for mm. Radians will be lower than Sp
        for(int j = 0; j < sizeX; j++){
            if(abs(dX_(j))>Sp)
                dX_(j) = dX_(j)/abs(dX_(j))*Sp;
        }
        if(abs(dCoM_X)>Sp_CoM) {
            dCoM_X = dCoM_X/abs(dCoM_X)*Sp_CoM;
        }
        if(abs(dCoM_Y)>Sp_CoM) {
            dCoM_Y = dCoM_Y/abs(dCoM_Y)*Sp_CoM;
        }
        XY_CoM_desired << dCoM_X+new_XY_CoM_current(0),dCoM_Y+new_XY_CoM_current(1);
        X_desired << dX_+new_X_current;
    }
    //cout << "X_desired: " << X_desired.transpose() << endl;

}

