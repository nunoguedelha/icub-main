#ifndef __SAFE_MOTION_CONTROL_H__
#define __SAFE_MOTION_CONTROL_H__

#include <yarp/os/RateThread.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/Wrapper.h>
#include <yarp/dev/ISafeControl.h>

#include <yarp/dev/ControlBoardInterfaces.h>

#include <RobotModel.h>
#include <R1Model.h>

#include <Matrix.h>

using namespace icub::robot_model;

#define MARGIN 0.05
#define PERIOD 0.01

#define EOB -1

class SafeMotionControl : 
	public yarp::os::RateThread, 
	public yarp::dev::DeviceDriver,
	public yarp::dev::IMultipleWrapper,
    public yarp::dev::ISafeControl
{
public:
    void init_maps()
    {
        int j = 0;
        int b = 0;

        brd_map[BRD_TORSO][b++] = j++; 
        brd_map[BRD_TORSO][b++] = j++;
        brd_map[BRD_TORSO][b++] = EOB;
        
        b = 0;

        brd_map[BRD_LEFT_UPPER_ARM][b++] = j++;
        brd_map[BRD_LEFT_UPPER_ARM][b++] = j++;
        brd_map[BRD_LEFT_UPPER_ARM][b++] = j++;
        brd_map[BRD_LEFT_UPPER_ARM][b++] = j++;
        brd_map[BRD_LEFT_UPPER_ARM][b++] = EOB;

        b = 0;

        brd_map[BRD_LEFT_LOWER_ARM][b++] = j++;
        brd_map[BRD_LEFT_LOWER_ARM][b++] = j++;
        brd_map[BRD_LEFT_LOWER_ARM][b++] = j++;
        brd_map[BRD_LEFT_LOWER_ARM][b++] = j++;
        brd_map[BRD_LEFT_LOWER_ARM][b++] = EOB;

        b = 0;

        brd_map[BRD_RIGHT_UPPER_ARM][b++] = j++;
        brd_map[BRD_RIGHT_UPPER_ARM][b++] = j++;
        brd_map[BRD_RIGHT_UPPER_ARM][b++] = j++;
        brd_map[BRD_RIGHT_UPPER_ARM][b++] = j++;
        brd_map[BRD_RIGHT_UPPER_ARM][b++] = EOB;

        b = 0;

        brd_map[BRD_RIGHT_LOWER_ARM][b++] = j++;
        brd_map[BRD_RIGHT_LOWER_ARM][b++] = j++;
        brd_map[BRD_RIGHT_LOWER_ARM][b++] = j++;
        brd_map[BRD_RIGHT_LOWER_ARM][b++] = j++;
        brd_map[BRD_RIGHT_LOWER_ARM][b++] = EOB;

        b = 0;

        brd_map[BRD_HEAD][b++] = j++;
        brd_map[BRD_HEAD][b++] = j++;
        brd_map[BRD_HEAD][b++] = EOB;

        /////////////////////////////

        j = b = 0;

        wrp_map[WRP_TORSO][b++] = j++;
        wrp_map[WRP_TORSO][b++] = j++;
        wrp_map[WRP_TORSO][b++] = j++;

        b = 0;

        wrp_map[WRP_TORSO_PROSUP][b++] = j++;

        b = 0;

        wrp_map[WRP_LEFT_ARM][b++] = j++;
        wrp_map[WRP_LEFT_ARM][b++] = j++;
        wrp_map[WRP_LEFT_ARM][b++] = j++;
        wrp_map[WRP_LEFT_ARM][b++] = j++;
        wrp_map[WRP_LEFT_ARM][b++] = j++;

        b = 0;

        wrp_map[WRP_LEFT_ARM_WRIST][b++] = j++;
        wrp_map[WRP_LEFT_ARM_WRIST][b++] = j++;
        wrp_map[WRP_LEFT_ARM_WRIST][b++] = j++;

        b = 0;

        wrp_map[WRP_RIGHT_ARM][b++] = j++;
        wrp_map[WRP_RIGHT_ARM][b++] = j++;
        wrp_map[WRP_RIGHT_ARM][b++] = j++;
        wrp_map[WRP_RIGHT_ARM][b++] = j++;
        wrp_map[WRP_RIGHT_ARM][b++] = j++;

        b = 0;

        wrp_map[WRP_RIGHT_ARM_WRIST][b++] = j++;
        wrp_map[WRP_RIGHT_ARM_WRIST][b++] = j++;
        wrp_map[WRP_RIGHT_ARM_WRIST][b++] = j++;

        b = 0;

        wrp_map[WRP_HEAD][b++] = j++;
        wrp_map[WRP_HEAD][b++] = j++;
    }

	SafeMotionControl() : RateThread(int(PERIOD*1000.0))
	{
        robotModel = new icub::robot_model::r1::R1Model();
	}

    virtual ~SafeMotionControl()
    {
        if (robotModel) delete robotModel;
    }

	void run()
	{
		static Matrix dQ, dD;

        getPos(mQ);
		
		robotModel->setPose(mQ);
		robotModel->calcInterference(mD);

		bool safe = true;

		for (int i = 0; i < mD.R; ++i)
		{
			if (mD(i) <= MARGIN) safe = false;
		}

		if (safe)
        {
            static unsigned int n = 0;
            static int noflood = 0;

            if (++noflood >= 200)
            {
                noflood = 0;
                printf("\nsafe ABS %d\n\n", ++n);
                mD.t().dump();
            }

            return;
        }

        getVel(mV);

		dQ = mQ + mV*PERIOD;

		robotModel->setPose(dQ);
		robotModel->calcInterference(dD);

		safe = true;

		for (int i = 0; i < mD.R; ++i)
		{
			if (dD(i) <= MARGIN && dD(i) <= mD(i))
			{
				safe = false;
			}
		}

        if (!safe)
        {
            static unsigned int n = 0;
            static int noflood = 0;

            if (++noflood >= 50)
            {
                noflood = 0;
                printf("\nUNSAFE %d\n\n", ++n);
                mD.t().dump();
            }
        }
        else
        {
            static unsigned int n = 0;
            static int noflood = 0;

            if (++noflood >= 200)
            {
                noflood = 0;
                printf("\nsafe REL %d\n\n", ++n);
                mD.t().dump();
            }
        }
	}

    bool checkVelocity(int part, double *v)
    {
        return true;
    }

    bool checkVelocity(int part, int njoints, int *joints, double *v)
	{
		static Matrix dQ, dD;

        getPos(mQ);

        robotModel->setPose(mQ);
        robotModel->calcInterference(mD);

        dQ.resize(mQ.R);

        for (int i = 0; i < mQ.R; ++i) dQ(i) = mQ(i) + v[i]*PERIOD;

		robotModel->setPose(dQ);
		robotModel->calcInterference(dD);

		bool safe = true;

		for (int i = 0; i < dD.R; ++i)
		{
			if (dD(i) <= MARGIN && dD(i) <= mD(i))
			{
				safe = false;
			}
		}

		if (safe) return true;

		for (int j = 0; j < NDOF; ++j)
		{
			dQ = mQ;

			dQ(j) += v[j]*PERIOD;

			robotModel->setPose(dQ);
			robotModel->calcInterference(dD);

			for (int i = 0; i < dD.R; ++i)
			{
				if (dD(i) <= MARGIN && dD(i) <= mD(i))
				{
					v[j] = 0.0;

					break;
				}
			}
		}

		return false;
	}

    bool checkPosition(int part, double *p)
    {
        return true;
    }

	bool checkPosition(int part, int njoints, int *joints, double *p)
	{
        getPos(mQ);

        robotModel->setPose(mQ);
        robotModel->calcInterference(mD);

        /////////////////////////////////

        static Matrix dQ, dD;

        dQ = mQ;

        for (int j = 0; j < njoints; ++j) dQ(joints[j]) = p[j];

		robotModel->setPose(dQ);
		robotModel->calcInterference(dD);

		bool safe = true;

		for (int i = 0; i < dD.R; ++i)
		{
			if (dD(i) <= MARGIN && dD(i) <= mD(i))
			{
				safe = false;
			}
		}

		if (safe) return true;

		for (int j = 0; j < njoints; ++j)
		{
			dQ = mQ;

			dQ(joints[j]) = p[j];

			robotModel->setPose(dQ);

			robotModel->calcInterference(dD);

			for (int i = 0; i < dD.R; ++i)
			{
				if (dD(i) <= MARGIN && dD(i) <= mD(i))
				{
					p[j] = mQ(joints[j]);

					break;
				}
			}
		}

		return false;
	}

	bool attachAll(const yarp::dev::PolyDriverList &p)
	{
        int njoints = 0;

		for (int i = 0; i < p.size(); ++i)
		{            
            /* /hardware/motorControl/cer_safe-mc.xml
            <devices robot = "CER01" build = "1">
                <device name = "cer_safe_mc" type = "safeMotionControl">
                <action phase = "startup" level = "7" type = "attach">
                <paramlist name = "networks">
                <!--The param value must match the device name in the corresponding emsX file-->
                <elem name = "safe_board_0"> cer_torso_mc < / elem>
                <elem name = "safe_board_1"> cer_left_upper_arm_mc < / elem>
                <elem name = "safe_board_2"> cer_left_lower_arm_mc < / elem>
                <elem name = "safe_board_3"> cer_right_upper_arm_mc < / elem>
                <elem name = "safe_board_4"> cer_right_lower_arm_mc < / elem>
                <elem name = "safe_board_5"> cer_head_mc < / elem>
                < / paramlist>
                < / action>

                <action phase = "shutdown" level = "7" type = "detach" / >

                < / device>
            < / devices>
            */

            yarp::os::Bottle options = p[i]->poly->getOptions();

            yarp::os::ConstString boardname = p[i]->key.c_str();

            char boardtag[16];
            int boardID = EOB;

            for (int b = 0; b < 16; ++b)
            {
                sprintf(boardtag, "safe_board_%d", b);

                if (boardname == boardtag)
                {
                    boardID = b;
                }
            }

            if (boardID == EOB) continue;

            yarp::dev::PolyDriver* partdrivers = p[i]->poly;

            if (!partdrivers || !partdrivers->isValid())
            {
                printf("\n%s drivers not found\n\n", boardname);
                return false;
            }
            else
            {
                printf("\n%s drivers found\n\n", boardname);
            }

            partdrivers->view(pEncoder[boardID]);
            partdrivers->view(pPosCtrl[boardID]);
            partdrivers->view(pVelCtrl[boardID]);
            partdrivers->view(pDirCtrl[boardID]);
            partdrivers->view(pControl[boardID]);

            if (!pEncoder[boardID]) { printf("\npEncoder[%s] == NULL\n\n", boardname); return false; }
            if (!pPosCtrl[boardID]) { printf("\npPosCtrl[%s] == NULL\n\n", boardname); return false; }
            if (!pVelCtrl[boardID]) { printf("\npVelCtrl[%s] == NULL\n\n", boardname); return false; }
            if (!pDirCtrl[boardID]) { printf("\npDirCtrl[%s] == NULL\n\n", boardname); return false; }
            if (!pControl[boardID]) { printf("\npControl[%s] == NULL\n\n", boardname); return false; }
		
            int na;
            pEncoder[boardID]->getAxes(&na);
            njoints += na;
        }

        mQ.resize(njoints, 1, true);
	    mV.resize(njoints, 1, true);
        mD.resize(njoints, 1, true);

        start();

		return true;
	}

	bool detachAll()
	{
        if (isRunning()) stop();

		return true;
	}

    int bind(std::string part)
    {
        return robotModel->bind(part);
    }

protected:
    enum { BRD_TORSO, BRD_LEFT_UPPER_ARM, BRD_LEFT_LOWER_ARM, BRD_RIGHT_UPPER_ARM, BRD_RIGHT_LOWER_ARM, BRD_HEAD, N_BOARDS };
    enum { WRP_TORSO, WRP_TORSO_PROSUP, WRP_WRP_HEAD, WRP_LEFT_ARM, WRP_LEFT_ARM_WRIST, WRP_RIGHT_ARM, WRP_RIGHT_ARM_WRIST, WRP_HEAD, N_WRAPPERS };

    yarp::dev::IEncoders          *pEncoder[N_BOARDS];
    yarp::dev::IPositionControl2  *pPosCtrl[N_BOARDS];
    yarp::dev::IVelocityControl2  *pVelCtrl[N_BOARDS];
    yarp::dev::IPositionDirect    *pDirCtrl[N_BOARDS];
    yarp::dev::IControlMode2      *pControl[N_BOARDS];

	Matrix mQ;
	Matrix mV;
	Matrix mD;

	int NDOF;

	icub::robot_model::RobotModel* robotModel;

    int wrp_map[16][16];
    int brd_map[16][16];

    virtual void getPos(Matrix& q)
    {
        double pos[16];

        for (int b = 0; b < N_BOARDS; ++b)
        {
            pEncoder[b]->getEncoders(pos);

            for (int j = 0; brd_map[b][j] != EOB; ++j)
            {
                q(brd_map[b][j]) = pos[j];
            }
        }
    }

    virtual void getVel(Matrix& v)
    {
        double vel[16];

        for (int b = 0; b < N_BOARDS; ++b)
        {
            pEncoder[b]->getEncoderSpeeds(vel);

            for (int j = 0; brd_map[b][j] != EOB; ++j)
            {
                v(brd_map[b][j]) = vel[j];
            }
        }
    }
};

#endif
