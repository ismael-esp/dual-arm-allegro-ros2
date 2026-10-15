#ifndef CAN_AH_H_
#define CAN_AH_H_

#include <string>
// #include <libpcan.h>
#include <PCANBasic.h>
// #include "CAN_Version.h"
// #include <vector>

#include "dynamics_AH.h"

#include <kdl/chain.hpp>
#include <thread>
#include <mutex>

#include <urdf/model.h>
#include <kdl/frames_io.hpp>
#include "kdl_parser/kdl_parser.hpp"

#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <list>
#include <yaml-cpp/yaml.h>

#define PWM_LIMIT_ROLL 250.*1.5
#define PWM_LIMIT_NEAR 450.*1.5
#define PWM_LIMIT_MIDDLE 300.*1.5
#define PWM_LIMIT_FAR 190.*1.5

#define PWM_LIMIT_THUMB_ROLL 350.*1.5
#define PWM_LIMIT_THUMB_NEAR 270.*1.5
#define PWM_LIMIT_THUMB_MIDDLE 180.*1.5
#define PWM_LIMIT_THUMB_FAR 180.*1.5

#define MAX_BUS             (256)
typedef char    TCHAR;
#define _tcsicmp(x, y)   strcmp(x, y)


#define ID_CMD_QUERY_STATE_DATA     0x0e
#define ID_CMD_QUERY_CONTROL_DATA   0x0f
#define ID_COMMON                   0x01
#define ID_CMD_SET_MODE_JOINT       0x04
#define ID_CMD_SET_MODE_TASK        0x05
#define ID_DEVICE_MAIN              0x02
#define ID_RTR_FINGER_POSE              0x20
#define ID_RTR_FINGER_POSE_1            (ID_RTR_FINGER_POSE+0)
#define ID_RTR_FINGER_POSE_2            (ID_RTR_FINGER_POSE+1)
#define ID_RTR_FINGER_POSE_3            (ID_RTR_FINGER_POSE+2)
#define ID_RTR_FINGER_POSE_4            (ID_RTR_FINGER_POSE+3)
#define ID_CMD_CONFIG                   0x68
#define ID_CMD_SYSTEM_ON                0x40
#define ID_RTR_HAND_INFO                0x80
#define ID_RTR_SERIAL                   0x88
#define ID_CMD_QUERY_ID					0x10
#define ID_RTR_IMU_DATA                 0x30
#define ID_RTR_TEMPERATURE              0x38
#define ID_RTR_TEMPERATURE_1            (ID_RTR_TEMPERATURE+0)
#define ID_RTR_TEMPERATURE_2            (ID_RTR_TEMPERATURE+1)
#define ID_RTR_TEMPERATURE_3            (ID_RTR_TEMPERATURE+2)
#define ID_RTR_TEMPERATURE_4            (ID_RTR_TEMPERATURE+3)


class CAN_AH {
public:
    struct Parameters {
        std::string serial;
        double handVersion;
        struct {
            int j00, j01, j02, j03;
            int j10, j11, j12, j13;
            int j20, j21, j22, j23;
            int j30, j31, j32, j33;
        } encoderOffset, encoderDirection, motorDirection;
    };

    CAN_AH(const std::string &channel, const Parameters &params);

    ~CAN_AH();

    // Subclass for the CAN versions
    class VersionHandler {
    public:
        virtual void readDevices() = 0;
        virtual void writeDevices() = 0;
        virtual void startCAN(int CAN_Ch) = 0;
        virtual int command_can_stop(int bus) = 0;     // Stop communications v3
        int command_can_close(int bus); //Can Close comunications

        double position[16];
        double torque[16];
        TPCANMsg lmsgs[4];  //All hand msgs with timestamp
        double tau_cov_const;
        volatile bool emergencyStop;
        double hand_version;
        TPCANTimestamp timeStamp;
        int encoderOffset[16];
        int encoderDirection[16];
        int motorDirection[16];
        double PWM_MAX[16];
        DWORD ID_MSG_FIRST_FINGER;
        TPCANHandle bus;
        virtual ~VersionHandler() {}
    protected:
        DWORD ID_CMD_SET_SYSTEM_ON;
        DWORD ID_CMD_SET_SYSTEM_OFF;
        DWORD ID_CMD_SET_PERIOD;
        DWORD ID_CMD_SET_TORQUE;
        DWORD ID_CMD_SET_TORQUE_1;
        DWORD ID_CMD_SET_TORQUE_2;
        DWORD ID_CMD_SET_TORQUE_3;
        DWORD ID_CMD_SET_TORQUE_4;
        DWORD ID_DEVICE_SUB_01;
        DWORD ID_DEVICE_SUB_02;
        DWORD ID_DEVICE_SUB_03;
        DWORD ID_DEVICE_SUB_04;

        TPCANRdMsg read_msg;
        unsigned char CAN_ID;
        std::mutex mutex;
        // Variables parseCANMsg
        unsigned char cmd, src;
        unsigned char len;
        unsigned char tmpdata[8];
        int tmppos[4];
        int lIndexBase;

        // Variables
        unsigned char data[8];
        DWORD Txid;
        //char lID;
        int ret;
        int ite;
        unsigned int good_msgs_counter;
        DWORD lIDs[4];
        TPCANStatus rets[4];
        double qs[4][4];
    };
    // Version 3
    class Version3Handler : public VersionHandler {
    public:
        void readDevices() override;
        void writeDevices() override;
        void startCAN(int CAN_Ch) override;
        int command_can_stop(int bus) override;
    private:
        void readMsg();
        int command_can_start_v3(int ch); // Can initialize comunications v3
        int command_set_period(int ch, int period_msec); // Set periodic comunication v3
        int canSendMsg(int bus, int id, char len, unsigned char *data); // Send Message
        int canReadMsg(int bus, int *id, int *len, unsigned char *data);
        int get_message(int ch, char* cmd, char* src, char* des, int* len, unsigned char* data);
        int command_set_torque(int ch, int findex, short* pwm);
        char parseCANMessage(TPCANMsg CANMsg, double *values);
    };
    // Version 4
    class Version4Handler : public VersionHandler {
    public:
        void readDevices() override;
        void writeDevices() override;
        void startCAN(int CAN_Ch) override;
        int command_can_stop(int bus) override;
    private:
        void readMsg();
        int canSentRTR(int bus, int id); // V4. Send RTR message
        int command_set_period(int bus, short* period); // Set periodic comunication v4
        int canSendMsg(int bus, int id, char len, unsigned char *data); //override); Send Message
        int canReadMsg(int bus, int *id, int *len, unsigned char *data);
        int get_message(int ch, int* id, int* len, unsigned char* data);
        int command_set_torque(int ch, int findex, short* pwm);
        char parseCANMessage(TPCANMsg CANMsg, double *values);
    };

    void setTorque(const double *torque);

    void getPosition(double *position);

    void getTimeStamp(TPCANTimestamp &timeStamp_);

    void readDevices();

    void writeDevices();

    bool getEmergencyStop() {return versionHandler->emergencyStop;}

    double getTorqueConversion() { return versionHandler->tau_cov_const;}

    void readDataTimeStamp(TPCANTimestamp &timeStamp_A); //, DWORD &timeStamp_B, DWORD &timeStamp_C, DWORD &timeStamp_D);
    void readDataTimeStamp();

private:
    TPCANHandle bus;
    std::unique_ptr<VersionHandler> versionHandler;
    std::mutex mutex;
    void startCAN(int CAN_Ch);
    TPCANStatus rets[4];
    TPCANMsg CANMsgs[4];
    TPCANMsg CANMsg;
    TPCANTimestamp CANTimeStamp;
    TPCANStatus Status;
    bool notmsg;
    char strMsg[256];
//protected:
    double hand_version;
    // double getHandVersion(){return hand_version;};
    bool run_commLoop; // If true we have a loop that reads the data every 3ms.
    DWORD getIDFirstFinger();
    int GetCANChannelIndex(const TCHAR* cname); // Get the CAN Channel
    int initCAN(int bus); // Initialize CAN comunications in the Channel
    void initV3(const Parameters &params);
    void channelToBus(std::string channel, TPCANHandle &bus);
};

#endif // CAN_AH_H_
