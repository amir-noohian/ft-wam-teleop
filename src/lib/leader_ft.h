#pragma once

#include <boost/asio.hpp>

#include "udp_handler_ft.h"
#include <barrett/detail/ca_macro.h>
#include <barrett/systems/abstract/single_io.h>
#include <barrett/thread/abstract/mutex.h>
#include <barrett/units.h>

template <size_t DOF>
class Leader : public barrett::systems::System {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  public:
    Input<jp_type> wamJPIn;
    Input<jv_type> wamJVIn;
    Input<jt_type> wamGravIn;
    Input<jt_type> wamDynIn;
    Input<jt_type> ftTorqueIn;
    Output<jt_type> wamJPOutput;

    enum class State { INIT, LINKED, UNLINKED };

    explicit Leader(barrett::systems::ExecutionManager* em, char* remoteHost, int rec_port = 5555,
                      int send_port = 5554, const std::string& sysName = "Leader Nowrist")
        : System(sysName)
        , theirJp(0.0)
        , theirJv(0.0)
        , control(0.0)
        , wamJPIn(this)
        , wamJVIn(this)
        , wamGravIn(this)
        , wamDynIn(this)
        , ftTorqueIn(this)
        , wamJPOutput(this, &jtOutputValue)
        , udp_handler(remoteHost, send_port, rec_port)
        , state(State::INIT) {

        // kp << 600, 700, 250, 120;
        // kd << 30, 25, 15, 10;
        kp << 600, 700, 250, 120, 10, 10, 2.5;
        kd << 8.3, 8, 3.3, 0.8, 0.5, 0.5, 0.05;

        if (em != NULL) {
            em->startManaging(*this);
        }
    }

    virtual ~Leader() {
        this->mandatoryCleanUp();
    }

    bool isLinked() const {
        return state == State::LINKED;
    }
    void tryLink() {
        BARRETT_SCOPED_LOCK(this->getEmMutex());
        state = State::LINKED;
    }
    void unlink() {
        BARRETT_SCOPED_LOCK(this->getEmMutex());
        state = State::UNLINKED;
    }

  protected:
    typename Output<jt_type>::Value* jtOutputValue;
    jp_type wamJP;
    jv_type wamJV;
    jt_type wamGrav;
    jt_type wamDyn;
    jt_type wamFT;
    Eigen::Matrix<double, DOF, 1> sendJpMsg;
    Eigen::Matrix<double, DOF, 1> sendJvMsg;
    Eigen::Matrix<double, DOF, 1> sendFtMsg;

    using ReceivedData = typename UDPHandler<DOF>::ReceivedData;

    virtual void operate() {

        wamJP = wamJPIn.getValue();
        wamJV = wamJVIn.getValue();
        wamGrav = wamGravIn.getValue();
        wamDyn = wamDynIn.getValue();
        wamFT = ftTorqueIn.getValue();
        sendJpMsg << wamJP; // added zero to send zero joint positions to the wrist part of the 7-dof follower
        sendJvMsg << wamJV;
        sendFtMsg << wamFT;

        udp_handler.send(sendJpMsg, sendJvMsg, sendFtMsg);

        boost::optional<ReceivedData> received_data = udp_handler.getLatestReceived();
        auto now = std::chrono::steady_clock::now();
        if (received_data && (now - received_data->timestamp <= TIMEOUT_DURATION)) {

            // theirJp = received_data->jp;
            // theirJv = received_data->jv;

            theirJp = received_data->jp.template head<DOF>();
            theirJv = received_data->jv.template head<DOF>();

        } else {
            if (state == State::LINKED) {
                std::cout << "lost link" << std::endl;
                state = State::UNLINKED;
            }
        }

        switch (state) {
            case State::INIT:
                control.setZero();
                jtOutputValue->setData(&control);
                break;
            case State::LINKED:
                // Active teleop. Only the callee can transition to LINKED
                control = compute_control(theirJp, theirJv, wamJP, wamJV, wamGrav, wamDyn, wamFT);
                jtOutputValue->setData(&control);
                break;
            case State::UNLINKED:
                // Changed to unlinked with either timeout or callee.
                control.setZero();
                jtOutputValue->setData(&control);
                break;
        }
    }

    jp_type theirJp;
    jp_type theirJv;
    jt_type control;

  private:
    DISALLOW_COPY_AND_ASSIGN(Leader);
    std::mutex state_mutex;
    jp_type joint_positions;
    UDPHandler<DOF> udp_handler;
    const std::chrono::milliseconds TIMEOUT_DURATION = std::chrono::milliseconds(20);
    State state;
    Eigen::Matrix<double, DOF, 1> kp;
    Eigen::Matrix<double, DOF, 1> kd;

    jt_type compute_control(const jp_type& ref_pos, const jv_type& ref_vel, const jp_type& cur_pos,
                            const jv_type& cur_vel, const jt_type& wam_grav, const jt_type& wam_dyn,
                            const jt_type& cur_ft) {
        jt_type pos_term = kp.asDiagonal() * (ref_pos - cur_pos);
        jt_type vel_term = kd.asDiagonal() * (ref_vel - cur_vel);
        jt_type cur_ft_term = cur_ft;
        jt_type grav_mod = wam_grav;
        grav_mod[4] = 0.0;
        grav_mod[5] = 0.0;
        grav_mod[6] = 0.0;
        jt_type feedforward = wam_dyn - grav_mod;


        jt_type u0 = 0.0 * pos_term;
        jt_type u1 = 1.0 * cur_ft_term; // using f/t information on the leader side to compensate for the dynamic
        jt_type u2 = 0.0 * cur_ft_term; // the case of using the ft information from the leader side on the follower side
        jt_type u3 = 1.0 * cur_ft_term;

        return pos_term + vel_term + u1;
    };
};

