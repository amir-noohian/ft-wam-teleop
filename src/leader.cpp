/*
 * ex11_master_master.cpp
 *
 *  Created on: Feb 22, 2010
 *      Author: Christopher Dellin
 *      Author: Dan Cody
 *      Author: Brian Zenowich
 */

#include <iostream>
#include <string>

#include <boost/thread.hpp>

#include <barrett/detail/stl_utils.h>
#include <barrett/os.h>
#include <barrett/products/product_manager.h>
#include <barrett/systems.h>
#include <barrett/units.h>

#define BARRETT_SMF_VALIDATE_ARGS
#include <barrett/standard_main_function.h>

#include "leader.h"
// #include "background_state_publisher.h"
#include "leader_dynamics.h"

using namespace barrett;
using detail::waitForEnter;

void printUsage(const std::string& programName, const std::string& remoteHost, int recPort, int sendPort) {
    std::cout << "Usage: " << programName << " [remoteHost] [recPort] [sendPort]" << std::endl;
    std::cout << "       Defaults: remoteHost=" << remoteHost << ", recPort=" << recPort << ", sendPort=" << sendPort
              << std::endl;
    std::cout << "       -h or --help: Display this help message." << std::endl;
}

bool validate_args(int argc, char** argv) {

    if ((argc == 2 && (std::string(argv[1]) == "-h" || std::string(argv[1]) == "--help")) || (argc > 4)) {
        printUsage(argv[0], "127.0.0.1", 5555, 5554);
        return 0;
    }

    return true;
}

template <size_t DOF> int wam_main(int argc, char **argv, ProductManager &pm, systems::Wam<DOF> &wam) {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

    jp_type SYNC_POS; // the position each WAM should move to before linking
    if (DOF == 7) {
        SYNC_POS[0] = 0.0;
        SYNC_POS[1] = -1.7;
        SYNC_POS[2] = 0.0;
        SYNC_POS[3] = 2.97;
        SYNC_POS[4] = 0.0;
        SYNC_POS[5] = 0.0;
        SYNC_POS[6] = 0.0;

    } else {
        printf("Error: Only 7 DOF supported\n");
        return false;
    }

    std::string remoteHost = "127.0.0.1";
    int rec_port = 5555;
    int send_port = 5554;

    if (argc >= 2) {
        remoteHost = argv[1];
    }
    if (argc >= 3) {
        rec_port = std::atoi(argv[2]);
    }
    if (argc >= 4) {
        send_port = std::atoi(argv[3]);
    }

    // ros::init(argc, argv, "leader_nowrist");
    // BackgroundStatePublisher<DOF> state_publisher(pm.getExecutionManager(), wam);

    Leader<DOF> leader(pm.getExecutionManager(), argv[1], rec_port, send_port);

    LeaderDynamics<DOF> leaderDynamics;

    ja_type ja;
    ja.setConstant(0.0);
    systems::Constant<ja_type> zeroAcceleration(ja);
    pm.getExecutionManager()->startManaging(zeroAcceleration);

    systems::FirstOrderFilter<jt_type> Filter;
    jt_type omega_p = jt_type::Constant(10);
    Filter.setLowPass(omega_p);
    pm.getExecutionManager()->startManaging(Filter);

    double h_omega_p = 25.0;
    barrett::systems::FirstOrderFilter<jv_type> hp1;
    hp1.setHighPass(jv_type(h_omega_p), jv_type(h_omega_p));
    systems::Gain<jv_type, double, ja_type> jaWAM(1.0);
    pm.getExecutionManager()->startManaging(hp1);

    barrett::systems::FirstOrderFilter<ja_type> jaFilter;
    ja_type l_omega_p = ja_type::Constant(50.0);
    jaFilter.setLowPass(l_omega_p);
    pm.getExecutionManager()->startManaging(jaFilter);


    systems::connect(wam.jvOutput, hp1.input);
    systems::connect(hp1.output, jaWAM.input);
    systems::connect(jaWAM.output, jaFilter.input);
    systems::connect(jaFilter.output, leaderDynamics.jaInputDynamics);

    systems::connect(wam.jvOutput, hp1.input);
    systems::connect(hp1.output, jaWAM.input);
    systems::connect(jaWAM.output, leaderDynamics.jaInputDynamics);

    systems::connect(wam.jpOutput, leaderDynamics.jpInputDynamics);
    systems::connect(wam.jvOutput, leaderDynamics.jvInputDynamics);
    // systems::connect(zeroAcceleration.output, leaderDynamics.jaInputDynamics);

    systems::connect(leaderDynamics.dynamicsFeedFWD, leader.wamDynIn);
    systems::connect(wam.gravity.output, leader.wamGravIn);
    systems::connect(wam.jpOutput, leader.wamJPIn);
    systems::connect(wam.jvOutput, leader.wamJVIn);
    systems::connect(leader.wamJPOutput, Filter.input);

    wam.gravityCompensate();

    std::string line;
    v_type gainTmp;

    bool going = true;

    while (going) {
        printf(">>> ");
        std::getline(std::cin, line);

        switch (line[0]) {
        case 'l':
            if (leader.isLinked()) {
                leader.unlink();
            } else {
                wam.moveTo(SYNC_POS);

                printf("Press [Enter] to link with the other WAM.");
                waitForEnter();
                leader.tryLink();
                wam.trackReferenceSignal(leader.wamJPOutput);

                btsleep(0.1); // wait an execution cycle or two
                if (leader.isLinked()) {
                    printf("Linked with remote WAM.\n");
                } else {
                    printf("WARNING: Linking was unsuccessful.\n");
                }
            }

            break;

        case 't':
            size_t jointIndex;
            {
                size_t jointNumber;
                std::cout << "\tJoint: ";
                std::cin >> jointNumber;
                jointIndex = jointNumber - 1;

                if (jointIndex >= DOF) {
                    std::cout << "\tBad joint number: " << jointNumber;
                    break;
                }
            }

            char gainId;
            std::cout << "\tGain identifier (p, i, or d): ";
            std::cin >> line;
            gainId = line[0];

            std::cout << "\tCurrent value: ";
            switch (gainId) {
            case 'p':
                gainTmp = wam.jpController.getKp();
                break;
            case 'i':
                gainTmp = wam.jpController.getKi();
                break;
            case 'd':
                gainTmp = wam.jpController.getKd();
                break;

            default:
                std::cout << "\tBad gain identifier.";
            }
            std::cout << gainTmp[jointIndex] << std::endl;

            std::cout << "\tNew value: ";
            std::cin >> gainTmp[jointIndex];
            switch (gainId) {
            case 'p':
                wam.jpController.setKp(gainTmp);
                break;
            case 'i':
                wam.jpController.setKi(gainTmp);
                break;
            case 'd':
                wam.jpController.setKd(gainTmp);
                break;

            default:
                std::cout << "\tBad gain identifier.";
            }

            break;
        case 'x':
            going = false;
            break;

        default:
            printf("\n");
            printf("    'l' to toggle linking with other WAM\n");
            printf("    't' to tune control gains\n");
            printf("    'x' to exit\n");

            break;
        }
    }


    pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);

    return 0;
}

