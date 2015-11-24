//| This file is a part of the ERC ResiBots project.
//| Copyright 2015, ISIR / Universite Pierre et Marie Curie (UPMC)
//| Main contributor(s): Jean-Baptiste Mouret, mouret@isir.fr
//|                      Antoine Cully, cully@isir.upmc.fr
//|
//| This software is governed by the CeCILL license under French law
//| and abiding by the rules of distribution of free software.  You
//| can use, modify and/ or redistribute the software under the terms
//| of the CeCILL license as circulated by CEA, CNRS and INRIA at the
//| following URL "http://www.cecill.info".
//|
//| As a counterpart to the access to the source code and rights to
//| copy, modify and redistribute granted by the license, users are
//| provided only with a limited warranty and the software's author,
//| the holder of the economic rights, and the successive licensors
//| have only limited liability.
//|
//| In this respect, the user's attention is drawn to the risks
//| associated with loading, using, modifying and/or developing or
//| reproducing the software by the user in light of its specific
//| status of free software, that may mean that it is complicated to
//| manipulate, and that also therefore means that it is reserved for
//| developers and experienced professionals having in-depth computer
//| knowledge. Users are therefore encouraged to load and test the
//| software's suitability as regards their requirements in conditions
//| enabling the security of their systems and/or data to be ensured
//| and, more generally, to use and operate it in the same conditions
//| as regards security.
//|
//| The fact that you are presently reading this means that you have
//| had knowledge of the CeCILL license and that you accept its terms.

//#define SHOW_TIMER
#include <limbo/limbo.hpp>
#include <limbo/inner_cmaes.hpp>
#include "exhaustiveSearchMap.hpp"
#include "meanMap.hpp"
#include "statTransferts.hpp"

#ifdef GRAPHIC
//#define NO_PARALLEL
#include "renderer/osg_visitor.hh"
#endif

#include "hexapod.hh"
#include "simu.hpp"

#define NO_PARALLEL
#include "limbo/parallel.hpp"

#ifdef ROBOT
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <hexa_control/Transfert.h>
#endif

using namespace limbo;

struct Params {
    struct boptimizer {
        BO_PARAM(double, noise, 0.001);
        BO_PARAM(int, dump_period, 1);
    };
    struct maxiterations {
        BO_DYN_PARAM(int, n_iterations);
    };
    struct maxpredictedvalue {
        BO_PARAM(float, ratio, 0.9);
    };

    struct kf_maternfivehalfs {
        BO_PARAM(float, sigma, 1);
        BO_DYN_PARAM(float, l);
    };
    struct ucb {
        BO_DYN_PARAM(float, alpha);
    };

    struct archiveparams {

        struct elem_archive {
            std::vector<float> duty_cycle;
            float fit;
            std::vector<float> controller;
        };

        struct classcomp {
            bool operator()(const std::vector<float>& lhs, const std::vector<float>& rhs) const
            {
                assert(lhs.size() == 6 && rhs.size() == 6);
                int i = 0;
                while (i < 5 && round(lhs[i] * 4) == round(rhs[i] * 4)) //lhs[i]==rhs[i])
                    i++;
                return round(lhs[i] * 4) < round(rhs[i] * 4); //lhs[i]<rhs[i];
            }
        };
        typedef std::map<std::vector<float>, elem_archive, classcomp> archive_t;
        static std::map<std::vector<float>, elem_archive, classcomp> archive;
    };
};

Params::archiveparams::archive_t load_archive(std::string archive_name);
Params::archiveparams::archive_t create_random_map(int size);

namespace global {

    struct timeval timev_selection; // Initial absolute time (static)
    std::string res_dir;
#ifdef ROBOT
    boost::shared_ptr<ros::NodeHandle> node;

    ros::ServiceClient hexapod;

    // crazy stuff
    std::vector<int> brokenLegs;
    boost::shared_ptr<ode::Environment_hexa> global_env;
    boost::shared_ptr<robot::Hexapod> global_robot;
#else
    std::vector<int> brokenLegs;
    boost::shared_ptr<robot::Hexapod> global_robot;
    boost::shared_ptr<ode::Environment_hexa> global_env;
#endif
};
///---------------------------

#ifdef ROBOT
void init_ros_node(int argc, char** argv)
{
    ros::init(argc, argv, "hexap_bomean", ros::init_options::NoSigintHandler);
    global::node = boost::shared_ptr<ros::NodeHandle>(new ros::NodeHandle());
    global::hexapod = global::node->serviceClient<hexa_control::Transfert>("Transfert");
}
#endif
//hexa_control::Transfert srv;

void init_simu(int argc, char** argv, bool master, std::vector<int> broken_legs = std::vector<int>())
{

    global::global_env = boost::shared_ptr<ode::Environment_hexa>(new ode::Environment_hexa(0));

    if (broken_legs.size() > 0)
        global::brokenLegs = broken_legs;

    if (global::brokenLegs.size() == 0)
        std::cout << "global_real_robot is undamaged" << std::endl;
    else {
        std::cout << "legs ";
        for (size_t i = 0; i < global::brokenLegs.size(); i++)
            std::cout << global::brokenLegs[i] << " ";
        std::cout << " are removed from global_real_robot" << std::endl;
    }

    global::global_robot = boost::shared_ptr<robot::Hexapod>(new robot::Hexapod(*global::global_env, Eigen::Vector3d(0, 0, 0.5), global::brokenLegs));

    float step = 0.001;

    // low gravity to slow things down (eq. smaller timestep?)
    global::global_env->set_gravity(0, 0, -9.81);
    bool stabilized = false;
    int stab = 0;
    for (size_t s = 0; s < 1000 && !stabilized; ++s) {

        Eigen::Vector3d prev_pos = global::global_robot->pos();
        global::global_robot->next_step(step);
        global::global_env->next_step(step);

        if ((global::global_robot->pos() - prev_pos).norm() < 1e-4)
            stab++;
        else
            stab = 0;
        if (stab > 100)
            stabilized = true;
    }

    global::global_env->set_gravity(0, 0, -9.81);
}

template <typename Params>
struct fit_eval_map {

    BOOST_STATIC_CONSTEXPR int dim = 6;
    fit_eval_map()
    {
        timerclear(&global::timev_selection);
        gettimeofday(&global::timev_selection, NULL);
    }

    float operator()(Eigen::VectorXd x) const
    {

        std::cout << "start eval" << std::endl;
        std::vector<float> key(x.size(), 0);
        for (int i = 0; i < x.size(); i++)
            key[i] = x[i];
        if (Params::archiveparams::archive.count(key) == 0)
            return -1000;

#ifdef ROBOT
        std::vector<float> ctrl = Params::archiveparams::archive.at(key).controller;
        struct timeval timev_init; // Initial absolute time (static)

        struct timeval timev_cur; // Current absolute
        struct timeval timev_duration; // Current tick position (curent - previous)
        timerclear(&timev_cur);
        gettimeofday(&timev_cur, NULL);
        timersub(&timev_cur, &global::timev_selection, &timev_duration);

        std::ofstream ofile((global::res_dir + "/times.dat").c_str(), std::ios_base::app);
        std::cout << "selection " << timev_duration.tv_sec + timev_duration.tv_usec / 1e6 << "sec" << std::endl;
        ofile << "selection " << timev_duration.tv_sec + timev_duration.tv_usec / 1e6 << "sec" << std::endl;

        ///- COMMUNICATION WITH ROS/HEXAPOD
        hexa_control::Transfert srv;

        srv.request.duration = -1; // pose zero
        if (global::hexapod.call(srv)) {
            ROS_INFO("init pose");
        }
        else {
            ROS_ERROR("Failed to call service");
            return 1;
        }

        std::cout << __FILE__ << "  " << __LINE__ << std::endl;
        for (int i = 0; i < ctrl.size(); i++)
            srv.request.params[i] = ctrl[i];
        std::cout << __FILE__ << "  " << __LINE__ << std::endl;

        float obs = 0;
        bool ok = false;
        std::cout << __FILE__ << "  " << __LINE__ << std::endl;

        do {
            timerclear(&timev_init);
            gettimeofday(&timev_init, NULL);
            std::cout << __FILE__ << "  " << __LINE__ << std::endl;

            srv.request.duration = 5; // action
            if (global::hexapod.call(srv)) {
                ROS_INFO("controller executed");
                obs = srv.response.covered_distance;
            }
            else {
                ROS_ERROR("Failed to call service");
                //return 1;
            }
            srv.request.duration = -1; // pose zero
            if (global::hexapod.call(srv)) {
                ROS_INFO("init pose");
            }
            else {
                ROS_ERROR("Failed to call service");
                return 1;
            }
            timerclear(&timev_cur);
            gettimeofday(&timev_cur, NULL);
            timersub(&timev_cur, &timev_init, &timev_duration);

            std::cout << "estimated distance: " << obs << std::endl;
            if (obs < 0 || obs > 2) {
                std::cout << "measurement seems wrong, set to zero" << std::endl;
                obs = 0;
            }

            std::cin.clear();
            std::cout << "transfert ok? :" << std::endl;
            std::cin >> ok;
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        } while (!ok);

        std::cout << "action " << timev_duration.tv_sec + timev_duration.tv_usec / 1e6 << "sec" << std::endl;
        ofile << "action " << timev_duration.tv_sec + timev_duration.tv_usec / 1e6 << "sec" << std::endl;

        gettimeofday(&global::timev_selection, NULL);
        return obs;

#else
        Simu simu = Simu(Params::archiveparams::archive.at(key).controller, global::global_robot, global::brokenLegs, false, 5, 1, global::global_env->angle);

        if (simu.covered_distance() < 0 || simu.covered_distance() > 2.5) {

            std::cout << simu.covered_distance() << " measurement seems wrong, set to zero" << std::endl;
            return 0;
        }

        return simu.covered_distance() * limbo::misc::gaussian_rand(0.95, 0.1);
#endif
    }
};

std::map<std::vector<float>, Params::archiveparams::elem_archive, Params::archiveparams::classcomp> load_archive(std::string archive_name)
{

    std::map<std::vector<float>, Params::archiveparams::elem_archive, Params::archiveparams::classcomp> archive;

    std::ifstream monFlux(archive_name.c_str()); //Ouverture d'un fichier en lecture
    if (monFlux) {
        while (!monFlux.eof()) {
            Params::archiveparams::elem_archive elem;
            std::vector<float> candidate(6);
            for (int i = 0; i < 43; i++) {
                if (monFlux.eof())
                    break;
                float data;
                monFlux >> data;
                if (i <= 5) {
                    candidate[i] = data;
                    elem.duty_cycle.push_back(data);
                }
                if (i == 6) {

                    elem.fit = data;
                }
                if (i >= 7)
                    elem.controller.push_back(data);
            }
            if (elem.controller.size() == 36) {

                archive[candidate] = elem;
            }
        }
    }
    else {
        std::cout << "ERREUR: Impossible d'ouvrir le fichier en lecture." << std::endl;
        return archive;
    }
    std::cout << archive.size() << " elements loaded" << std::endl;
    return archive;
}

void lecture(int argc, char** argv)
{
#ifdef ROBOT
    std::cout << "reading params from command line" << std::endl;
    std::cout << "Type any key whem ready" << std::endl;
    int x;
    std::cin >> x;
    //init_simu(argc, argv,false);
    hexa_control::Transfert srv;

    srv.request.duration = -1; // pose zero
    if (global::hexapod.call(srv)) {
        ROS_INFO("init pose");
    }
    else {
        ROS_ERROR("Failed to call service");
        return;
    }

    std::cout << "LOADING..." << std::endl;
    for (int i = 0; i < 36; i++)
        srv.request.params[i] = atof(argv[i + 1]);
    srv.request.duration = 5; // pose zero
    if (global::hexapod.call(srv)) {
        ROS_INFO("executed");
        std::cout << "covered distance! " << srv.response.covered_distance << std::endl;
    }
    else {
        ROS_ERROR("Failed to call service");
        return;
    }

    srv.request.duration = -5; // relax
    if (global::hexapod.call(srv)) {
        ROS_INFO("relax");
    }
    else {
        ROS_ERROR("Failed to call service");
        return;
    }

#else
    std::vector<float> ctrl;
    for (int i = 0; i < 36; i++)
        ctrl.push_back(atof(argv[i + 1]));
    Simu simu = Simu(ctrl, global::global_robot, global::brokenLegs);
    std::cout << "covered distance! " << simu.covered_distance() << std::endl;
#endif

    return;
}

Params::archiveparams::archive_t Params::archiveparams::archive;
BO_DECLARE_DYN_PARAM(float, Params::kf_maternfivehalfs, l);
BO_DECLARE_DYN_PARAM(int, Params::maxiterations, n_iterations);
BO_DECLARE_DYN_PARAM(float, Params::ucb, alpha);

int main(int argc, char** argv)
{

    if (argc < 2) {
        std::cout << "please provide a map" << std::endl;
        return -1;
    }
    Params::archiveparams::archive = load_archive(argv[1]);

    if (argc > 2)
        Params::kf_maternfivehalfs::set_l(atof(argv[2]));
    else
        Params::kf_maternfivehalfs::set_l(0.4); //0.4 (antoine value)

    Params::ucb::set_alpha(0.05);
    Params::maxiterations::set_n_iterations(20);
    srand(time(NULL));

    typedef kernel_functions::MaternFiveHalfs<Params> Kernel_t;
    typedef inner_optimization::ExhaustiveSearchArchive<Params> InnerOpt_t;
    typedef boost::fusion::vector<stopping_criterion::MaxIterations<Params>, stopping_criterion::MaxPredictedValue<Params>> Stop_t;
    typedef mean_functions::MeanArchive_Map<Params> Mean_t;
    typedef boost::fusion::vector<stat::Acquisitions<Params>, stat::StatTransferts<Params>> Stat_t;

    typedef init_functions::NoInit<Params> Init_t;
    typedef model::GP<Params, Kernel_t, Mean_t> GP_t;
    typedef acquisition_functions::UCB<Params, GP_t> Acqui_t;

#ifdef ROBOT
    init_ros_node(argc, argv);
    hexa_control::Transfert srv;
    srv.request.duration = 0; // init
    if (global::hexapod.call(srv)) {
        ROS_INFO("executed");
    }
    else {
        ROS_ERROR("Failed to call service");
        return 1;
    }

#else
    dInitODE();
    std::vector<int> brokenleg;
    if (argc > 3) {
        for (int i = 3; i < argc; i++) {
            brokenleg.push_back(atoi(argv[i]));
        }
    }
    init_simu(argc, argv, true, brokenleg);
#endif

    BOptimizer<Params, model_fun<GP_t>, init_fun<Init_t>, acq_fun<Acqui_t>, inneropt_fun<InnerOpt_t>, stat_fun<Stat_t>, stop_fun<Stop_t>> opt;
    global::res_dir = opt.res_dir();

    Eigen::VectorXd result(1);

    opt.optimize(fit_eval_map<Params>());
    float val = opt.best_observation();
    result = opt.best_sample().transpose();

    std::ofstream oofile((opt.res_dir() + std::string("/panne.dat")).c_str(), std::ios_base::app);
    for (size_t i = 0; i < global::brokenLegs.size(); i++)
        oofile << global::brokenLegs[i] << " ";

#ifdef ROBOT
    std::ofstream ofile((std::string("result_") + argv[2] + ".dat").c_str(), std::ios_base::app);

    ofile << argv[1] << " " << argv[2] << "  ";
    for (int i = 3; i < argc; i++)
        ofile << argv[i];
    ofile << " " << val << " " << opt.iteration() << std::endl;
#endif

    std::cout << val << " res  " << result.transpose() << std::endl;

#ifdef ROBOT
    srv.request.duration = -5; // relax
    if (global::hexapod.call(srv)) {
        ROS_INFO("executed");
    }
    else {
        ROS_ERROR("Failed to call service");
        return 1;
    }

#else

    global::global_robot.reset();
    global::global_env.reset();
    dCloseODE();
#endif

    std::cout << "fin" << std::endl;

    return 0;
}
