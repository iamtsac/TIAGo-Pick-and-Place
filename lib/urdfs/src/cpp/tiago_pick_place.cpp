#include <robot_dart/robot_dart_simu.hpp>
#include <robot_dart/robots/tiago.hpp>

#ifdef GRAPHIC
#include <robot_dart/gui/magnum/graphics.hpp>
#endif

#include <dart/dynamics/BodyNode.hpp>

#include <cpp/utils.hpp>

#include <random>

int main()
{
    double dt = 0.001;
    double simulation_time = 20.;
    int total_steps = static_cast<int>(std::ceil(simulation_time / dt));

    // Create robot
    auto robot = std::make_shared<robot_dart::robots::Tiago>(1. / dt);
    std::vector<std::string> arm_dofs = {"arm_1_joint",
        "arm_2_joint",
        "arm_3_joint",
        "arm_4_joint",
        "arm_5_joint",
        "arm_6_joint",
        "arm_7_joint",
        "gripper_finger_joint",
        "gripper_right_finger_joint"};

    // set initial joint positions
    {
        Eigen::VectorXd positions = robot->positions(arm_dofs);
        positions[0] = M_PI / 2.;
        positions[1] = M_PI / 4.;
        positions[2] = 0.;
        positions[3] = M_PI / 2.;
        positions[4] = 0.;
        positions[5] = 0.;
        positions[6] = M_PI / 2.;
        positions[7] = 0.03;
        positions[8] = 0.03;

        robot->set_positions(positions, arm_dofs);
    }

    // Control base - we make the base fully controllable
    robot->set_actuator_type("servo", "rootJoint", false, true, false);
    // robot->set_commands(robot_dart::make_vector({0.1, 0.1, 0.}), {"rootJoint_rot_z", "rootJoint_pos_x", "rootJoint_pos_y"});

    // Create position grid for the box/basket
    auto [basket_positions, box_positions] = create_grid();

    std::random_device rd; // only used once to initialise (seed) engine
    std::mt19937 rng(rd()); // random-number engine used (Mersenne-Twister in this case)
    std::uniform_int_distribution<int> basket_dist(0, basket_positions.size() - 1);
    std::uniform_int_distribution<int> box_dist(0, box_positions.size() - 1);

    // Create box
    Eigen::Vector3d box_size(3);
    box_size << 0.04, 0.04, 0.04;
    Eigen::Vector6d box_pose(6);
    // Random box position
    int box_pos_index = box_dist(rng);
    box_pose << 0., 0., 0., box_positions[box_pos_index](0), box_positions[box_pos_index](1), box_size(2) / 2.;
    auto box = robot_dart::Robot::create_box(box_size, box_pose, "free", 0.1, dart::Color::Red(1.0), "box_" + std::to_string(0));

    // Create basket
    Eigen::Vector6d basket_pose(6);
    // Random basket position
    int basket_pos_index = basket_dist(rng);
    double basket_z_angle = 0.;
    basket_pose << 0., 0., basket_z_angle, basket_positions[basket_pos_index](0), basket_positions[basket_pos_index](1), 0.0008;
    std::vector<std::pair<std::string, std::string>> basket_packages = {{"basket", "models/basket"}};
    auto basket = std::make_shared<robot_dart::Robot>("models/basket/basket.urdf", basket_packages, "basket");
    basket->set_positions(basket_pose);
    basket->fix_to_world();

    // Create graphics
#ifdef GRAPHIC
    robot_dart::gui::magnum::GraphicsConfiguration configuration;
    configuration.width = 1280;
    configuration.height = 960;
    auto graphics = std::make_shared<robot_dart::gui::magnum::Graphics>(configuration);
#endif

    // Create simulator object
    robot_dart::RobotDARTSimu simu(dt);
    simu.set_collision_detector("bullet");
    simu.set_control_freq(100);
#ifdef GRAPHIC
    simu.set_graphics(graphics);
    graphics->look_at({0., 4.5, 2.5}, {0., 0., 0.25});
#endif
    simu.add_checkerboard_floor();
    simu.add_robot(robot);
    simu.add_robot(box);
    simu.add_robot(basket);

    int finish_counter = 0;

    for (int i = 0; i < total_steps; i++) {
        if (simu.schedule(simu.control_freq())) {
            // Do something

            // Get box/basket state
            Eigen::Isometry3d box_tf = box->base_pose();
            Eigen::Isometry3d basket_tf = basket->base_pose();

            Eigen::Vector3d box_translation = box_tf.translation();
            Eigen::Vector3d basket_translation = basket_tf.translation();

            if (box_into_basket(box_translation, basket_translation, basket_z_angle)) {
                finish_counter++;
            }

            if (finish_counter >= 10)
                break;
        }

        if (simu.step_world())
            break;
    }

    robot.reset();
    return 0;
}