#ifndef TIAGO_UTILS_HPP
#define TIAGO_UTILS_HPP

#include <Eigen/Core>
#include <math.h>

bool box_into_basket(const Eigen::Vector3d& box_translation, const Eigen::Vector3d& basket_translation, const double b_yaw)
{
    Eigen::MatrixXd basket_xy_corners(2, 4);
    basket_xy_corners << basket_translation[0] + 0.14, basket_translation[0] + 0.14, basket_translation[0] - 0.14, basket_translation[0] - 0.14,
        basket_translation[1] - 0.08, basket_translation[1] + 0.08, basket_translation[1] + 0.08, basket_translation[1] - 0.08;

    Eigen::MatrixXd rotation_matrix(2, 2);
    rotation_matrix << std::cos(b_yaw), std::sin(b_yaw),
        -std::sin(b_yaw), std::cos(b_yaw);

    Eigen::MatrixXd rotated_basket_xy_corners(2, 4);
    Eigen::MatrixXd basket_center(2, 4);
    basket_center << basket_translation[0], basket_translation[0], basket_translation[0], basket_translation[0],
        basket_translation[1], basket_translation[1], basket_translation[1], basket_translation[1];
    rotated_basket_xy_corners = rotation_matrix * (basket_xy_corners - basket_center) + basket_center;

    auto d1 = (rotated_basket_xy_corners(0, 1) - rotated_basket_xy_corners(0, 0)) * (box_translation[1] - rotated_basket_xy_corners(1, 0)) - (box_translation[0] - rotated_basket_xy_corners(0, 0)) * (rotated_basket_xy_corners(1, 1) - rotated_basket_xy_corners(1, 0));
    auto d2 = (rotated_basket_xy_corners(0, 2) - rotated_basket_xy_corners(0, 1)) * (box_translation[1] - rotated_basket_xy_corners(1, 1)) - (box_translation[0] - rotated_basket_xy_corners(0, 1)) * (rotated_basket_xy_corners(1, 2) - rotated_basket_xy_corners(1, 1));
    auto d3 = (rotated_basket_xy_corners(0, 3) - rotated_basket_xy_corners(0, 2)) * (box_translation[1] - rotated_basket_xy_corners(1, 2)) - (box_translation[0] - rotated_basket_xy_corners(0, 2)) * (rotated_basket_xy_corners(1, 3) - rotated_basket_xy_corners(1, 2));
    auto d4 = (rotated_basket_xy_corners(0, 0) - rotated_basket_xy_corners(0, 3)) * (box_translation[1] - rotated_basket_xy_corners(1, 3)) - (box_translation[0] - rotated_basket_xy_corners(0, 3)) * (rotated_basket_xy_corners(1, 0) - rotated_basket_xy_corners(1, 3));

    if (d1 > 0.0 && d2 > 0.0 && d3 > 0.0 && d4 > 0.0 && box_translation[2] <= 0.04) {
        return true;
    }
    return false;
}

std::tuple<std::vector<Eigen::Vector2d>, std::vector<Eigen::Vector2d>> create_grid(double box_step_x=0.5, double box_step_y=0.5, double basket_step_x=1., double basket_step_y=1.)
{
    std::vector<Eigen::Vector2d> basket_positions;
    double basket_x_min = -2.;
    double basket_x_max = 2.;
    double basket_y_min = -2.;
    double basket_y_max = 2.;

    int basket_nx_steps = static_cast<int>(floor((basket_x_max-basket_x_min) / basket_step_x));
    int basket_ny_steps = static_cast<int>(floor((basket_y_max-basket_y_min) / basket_step_y));

    for (int x = 0; x <= basket_nx_steps; x++) {
        for (int y = 0; y <= basket_ny_steps; y++) {
            double basket_x = basket_x_min + x * basket_step_x;
            double basket_y = basket_y_min + y * basket_step_y;
            Eigen::Vector2d basket_pt;
            basket_pt << basket_x, basket_y;
            if (basket_pt.norm() < 2.)
                continue;
            basket_positions.push_back(basket_pt);
        }
    }

    std::vector<Eigen::Vector2d> box_positions;
    double box_x_min = -1.;
    double box_x_max = 1.;
    double box_y_min = -1.;
    double box_y_max = 1.;

    int box_nx_steps = static_cast<int>(floor((box_x_max-box_x_min) / box_step_x));
    int box_ny_steps = static_cast<int>(floor((box_y_max-box_y_min) / box_step_y));

    for (int x = 0; x <= box_nx_steps; x++) {
        for (int y = 0; y <= box_ny_steps; y++) {
            double box_x = box_x_min + x * box_step_x;
            double box_y = box_y_min + y * box_step_y;
            Eigen::Vector2d box_pt;
            box_pt << box_x, box_y;
            if (box_pt.norm() < 1.)
                continue;
            box_positions.push_back(box_pt);
        }
    }

    return {basket_positions, box_positions};
}

#endif