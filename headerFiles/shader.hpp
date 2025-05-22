#pragma once

#include <Eigen/Eigen>
#include <vector>
#include "texture.hpp"
#include "light.hpp"
#include <Eigen/Core>
#include "object.hpp" // Added for Material

// Fragment shader payload structure from CLab2
struct fragment_shader_payload
{
    fragment_shader_payload(const Eigen::Vector3f& col, const Eigen::Vector3f& nor, const Eigen::Vector2f& tc, const std::vector<Light> vl, Texture* tex, const Material* mat) :
        colour(col), normal(nor), tex_coords(tc), view_lights(vl), texture(tex), material(mat) {}

    Eigen::Vector3f view_pos;
    Eigen::Vector3f colour;
    Eigen::Vector3f normal;
    Eigen::Vector2f tex_coords;
    std::vector<Light> view_lights;
    Texture* texture;
};

Eigen::Vector3f blinn_phong_fragment_shader(const fragment_shader_payload &payload);
Eigen::Vector3f texture_fragment_shader(const fragment_shader_payload &payload);