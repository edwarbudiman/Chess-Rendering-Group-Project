#include <Eigen/Eigen>
#include <vector>

#include <shader.hpp>
#include <light.hpp>


Eigen::Vector3f blinn_phong_fragment_shader(const fragment_shader_payload &payload) {
    // Material properties
    Eigen::Vector3f ka = payload.material->ka;
    Eigen::Vector3f kd = payload.material->kd; // Using material's diffuse color
    Eigen::Vector3f ks = payload.material->ks;

    // Lighting parameters
    Eigen::Vector3f amb_light_intensity{10, 10, 10}; 
    float p = payload.material->shininessExponant; // Phong exponent (shininess)

    // Inputs for Blinn-Phong model
    auto lights = payload.view_lights;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    // Resulting colour accumulator
    Eigen::Vector3f result_color = {0, 0, 0};

    // Ambient light component
    Eigen::Vector3f La = ka.cwiseProduct(amb_light_intensity);  

    // Compute the effect of each light source
    for (auto &light : lights) {
        // Compute remaining inputs for Blinn-Phong model
        Eigen::Vector3f l = (light.position - point).normalized();
        Eigen::Vector3f v = (-point).normalized();
        Eigen::Vector3f h = (l + v).normalized();
        float r2 = (light.position - point).squaredNorm();

        // Diffused light component
        float diff = std::max(0.0f, normal.dot(l));
        Eigen::Vector3f Ld = kd.cwiseProduct(light.intensity / r2) * diff;

        // Specular light component
        float spec = std::pow(std::max(0.0f, normal.dot(h)), p);
        Eigen::Vector3f Ls = ks.cwiseProduct(light.intensity / r2) * spec;

        // Accumulate the components
        result_color += Ld + Ls;
    }
    // Add the ambient light component
    result_color += La;

    // Convert the colour to [0, 255] range for display
    return result_color * 255.0f;
}


Eigen::Vector3f texture_fragment_shader(const fragment_shader_payload &payload) {
    // Retrieve the texture colour at the current fragment's texture coordinates (given it exists)
    Eigen::Vector3f texture_color = {0, 0, 0};
    if (payload.texture) {
        texture_color = payload.texture->getColor(payload.tex_coords.x(), payload.tex_coords.y());
    }

    // Material properties
    Eigen::Vector3f ka = payload.material->ka;
    Eigen::Vector3f kd = texture_color / 255.0f; // kd is from texture
    Eigen::Vector3f ks = payload.material->ks;

    // Lighting parameters
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    float p = payload.material->shininessExponant;

    // Inputs for Blinn-Phong model
    std::vector<Light> lights = payload.view_lights;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal.normalized();

    // Resulting colour accumulator
    Eigen::Vector3f result_color = {0, 0, 0};

    // Ambient light component
    Eigen::Vector3f La = ka.cwiseProduct(amb_light_intensity);

    // Compute the effect of each light source
    for (auto &light : lights) {
        // Compute remaining inputs for Blinn-Phong model
        Eigen::Vector3f l = (light.position - point).normalized();
        Eigen::Vector3f v = (-point).normalized();
        Eigen::Vector3f h = (l + v).normalized();
        float r2 = (light.position - point).squaredNorm();

        // Diffused light component
        float diff = std::max(0.0f, normal.dot(l));
        Eigen::Vector3f Ld = kd.cwiseProduct(light.intensity / r2) * diff;

        // Specular light component
        float spec = std::pow(std::max(0.0f, normal.dot(h)), p);
        Eigen::Vector3f Ls = ks.cwiseProduct(light.intensity / r2) * spec;

        // Accumulate the components
        result_color += Ld + Ls;
    }
    // Add the ambient light component
    result_color += La;

    // Convert the colour to [0, 255] range for display
    return result_color * 255.0f;
}
