#pragma once

#include <Eigen/Eigen>
#include <algorithm>
#include <optional>
#include <vector>

#include <object.hpp>
#include <shader.hpp>
#include <texture.hpp>
#include <scene.hpp>

using namespace Eigen;

namespace rst {
enum class Buffers { Colour = 1, Depth = 2 };

inline Buffers operator|(Buffers a, Buffers b) {
    return Buffers((int)a | (int)b);
}

inline Buffers operator&(Buffers a, Buffers b) {
    return Buffers((int)a & (int)b);
}

class Rasterizer {
public:
    Rasterizer(int w, int h);

    void setView(const Eigen::Matrix4f& v);
    void setProjection(const Eigen::Matrix4f& p);

    void set_texture(Texture tex) {
        texture = tex;
    }

    void setFragmentShader(std::function<Eigen::Vector3f(fragment_shader_payload)> frag_shader);

    void rasterizeObjects(Scene scene);

    void draw(std::vector<std::shared_ptr<Face>> &faces, std::vector<Light> lights, const Material& currentMaterial);

    void setPixel(const Vector2i& point, const Eigen::Vector3f& color);

    void clear(Buffers buff);

    std::vector<Eigen::Vector3f>& frame_buffer() {
        return frameBuffer;
    }
    std::vector<float>& depth_buffer() {
        return depthBuffer;
    }

private:
    int width, height;

    Eigen::Matrix4f view;
    Eigen::Matrix4f projection;

    std::optional<Texture> texture;

    std::function<Eigen::Vector3f(fragment_shader_payload)> fragment_shader;

    std::vector<Eigen::Vector3f> frameBuffer;
    std::vector<Eigen::Vector3f> ssaaFrameBuffer;
    std::vector<float> depthBuffer;
    std::vector<float> ssaaDepthBuffer;

    void rasterizeTriangle(std::vector<Vertex> &vertices, std::vector<Eigen::Vector3f> &view_pos, std::vector<Light> &view_lights, const Material& currentMaterial);
    void postProcessBuffer();

    int getIndex(int x, int y);
};
};