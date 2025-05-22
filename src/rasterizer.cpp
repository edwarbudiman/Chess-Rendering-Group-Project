#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include "Eigen/Dense"

#include <headerFiles/rasterizer.hpp>
#include <headerFiles/scene.hpp>
#include <headerFiles/light.hpp>
#include <headerFiles/object.hpp>
#include <headerFiles/camera.hpp>

using namespace std;
using namespace Eigen;

rst::Rasterizer::Rasterizer(int w, int h) : width(w), height(h) {
    frameBuffer.resize(w * h);
    depthBuffer.resize(w * h);
    ssaaFrameBuffer.resize(4 * w * h);
    ssaaDepthBuffer.resize(4 * w * h);
    texture = std::nullopt;
}

void rst::Rasterizer::setView(const Eigen::Matrix4f& v) {
    view = v;
}

void rst::Rasterizer::setProjection(const Eigen::Matrix4f& p) {
    projection = p;
}

void rst::Rasterizer::setFragmentShader(std::function<Eigen::Vector3f(fragment_shader_payload)> frag_shader) {
    fragment_shader = frag_shader;
}

// Clear the frame buffer and/or depth buffer
void rst::Rasterizer::clear(Buffers buff) {
    if ((buff & Buffers::Colour) == Buffers::Colour) {
        std::fill(frameBuffer.begin(), frameBuffer.end(), Eigen::Vector3f(0, 0, 0));
        std::fill(ssaaFrameBuffer.begin(), ssaaFrameBuffer.end(), Eigen::Vector3f(0, 0, 0));
    }
    if ((buff & Buffers::Depth) == Buffers::Depth) {
        std::fill(depthBuffer.begin(), depthBuffer.end(), std::numeric_limits<float>::infinity());
        std::fill(ssaaDepthBuffer.begin(), ssaaDepthBuffer.end(), std::numeric_limits<float>::infinity());
    }
}

// Convert a 3D vector to a 4D vector
auto to_vec4(const Eigen::Vector3f &v3, float w = 1.0f) {
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


// Rasterize the objects in the scene
void rst::Rasterizer::rasterizeObjects(Scene scene){
    int count = 1;
    for (auto& object : scene.objects) {
        cout << "\trasterizing object " << count++ << "/" << scene.objects.size() << "\n";
        if (object.material.diffuseTextureFile != "") {
            set_texture(Texture(object.material.diffuseTextureFile));
        }
        auto faces = object.faces;
        draw(faces, scene.lights, object.material); // Pass object's material
    }
}

void rst::Rasterizer::draw(std::vector<std::shared_ptr<Face>> &faces, std::vector<Light> lights, const Material& currentMaterial) { // Add currentMaterial
    // Needed to manually map z screen positions to [nearPlane, farPlane] for current test camera
    float f1 = -(50 - 0.1) / 2.0f;
    float f2 = -(50 + 0.1) / 2.0f;

    // Convert lights to view space
    std::vector<Light> viewspace_lights;
    for (auto &light : lights) {
        Eigen::Vector4f lightPos = view * to_vec4(light.position);
        viewspace_lights.push_back(Light(lightPos.head<3>(), light.intensity));
    }

    Eigen::Matrix4f model = Eigen::Matrix4f::Identity(); // Assuming face is already in world space
    Eigen::Matrix4f mvp = projection * view * model;
    // Extract vertices from the face
    int count = 1;
    for (auto &face : faces) {
        cout << "\r\trasterizing face " << count++ << "/" << faces.size();
        auto vertices = face->getVertices();
        if (vertices.size() != 3) {
            std::cerr << "Error: Face is not a triangle!" << std::endl;
            return;
        }

        // Convert vertices to viewspace and screen space
        std::vector<Eigen::Vector3f> viewspace_vertices;
        std::vector<Vertex> screenspace_vertices;
        for (int i = 0; i < 3; i++) {
            // Convert to view space
            Eigen::Vector4f view_pos = view * to_vec4(vertices[i]->position);
            viewspace_vertices.push_back(view_pos.head(3));

            // Convert to screen space
            Eigen::Vector4f screen_pos = mvp * to_vec4(vertices[i]->position);

            screen_pos /= screen_pos.w(); // Perspective divide

            // Map NDC to screen space
            screen_pos.x() = 0.5 * width * (screen_pos.x() + 1.0);
            screen_pos.y() = 0.5 * height * (screen_pos.y() + 1.0);
            screen_pos.z() = (screen_pos.z() * f1) + f2;

            // Create a new vertex with the screen position and original attributes
            Vertex newVertex = *vertices[i];
            newVertex.position = screen_pos.head<3>();
            screenspace_vertices.push_back(newVertex);

        }
        //back face culling
        Vector3f triNorm = (screenspace_vertices[1].position - screenspace_vertices[0].position).cross(screenspace_vertices[2].position - screenspace_vertices[0].position);
        if(triNorm.z() < 0){
            rasterizeTriangle(screenspace_vertices, viewspace_vertices, viewspace_lights, currentMaterial); // Pass currentMaterial
        }
    }
    cout << "\n";
    // Perform post-processing to average the SSAA samples
    postProcessBuffer();
}

// Check if a point (x, y) is inside a triangle defined by three vertices
static bool insideTriangle(float x, float y, const Vector4f *_v) {
    Vector3f v[3];
    for (int i = 0; i < 3; i++)
        v[i] = {_v[i].x(), _v[i].y(), 1.0};
    Vector3f p(x, y, 1.);
    Vector3f f0, f1, f2;
    f0 = (p - v[0]).cross(v[1] - v[0]);
    f1 = (p - v[1]).cross(v[2] - v[1]);
    f2 = (p - v[2]).cross(v[0] - v[2]);
    if (f0.dot(f1) > 0 && f1.dot(f2) > 0)
        return true;
    return false;
}

// Compute barycentric coordinates for a point (x, y) inside a triangle defined by three vertices
static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector4f *v) {
    float c1 = (x * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * y + v[1].x() * v[2].y() - v[2].x() * v[1].y()) /
               (v[0].x() * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * v[0].y() + v[1].x() * v[2].y() -
                v[2].x() * v[1].y());
    float c2 = (x * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * y + v[2].x() * v[0].y() - v[0].x() * v[2].y()) /
               (v[1].x() * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * v[1].y() + v[2].x() * v[0].y() -
                v[0].x() * v[2].y());
    float c3 = (x * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * y + v[0].x() * v[1].y() - v[1].x() * v[0].y()) /
               (v[2].x() * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * v[2].y() + v[0].x() * v[1].y() -
                v[1].x() * v[0].y());
    return {c1, c2, c3};
}

// Convert a 2D screen coordinate (x, y) to a 1D index in the frame buffer
int rst::Rasterizer::getIndex(int x, int y) {
    return (height - y - 1) * width + x;
}

// Post-process the SSAA buffer to average the samples and store them in the main frame buffer
void rst::Rasterizer::postProcessBuffer() {
    for (int x = 0; x < width; x++) {
        for (int y = 0; y < height; y++) {
            int index = getIndex(x, y);
            for (int i = 0; i < 4; i++) {
                frameBuffer[index] += ssaaFrameBuffer[4 * index + i];
            }
            frameBuffer[index] = (frameBuffer[index] / 4) * 255;
        }
    }
}

// Interpolates a 3D vector (e.g., color, normal) using barycentric coordinates
static Eigen::Vector3f interpolate(float alpha, float beta, float gamma, const Eigen::Vector3f &vert1,
    const Eigen::Vector3f &vert2, const Eigen::Vector3f &vert3, float weight) {
    return (alpha * vert1 + beta * vert2 + gamma * vert3) / weight;
}

void rst::Rasterizer::rasterizeTriangle(std::vector<Vertex> &vertices, std::vector<Eigen::Vector3f> &view_pos, std::vector<Light> &view_lights, const Material& currentMaterial) { // Add currentMaterial
    // Convert vertices to 4D homogeneous coordinates
    Eigen::Vector4f v[3];
    for (int i = 0; i < 3; i++) {
        v[i] = to_vec4(vertices[i].position);
    }

    // Compute bounding box
    float minX = std::max(0.0f, std::floor(std::min({v[0].x(), v[1].x(), v[2].x()})));
    float maxX = std::min((float)(width - 1), std::ceil(std::max({v[0].x(), v[1].x(), v[2].x()})));
    float minY = std::max(0.0f, std::floor(std::min({v[0].y(), v[1].y(), v[2].y()})));
    float maxY = std::min((float)(height - 1), std::ceil(std::max({v[0].y(), v[1].y(), v[2].y()})));

    //Screen‐space rejection
    if(maxX < 0 || minX > width || maxY < 0 || minY > height){
        return;
    }

    // Subpixel sampling offsets for 2x2 SSAA
    Eigen::Vector2f ssaa_offset[4] = {
        Eigen::Vector2f(0.25, 0.25), // Top-left
        Eigen::Vector2f(0.75, 0.25), // Top-right
        Eigen::Vector2f(0.25, 0.75), // Bottom-left
        Eigen::Vector2f(0.75, 0.75)  // Bottom-right
    };

    // Compute to colours of the triangle's vertices
    Eigen::Vector3f vertex_colours[3];
    for (int j = 0; j < 3; j++) {
        // Assuming the material is uniform across the face, which is typical
        // If individual vertices could have different materials, this would need adjustment
        // and the Material would likely need to be part of the Vertex struct.
        // For now, we'll assume the first vertex's parent face's material applies to all.
        // This requires that Vertex has a way to reference its parent Face or Object's material.
        // Let's assume `vertices[j].material` exists and is of type `const Material*`.
        // If not, we'd need to trace back to the Object or Face that owns these vertices.
        // Given the current structure, it's more likely the material is associated with the `Face` or `Object`.
        // Let's assume `face->material` is accessible here, or `object.material` if we pass `object` down.
        // The prompt implies object.material, so we'll need to ensure it's available.
        // The `draw` function takes `faces`, and `rasterizeObjects` iterates `scene.objects`.
        // We need to pass the material from the object to `rasterizeTriangle` or access it globally.

        // Simplification: Assuming `texture` optional field in `Rasterizer` is a stand-in and
        // the actual material should come from the object.
        // The current `fragment_shader_payload` takes `Texture*`.
        // The `draw` function iterates through `faces`. Each `Face` should belong to an `Object` which has a `Material`.
        // Let's assume `face->material` is available (this needs to be added to Face struct if not).
        // For now, I'll use a placeholder for material, as the structure doesn't directly provide it here.
        // This highlights a potential need to refactor how material is accessed.
        // However, the task implies object.material. The loop in rasterizeObjects has `object.material`.
        // We need to pass this down.
        // The current structure of rasterizeTriangle receives `std::vector<Vertex> &vertices`.
        // It doesn't directly receive the `Object` or its `Material`.
        // This is a problem. Let's assume for now that `vertices[j]` can provide its material.
        // This would mean `Vertex` needs a `const Material* material;` field.
        // This is a significant change to `Vertex` struct.

        // Revisiting the `rasterizeObjects` loop:
        // `for (auto& object : scene.objects)`
        // `auto faces = object.faces;`
        // `draw(faces, scene.lights);`
        // `draw` calls `rasterizeTriangle`.
        // We need to pass `object.material` from `rasterizeObjects` through `draw` to `rasterizeTriangle`.

        // Let's modify `draw` and `rasterizeTriangle` signatures first. This is outside the scope of just this file.
        // For now, I will proceed with the assumption that `vertices[j].material` is available.
        // This is a temporary assumption to make progress on the current file, but it needs to be addressed.
        // A better way would be to pass `const Material& currentMaterial` to `rasterizeTriangle`.

        // Given the constraints, I must use the existing structure as much as possible.
        // The `fragment_shader_payload` constructor was updated to take `const Material* mat`.
        // In `rasterizeObjects`, `object.material` is available.
        // `set_texture` uses `object.material.diffuseTextureFile`.
        // The `texture` member of `Rasterizer` seems to be how the texture is currently passed.
        // This is not ideal for full material properties.

        // Let's look at where `rasterizeTriangle` is called:
        // `rasterizeTriangle(screenspace_vertices, viewspace_vertices, viewspace_lights);`
        // The `screenspace_vertices` are of type `Vertex`.
        // The `Face` is available one level up in `draw`. `face->material` would be ideal.
        // Let's assume `face->material` is the source.

        // The `fragment_shader_payload` is created inside `rasterizeTriangle`.
        // To get `face->material` here, `rasterizeTriangle` needs the current `Face` or its `Material`.
        // Modifying `rasterizeTriangle` signature:
        // void rst::Rasterizer::rasterizeTriangle(std::vector<Vertex> &vertices, ..., const Material& material)
        // And in `draw` when calling it:
        // `rasterizeTriangle(screenspace_vertices, ..., face->material);`
        // This requires `Face` to have a `material` member. Let's assume `object.hpp`'s `Face` struct will be updated.

        // For this step, I will modify the payload creation assuming `currentMaterial` is passed to `rasterizeTriangle`.
        // This means I need to show the change in `rasterizeTriangle`'s signature here for context,
        // and then the payload part.

        // Let's assume `rasterizeTriangle` now has `const Material& currentMaterial` as a parameter.
        // This change is just for the payload line. The signature change itself is out of scope for this specific tool call.

        // The provided snippet is inside `rasterizeTriangle`.
        // The `texture` variable is a member of `Rasterizer`, set by `set_texture`.
        // This is tricky. The current `texture` is set from `object.material.diffuseTextureFile`.
        // The `fragment_shader_payload` needs `const Material*`.
        // The `object.material` is the source.

        // The most direct way with minimal changes to function signatures for *this specific tool call*
        // is to assume that `vertices[0].face->material` is accessible, or that `object` is passed down.
        // However, the instructions are to modify *this* file.
        // The `texture` in payload is `texture ? &*texture : nullptr`. This uses the rst::texture.
        // This needs to be `&face->material.texture` if texture is part of material, or just pass material.

        // The `fragment_shader_payload` constructor is:
        // `fragment_shader_payload(const Eigen::Vector3f& col, const Eigen::Vector3f& nor, const Eigen::Vector2f& tc, const std::vector<Light> vl, Texture* tex, const Material* mat)`

        // The `texture` object in `Rasterizer` is populated from `object.material.diffuseTextureFile`.
        // So, `&(*texture)` is the texture part. The material itself is `object.material`.
        // This implies `object.material` needs to be passed to `rasterizeTriangle`.

        // Let's assume `rasterizeTriangle` is called like this:
        // `rasterizeTriangle(screenspace_vertices, viewspace_vertices, viewspace_lights, object.material);`
        // So, `rasterizeTriangle` signature becomes:
        // `void rst::Rasterizer::rasterizeTriangle(std::vector<Vertex> &vertices, std::vector<Eigen::Vector3f> &view_pos, std::vector<Light> &view_lights, const Material& currentMaterial)`
        // Then inside `rasterizeTriangle`:
        fragment_shader_payload payload(vertices[j].colour,
                                        vertices[j].computeNormal(),
                                        vertices[j].textureCoordinates,
                                        view_lights,
                                        texture ? &*texture : nullptr, // This uses the rst::texture if set
                                        &currentMaterial); // Pass the material
        payload.view_pos = view_pos[j];
        Eigen::Vector3f vertex_colour = fragment_shader(payload);
        vertex_colours[j] = vertex_colour;
    }

    // Iterate over each pixel in the bounding box
    for (int x = minX; x <= maxX; ++x) {
        for (int y = minY; y <= maxY; ++y) {
            // Iterate over each subpixel
            for (int i = 0; i < 4; ++i) {
                float px = x + ssaa_offset[i].x();
                float py = y + ssaa_offset[i].y();

                // Check if the subpixel is inside the triangle
                if (insideTriangle(px, py, v)) {
                    // Compute barycentric coordinates
                    auto [alpha, beta, gamma] = computeBarycentric2D(px, py, v);

                    // Interpolate depth
                    float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + 
                                            beta * v[1].z() / v[1].w() + 
                                            gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;

                    // Compute index for subpixel in the SSAA buffer
                    int index = 4 * getIndex(x, y) + i;

                    // Perform depth test
                    if (z_interpolated < ssaaDepthBuffer[index]) {
                        ssaaDepthBuffer[index] = z_interpolated;

                        // If we were doing Phong shading per pixel, we would construct payload here.
                        // However, the current code structure does Gouraud shading (colors computed at vertices and interpolated).
                        // The task is to use material properties in shaders.
                        // The shaders are called for vertex_colours.
                        // So the modification in the loop above for vertex_colours is the primary place.

                        // For per-pixel Phong shading (more accurate):
                        // fragment_shader_payload payload_pixel(...); // Fill with interpolated attributes
                        // payload_pixel.material = &currentMaterial;
                        // Eigen::Vector3f pixel_colour = fragment_shader(payload_pixel);

                        // Current code: Gouraud shading
                        Eigen::Vector3f pixel_colour = interpolate(alpha, beta, gamma,
                            vertex_colours[0], vertex_colours[1], vertex_colours[2], alpha + beta + gamma);

                        // Set pixel colour in the SSAA buffer
                        ssaaFrameBuffer[index] = pixel_colour;
                    }
                }
            }
        }
    }
}

// Set a pixel in the frame buffer to a specific color
void rst::Rasterizer::setPixel(const Vector2i& position, const Eigen::Vector3f& colour) {
    int index = getIndex(position.x(), position.y());
    frameBuffer[index] = colour;
}