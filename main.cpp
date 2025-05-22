#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include "Eigen/Dense"
#include <filesystem>
#include <map>
#include <stdexcept> // Added for std::exception

#include <scene.hpp>
#include <object.hpp>
#include <rasterizer.hpp>
#include <camera.hpp>
#include <shader.hpp>
#include "headerFiles/loadModel.hpp" // Already included shader.hpp via <shader.hpp>

using namespace std;
using namespace Eigen;
using namespace filesystem;

void captureImage(){
    
}

Matrix4f rotateScene(float angle, Vector3f axis){
    
    return Matrix4f::Identity();
}

void renderScene(Scene scene){
    // Manually create camera for now -- potentially add to scene class
    Camera camera(Vector3f(0, 0, 10), Vector3f(0, 0, 0), Vector3f(0, 1, 0), 45.0f, 1.0f, 0.1f, 50.0f);
    std::function<Eigen::Vector3f(fragment_shader_payload)> active_shader = texture_fragment_shader;

    // Set up rasterizer
    float height = 512;
    float width = 512;
    rst::Rasterizer r(width, height);
    r.clear(rst::Buffers::Colour | rst::Buffers::Depth);
    r.setView(camera.getViewMatrix());
    r.setProjection(camera.getProjectionMatrix());
    r.setFragmentShader(active_shader);
    // Draw objects
    r.rasterizeObjects(scene);
    cv::Mat image(width, height, CV_32FC3, r.frame_buffer().data());
    image.convertTo(image, CV_8UC3, 1.0f);
    cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
    cv::imshow("image", image);
    cv::imwrite("output.png", image);

}

int main(){
    try{
        cout << "loading models\n";
        std::map<std::string, Object> objects = loadModels("../Models/");
        cout << "models loaded\n";
        cout << "building scene\n";
        Scene scene = loadSceneFromJson(objects, "../Models/chess_scene.json");
        cout << "scene built\n";
        cout << "rendering scene\n";
        auto start = std::chrono::system_clock::now();
        renderScene(scene);
        auto stop = std::chrono::system_clock::now();
        cout << "scene rendered\n";
        auto time = stop - start;
        auto hours = std::chrono::duration_cast<std::chrono::hours>(time);
        time = time - hours;
        auto minutes = std::chrono::duration_cast<std::chrono::minutes>(time);
        time = time - minutes;
        auto seconds = std::chrono::duration_cast<std::chrono::seconds>(time).count();
        cout << "render time: " << hours.count() << ":" << minutes.count() << ":" << seconds << "\n";
    }
    catch(const std::exception& e){ // Changed to catch std::exception
        cerr << "exception occurred, message: " << e.what() << "\n"; // Used e.what()
    }
    
    return 0;
}