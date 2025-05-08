#pragma once

#include <Eigen/Dense>

class Light;
class QuadrilateralAreaLight;
class ellipseAreaLight;

class Light{
    public:
        Eigen::Vector3f position; // {x, y, z}
        Eigen::Vector3f intensity; // {r, g, b}
        Light(){}
        Light(Eigen::Vector3f _position, Eigen::Vector3f _intensity): position(_position), intensity(_intensity){}
        Light* clone() const{
            return new Light(*this);
        }
        Eigen::Vector3f SamplePoint() const {
            return position;
        }
};

class QuadrilateralAreaLight: Light{
    //by default this is facing +z, need to rotate to place in right position
    //do this by setting rotation matrix
    float height;
    float length;
    Eigen::Matrix4f rotationMatrix = Eigen::Matrix4f::Identity();
    QuadrilateralAreaLight(float _height, float _length, Eigen::Matrix4f _rotationMatrix, Eigen::Vector3f _position, Eigen::Vector3f _intensity){
        height = _height;
        length = _length;
        rotationMatrix = _rotationMatrix;
        this->position = _position;
        this->intensity = _intensity;
    }
    QuadrilateralAreaLight* clone() const{
        return new QuadrilateralAreaLight(*this);
    }
    Eigen::Vector3f SamplePoint() const {
        //get random height and length
        float randomHeight = static_cast<float>(rand()) / static_cast<float>(RAND_MAX); //get random float between 0 and 1;
        randomHeight *= height; //make it a random float between 0 and height
        randomHeight -= height / 2;// make it a random float between -1/2 height and +1/2 height
        //do the same for length
        float randomLength = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        randomLength *= length;
        randomLength -= length / 2;

        Eigen::Vector4f homoSamplePoint_local = {randomLength, randomHeight, 0, 0};
        //rotate samplePoint around origin to get coordinates to match
        Eigen::Vector4f homoSamplePoint_world = rotationMatrix * homoSamplePoint_local;
        Eigen::Vector3f finalSamplePoint = position + homoSamplePoint_world.head(3);
        return finalSamplePoint;
    }
};

class ellipseAreaLight: Light{
    //by default this is facing +z, need to rotate to place in right position
    //do this by setting rotation matrix
    float height;
    float length;
    Eigen::Matrix4f rotationMatrix = Eigen::Matrix4f::Identity();
    ellipseAreaLight(float _height, float _length, Eigen::Matrix4f _rotationMatrix, Eigen::Vector3f _position, Eigen::Vector3f _intensity){
        height = _height;
        length = _length;
        rotationMatrix = _rotationMatrix;
        this->position = _position;
        this->intensity = _intensity;
    }
    ellipseAreaLight* clone() const{
        return new ellipseAreaLight(*this);
    }
    Eigen::Vector3f SamplePoint() const {
        //formula for random point in ellipse from: https://stackoverflow.com/questions/5529148/algorithm-calculate-pseudo-random-point-inside-an-ellipse
        
        //get random float
        float phi = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        phi *= (2 * M_PI);

        float rho = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);

        //random point in unit circle
        float x = sqrt(rho) * cos(phi);
        float y = sqrt(rho) * sin(phi);
       
        // translate to ecclipse
        x = x * length/2.0;
        y = y * height/2.0;

        Eigen::Vector4f homoSamplePoint_local = {x, y, 0, 0};
        //rotate samplePoint around origin to get coordinates to match
        Eigen::Vector4f homoSamplePoint_world = rotationMatrix * homoSamplePoint_local;
        Eigen::Vector3f finalSamplePoint = position + homoSamplePoint_world.head(3);
        return finalSamplePoint;
    }
};