#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include "Eigen/Dense"


#include <object.hpp>
#include <src/loadModel.cpp>

using namespace std;
using namespace Eigen;

vector<Object> loadModel(string fileLocation){

    return vector<Object>();
}

vector<Object> loadModels(vector<string> files){

    return vector<Object>();
}

vector<Object> loadModels(string folder){

    return vector<Object>();
}

void captureImage(){
    
}

Matrix4f rotateScene(float angle, Vector3f axis){
    
    return Matrix4f::Identity();
}

int main() {
    // Path to a chess model OBJ file
    string modelPath = "../Models/Stone_Chess_Board/Stone_Chess_Board.obj";
    
    cout << "Testing load function from src/loadModel.cpp..." << endl;
    
    // Call the external load function directly
    // We're using load from loadModel.cpp, not our unimplemented loadModel function
    Object chessBoard = load(modelPath);
    
    // Print information about the loaded model
    cout << "Loaded model information:" << endl;
    cout << "Number of vertices: " << chessBoard.vertices.size() << endl;
    cout << "Number of faces: " << chessBoard.faces.size() << endl;
    cout << "Number of half edges: " << chessBoard.halfEdges.size() << endl;
    
    if (!chessBoard.vertices.empty()) {
        cout << "First vertex position: (" 
             << chessBoard.vertices[0]->position.x() << ", " 
             << chessBoard.vertices[0]->position.y() << ", " 
             << chessBoard.vertices[0]->position.z() << ")" << endl;
    }
    
    return 0;
}