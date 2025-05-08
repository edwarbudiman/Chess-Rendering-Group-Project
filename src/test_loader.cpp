#include <iostream>
#include <string>
#include <vector>
#include <iomanip>
#include <cmath>
#include "Eigen/Dense"

#include <meshChecker.hpp>
#include <object.hpp>

// Include the implementation directly since we're just testing
#include "loadModel.cpp"

using namespace std;
using namespace Eigen;

// Helper function to print mesh statistics
void printMeshStats(const Object& obj) {
    cout << "=== Mesh Statistics ===" << endl;
    
    auto vertices = obj.getVertices().lock();
    auto faces = obj.getFaces().lock();
    auto halfEdges = obj.getHalfEdges().lock();
    
    cout << "Vertices: " << vertices->size() << endl;
    cout << "Faces: " << faces->size() << endl;
    cout << "Half-Edges: " << halfEdges->size() << endl;
    
    // Check Euler characteristic (V - E + F = 2 for closed manifolds)
    int edgeCount = halfEdges->size() / 2;  // Each edge has two half-edges
    int eulerChar = vertices->size() - edgeCount + faces->size();
    cout << "Euler Characteristic: " << eulerChar << endl;
    
    // Print some vertex positions
    cout << "\nSample Vertices:" << endl;
    int sampleSize = min(5, (int)vertices->size());
    for (int i = 0; i < sampleSize; i++) {
        const auto& v = (*vertices)[i];
        cout << "  V" << v.id << ": (" 
             << fixed << setprecision(3) << v.position.x() << ", " 
             << v.position.y() << ", " 
             << v.position.z() << ")" << endl;
    }
    
    // Print some face information
    cout << "\nSample Faces:" << endl;
    sampleSize = min(5, (int)faces->size());
    for (int i = 0; i < sampleSize; i++) {
        const auto& f = (*faces)[i];
        cout << "  F" << f.id << " Normal: (" 
             << fixed << setprecision(3) << f.normal.x() << ", " 
             << f.normal.y() << ", " 
             << f.normal.z() << ")" << endl;
        
        // Print vertices of this face
        auto faceVertices = f.getVertices();
        cout << "    Vertices: ";
        for (const auto& v : faceVertices) {
            cout << v->id << " ";
        }
        cout << endl;
    }
    
    // Check half-edge connectivity (sample)
    cout << "\nSample Half-Edge Connectivity:" << endl;
    sampleSize = min(5, (int)halfEdges->size());
    for (int i = 0; i < sampleSize; i++) {
        const auto& he = (*halfEdges)[i];
        cout << "  HE" << he.id << ":" << endl;
        cout << "    Origin: V" << he.vertex->id << endl;
        cout << "    Next: HE" << he.next->id << endl;
        cout << "    Previous: HE" << he.previous.lock()->id << endl;
        if (auto twin = he.twin.lock()) {
            cout << "    Twin: HE" << twin->id << endl;
        } else {
            cout << "    Twin: None (boundary)" << endl;
        }
        cout << "    Face: F" << he.face->id << endl;
    }
    
    cout << "\nMesh texture file: " << obj.textureFile << endl;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        cout << "Usage: " << argv[0] << " <obj_file_path>" << endl;
        return 1;
    }
    
    string filePath = argv[1];
    cout << "Loading model from: " << filePath << endl;
    
    Object object = load(filePath);
    
    if (object.getVertices().lock()->empty()) {
        cout << "Failed to load the model." << endl;
        return 1;
    }
    
    printMeshStats(object);
    
    // Check if the mesh is closed
    bool isClosed = objectClosed(object);
    cout << "\nIs mesh closed: " << (isClosed ? "Yes" : "No") << endl;
    
    return 0;
}