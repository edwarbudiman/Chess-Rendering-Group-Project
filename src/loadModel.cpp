#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <fstream>
#include <sstream>
#include <unordered_map>
#include "Eigen/Dense"

#include <meshChecker.hpp>
#include <object.hpp>

using namespace std;
using namespace Eigen;

// Structure to store raw data from OBJ file
struct OBJData {
    vector<Vector3f> vertices;
    vector<Vector3f> normals;
    vector<Vector2f> textureCoords;
    vector<vector<int>> faces;         // Vertex indices for each face
    vector<vector<int>> textureIdx;    // Texture indices for each face
    vector<vector<int>> normalIdx;     // Normal indices for each face
};

// Parse OBJ file and return raw data
OBJData parseOBJ(const string& filename) {
    OBJData data;
    ifstream file(filename);
    
    if (!file.is_open()) {
        cerr << "Error: Could not open file " << filename << endl;
        return data;
    }
    
    string line;
    while (getline(file, line)) {
        istringstream iss(line);
        string token;
        iss >> token;
        
        if (token == "v") {
            // Parse vertex
            float x, y, z;
            iss >> x >> y >> z;
            data.vertices.push_back(Vector3f(x, y, z));
        } 
        else if (token == "vt") {
            // Parse texture coordinate
            float u, v;
            iss >> u >> v;
            data.textureCoords.push_back(Vector2f(u, v));
        } 
        else if (token == "vn") {
            // Parse normal
            float nx, ny, nz;
            iss >> nx >> ny >> nz;
            data.normals.push_back(Vector3f(nx, ny, nz));
        } 
        else if (token == "f") {
            // Parse face
            vector<int> faceVertices;
            vector<int> faceTextures;
            vector<int> faceNormals;
            
            string vertex;
            while (iss >> vertex) {
                istringstream vss(vertex);
                string vPart;
                
                // Handle different face formats (v, v/vt, v//vn, v/vt/vn)
                getline(vss, vPart, '/');
                int vIdx = stoi(vPart) - 1;  // OBJ indices are 1-based
                faceVertices.push_back(vIdx);
                
                if (getline(vss, vPart, '/')) {
                    if (!vPart.empty()) {
                        int vtIdx = stoi(vPart) - 1;
                        faceTextures.push_back(vtIdx);
                    } else {
                        faceTextures.push_back(-1);  // No texture coordinate
                    }
                    
                    if (getline(vss, vPart, '/')) {
                        int vnIdx = stoi(vPart) - 1;
                        faceNormals.push_back(vnIdx);
                    } else {
                        faceNormals.push_back(-1);  // No normal
                    }
                }
            }
            
            data.faces.push_back(faceVertices);
            data.textureIdx.push_back(faceTextures);
            data.normalIdx.push_back(faceNormals);
        }
    }
    
    file.close();
    return data;
}

// Compute face normal from vertices
Vector3f computeFaceNormal(const vector<shared_ptr<Vertex>>& vertices) {
    if (vertices.size() < 3) {
        return Vector3f(0, 0, 0);
    }
    
    Vector3f v1 = vertices[1]->position - vertices[0]->position;
    Vector3f v2 = vertices[2]->position - vertices[0]->position;
    Vector3f normal = v1.cross(v2);
    normal.normalize();
    return normal;
}

// Convert raw OBJ data to half-edge structure
Object meshToHalfEdge(const OBJData& objData) {
    Object object;
    vector<Vertex> vertices;
    vector<Face> faces;
    vector<HalfEdge> halfEdges;
    
    // Create vertices
    for (int i = 0; i < objData.vertices.size(); i++) {
        Vertex vertex(i);
        vertex.position = objData.vertices[i];
        if (i < objData.textureCoords.size()) {
            vertex.textureCoordinates = Vector3f(objData.textureCoords[i].x(), objData.textureCoords[i].y(), 0);
        } else {
            vertex.textureCoordinates = Vector3f(0, 0, 0);
        }
        vertices.push_back(vertex);
    }
    
    // We'll use this map to keep track of half-edge twins
    unordered_map<uint64_t, int> edgeMap;
    
    int halfEdgeIndex = 0;
    
    // Create faces and half-edges
    for (int faceIdx = 0; faceIdx < objData.faces.size(); faceIdx++) {
        const auto& faceVertices = objData.faces[faceIdx];
        if (faceVertices.size() < 3) continue; // Skip invalid faces
        
        Face face(faceIdx);
        vector<shared_ptr<HalfEdge>> faceHalfEdges;
        
        // Create half-edges for this face
        for (int i = 0; i < faceVertices.size(); i++) {
            HalfEdge halfEdge(halfEdgeIndex);
            halfEdges.push_back(halfEdge);
            faceHalfEdges.push_back(make_shared<HalfEdge>(halfEdge));
            halfEdgeIndex++;
        }
        
        // Connect half-edges
        for (int i = 0; i < faceHalfEdges.size(); i++) {
            int nextIdx = (i + 1) % faceHalfEdges.size();
            int prevIdx = (i + faceHalfEdges.size() - 1) % faceHalfEdges.size();
            
            // Set next and previous pointers
            faceHalfEdges[i]->next = faceHalfEdges[nextIdx];
            faceHalfEdges[i]->previous = weak_ptr<HalfEdge>(faceHalfEdges[prevIdx]);
            
            // Set vertex and face
            int vertIdx = objData.faces[faceIdx][i];
            shared_ptr<Vertex> vertex = make_shared<Vertex>(vertices[vertIdx]);
            faceHalfEdges[i]->vertex = vertex;
            faceHalfEdges[i]->face = make_shared<Face>(face);
            
            // Store half-edge reference in vertex
            vertex->halfEdge = weak_ptr<HalfEdge>(faceHalfEdges[i]);
        }
        
        // Set face's half-edge reference
        face.halfEdge = weak_ptr<HalfEdge>(faceHalfEdges[0]);
        
        // Store the twin pairs to connect them later
        for (int i = 0; i < faceHalfEdges.size(); i++) {
            int nextIdx = (i + 1) % faceHalfEdges.size();
            int v1 = objData.faces[faceIdx][i];
            int v2 = objData.faces[faceIdx][nextIdx];
            
            // Create a unique key for edge (v1,v2)
            uint64_t edgeKey1 = (static_cast<uint64_t>(min(v1, v2)) << 32) | max(v1, v2);
            uint64_t edgeKey2 = (static_cast<uint64_t>(max(v1, v2)) << 32) | min(v1, v2);
            
            // Store the half-edge index
            if (edgeMap.count(edgeKey1) > 0) {
                // Found a twin
                int twinIdx = edgeMap[edgeKey1];
                faceHalfEdges[i]->twin = weak_ptr<HalfEdge>(faceHalfEdges[twinIdx]);
                faceHalfEdges[twinIdx]->twin = weak_ptr<HalfEdge>(faceHalfEdges[i]);
            } else {
                // No twin found yet, store this edge
                edgeMap[edgeKey2] = halfEdges.size() - faceHalfEdges.size() + i;
            }
        }
        
        // Compute face normal
        vector<shared_ptr<Vertex>> faceVerticesPtr;
        for (auto& he : faceHalfEdges) {
            faceVerticesPtr.push_back(he->vertex);
        }
        face.normal = computeFaceNormal(faceVerticesPtr);
        
        faces.push_back(face);
    }
    
    object.setVertices(vertices);
    object.setFaces(faces);
    object.setHalfEdges(halfEdges);
    
    return object;
}

// Main load function that parses OBJ file and converts to half-edge structure
Object load(string fileLocation) {
    try {
        OBJData objData = parseOBJ(fileLocation);
        
        if (objData.vertices.empty() || objData.faces.empty()) {
            cerr << "Error: Invalid OBJ file or no data loaded from " << fileLocation << endl;
            return Object();
        }
        
        // Extract texture file name from file path if available
        size_t lastSlash = fileLocation.find_last_of("/\\");
        size_t lastDot = fileLocation.find_last_of(".");
        string baseDir = lastSlash != string::npos ? fileLocation.substr(0, lastSlash + 1) : "";
        string baseName = fileLocation.substr(lastSlash + 1, lastDot - lastSlash - 1);
        string potentialTexturePath = baseDir + baseName + "_diffuse.jpg";
        
        Object object = meshToHalfEdge(objData);
        object.textureFile = potentialTexturePath; // Set potential texture file path
        
        return object;
    } catch (exception& e) {
        cerr << "Error loading model: " << e.what() << endl;
        return Object();
    }
}