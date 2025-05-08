#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <fstream>
#include <sstream>
#include <unordered_map>
#include "Eigen/Dense"

#include "../headerFiles/meshChecker.hpp"
#include "../headerFiles/object.hpp"

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

    auto all_vertices_sptr = make_shared<vector<Vertex>>();
    auto all_faces_sptr = make_shared<vector<Face>>();
    auto all_halfedges_sptr = make_shared<vector<HalfEdge>>();

    all_vertices_sptr->reserve(objData.vertices.size());
    for (size_t i = 0; i < objData.vertices.size(); ++i) {
        all_vertices_sptr->emplace_back(i); // Assuming Vertex constructor takes id
        Vertex& current_vertex = all_vertices_sptr->back();
        current_vertex.position = objData.vertices[i];
        // Initialize texture coordinates. Proper mapping will happen per face-vertex.
        current_vertex.textureCoordinates = Vector3f(0, 0, 0);
    }

    // Map to find twin half-edges. Key is a pair of vertex indices (min_idx, max_idx).
    // Value is a weak_ptr to the first half-edge found for that edge.
    unordered_map<uint64_t, weak_ptr<HalfEdge>> edge_to_halfedge_map;

    int halfEdgeIdCounter = 0;
    for (size_t faceIdx = 0; faceIdx < objData.faces.size(); ++faceIdx) {
        const auto& vertex_indices_for_face = objData.faces[faceIdx];
        if (vertex_indices_for_face.size() < 3) continue;

        // Create Face object
        all_faces_sptr->emplace_back(faceIdx); // Assuming Face constructor takes id
        Face& current_face_ref = all_faces_sptr->back();
        // Use aliasing constructor for shared_ptr to an element in the vector
        shared_ptr<Face> current_face_sptr(all_faces_sptr, &current_face_ref);

        vector<shared_ptr<HalfEdge>> current_face_he_sptrs;
        current_face_he_sptrs.reserve(vertex_indices_for_face.size());

        // Create HalfEdge objects for this face
        for (size_t i = 0; i < vertex_indices_for_face.size(); ++i) {
            all_halfedges_sptr->emplace_back(halfEdgeIdCounter++); // Assuming HE constructor takes id
            HalfEdge& current_he_ref = all_halfedges_sptr->back();
            shared_ptr<HalfEdge> current_he_sptr(all_halfedges_sptr, &current_he_ref); // Aliasing

            current_face_he_sptrs.push_back(current_he_sptr);

            // Set vertex
            int v_obj_idx = vertex_indices_for_face[i];
            if (v_obj_idx < 0 || static_cast<size_t>(v_obj_idx) >= all_vertices_sptr->size()) {
                cerr << "Error: Vertex index " << v_obj_idx << " out of bounds for face " << faceIdx << endl;
                continue;
            }
            shared_ptr<Vertex> he_vertex_sptr(all_vertices_sptr, &(*all_vertices_sptr)[v_obj_idx]);
            current_he_sptr->vertex = he_vertex_sptr;

            // Set face
            current_he_sptr->face = current_face_sptr;

            // Set outgoing half-edge for the vertex (if not already set)
            if (he_vertex_sptr->halfEdge.expired()) {
                he_vertex_sptr->halfEdge = current_he_sptr;
            }
            
            // Assign texture coordinates to vertex if available for this specific face vertex
            if (faceIdx < objData.textureIdx.size() && i < objData.textureIdx[faceIdx].size()) {
                int vt_idx = objData.textureIdx[faceIdx][i];
                if (vt_idx >= 0 && static_cast<size_t>(vt_idx) < objData.textureCoords.size()) {
                    // This updates the vertex's main texture coordinate.
                    // If per-face-vertex texture coordinates are needed, Vertex struct would need to change.
                    he_vertex_sptr->textureCoordinates = Vector3f(objData.textureCoords[vt_idx].x(), objData.textureCoords[vt_idx].y(), 0.0f);
                }
            }
        }

        // Link next/previous pointers for the face's half-edges
        for (size_t i = 0; i < current_face_he_sptrs.size(); ++i) {
            current_face_he_sptrs[i]->next = current_face_he_sptrs[(i + 1) % current_face_he_sptrs.size()];
            current_face_he_sptrs[i]->previous = current_face_he_sptrs[(i + current_face_he_sptrs.size() - 1) % current_face_he_sptrs.size()];
        }

        // Set face's starting half-edge
        if (!current_face_he_sptrs.empty()) {
            current_face_sptr->halfEdge = current_face_he_sptrs[0];
        }

        // Link twin half-edges
        for (size_t i = 0; i < current_face_he_sptrs.size(); ++i) {
            shared_ptr<HalfEdge> he1 = current_face_he_sptrs[i];
            // The vertex he1 points TO is its next half-edge's origin vertex
            shared_ptr<HalfEdge> he1_next = he1->next;

            if (!he1->vertex || !he1_next || !he1_next->vertex) {
                 cerr << "Error: Null vertex in halfedge for face " << faceIdx << " during twin linking." << endl;
                 continue;
            }

            // Edge is from he1->vertex to he1_next->vertex
            int v1_idx = he1->vertex->id;
            int v2_idx = he1_next->vertex->id;

            uint64_t edge_key = (static_cast<uint64_t>(min(v1_idx, v2_idx)) << 32) | static_cast<uint64_t>(max(v1_idx, v2_idx));

            if (edge_to_halfedge_map.count(edge_key)) {
                shared_ptr<HalfEdge> twin_he = edge_to_halfedge_map[edge_key].lock();
                if (twin_he) {
                    he1->twin = twin_he; // he1 is from v1 to v2
                    twin_he->twin = he1; // twin_he was from v2 to v1
                    edge_to_halfedge_map.erase(edge_key);
                } else {
                     // This case (found in map but weak_ptr expired) should ideally not happen with correct logic.
                     // It might indicate a non-manifold edge processed earlier or an issue.
                     // For robustness, one might re-insert or log. Here, we'll re-insert.
                    edge_to_halfedge_map[edge_key] = he1;
                }
            } else {
                edge_to_halfedge_map[edge_key] = he1;
            }
        }
        
        // Compute face normal
        vector<shared_ptr<Vertex>> temp_face_vertices_sptr;
        if (auto start_he_locked = current_face_sptr->halfEdge.lock()){
            shared_ptr<HalfEdge> current_he = start_he_locked;
            do {
                if(current_he && current_he->vertex){
                    temp_face_vertices_sptr.push_back(current_he->vertex);
                } else {
                    cerr << "Warning: Null vertex or HE encountered while collecting vertices for normal calculation on face " << faceIdx << endl;
                    break;
                }
                current_he = current_he->next;
                 if (!current_he) {
                    cerr << "Warning: Null next HE encountered while collecting vertices for normal calculation on face " << faceIdx << endl;
                    break;
                }
            } while (current_he != start_he_locked);
        }
        if(temp_face_vertices_sptr.size() >= 3) {
            current_face_sptr->normal = computeFaceNormal(temp_face_vertices_sptr);
        } else {
            current_face_sptr->normal = Vector3f(0,0,1); // Default normal for invalid faces
            cerr << "Warning: Face " << faceIdx << " has < 3 vertices for normal calculation." << endl;
        }
    }
    
    // Report any unmatched half-edges (boundaries or errors)
    for(const auto& pair : edge_to_halfedge_map){
        if(auto he = pair.second.lock()){
            // These are boundary half-edges or indicate an error (e.g. non-manifold)
            // For now, we just note they exist. They should not have a twin.
            // cerr << "Boundary or error: HalfEdge " << he->id << " (from V" << he->vertex->id << ") has no twin." << endl;
        }
    }


    object.setVertices(all_vertices_sptr);
    object.setFaces(all_faces_sptr);
    object.setHalfEdges(all_halfedges_sptr);

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