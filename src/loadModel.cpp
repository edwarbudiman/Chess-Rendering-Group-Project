#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <map>
#include "Eigen/Dense"
#include <fstream>
#include <stdexcept> // Added for std::runtime_error

#include "meshChecker.hpp"
#include "object.hpp"

using namespace std;
using namespace Eigen;

struct RawVertex{
    Vector3f position;
    Vector3f normal;
    Vector2f textureCoords;
};

struct Mesh{
    vector<RawVertex> vertices;
    vector<vector<int>> faces;
    Material material;
};

vector<string> split (const string &s, const char delim){
    vector<string> result;
    stringstream ss (s);
    string item;

    while (getline (ss, item, delim)) {
        result.push_back (item);
    }
    return result;
}

template <class T>
inline const T & getElement(const vector<T> &elements, string &stringIndex){
    int index = stoi(stringIndex);
    if (index < 0){
        index = int(elements.size()) + index;
    }
    else{
        index -= 1;
    }
    return elements[index];
}

float angleBetweenV3(const Vector3f a, const Vector3f b)        {
    float angle = a.dot(b);
    angle /= (a.norm() * b.norm());
    return angle = acosf(angle);
}

bool SameSide(Vector3f p1, Vector3f p2, Vector3f a, Vector3f b){
    Vector3f cp1 = (b - a).cross((p1 - a));
    Vector3f cp2 = (b - a).cross((p2 - a));

    if (cp1.dot(cp2) >= 0)
        return true;
    else
        return false;
}

bool inTriangle(Vector3f point, Vector3f tri1, Vector3f tri2, Vector3f tri3){
    // Test to see if it is within an infinite prism that the triangle outlines.
    bool within_tri_prisim = SameSide(point, tri1, tri2, tri3) && SameSide(point, tri2, tri1, tri3)
                                && SameSide(point, tri3, tri1, tri2);

    // If it isn't it will never be on the triangle
    if (!within_tri_prisim)
        return false;

    // Calulate Triangle's Normal
    Vector3f u = tri2 - tri1;
    Vector3f v = tri3 - tri1;
    Vector3f normal = u.cross(v);

    // Project the point onto this normal
    Vector3f bn = normal.normalized();
    Vector3f proj = bn * point.dot(bn);;

    // If the distance from the triangle to the point is 0
    //	it lies on the triangle
    if (proj.norm() == 0)
        return true;
    else
        return false;
}

tuple <vector<RawVertex>, vector<int>> getVertexInfo(vector<string> face, const vector<Vector3f> &positions, const vector<Vector2f> &tCoords, const vector<Vector3f> &normals){
    vector<RawVertex> vertices;
    vector<int> verticesIndex;
    bool noNormal = false;
    for(string stringVertex : face){
        RawVertex vertex;
        int positionIndex;
        int vertexType;
        char delim = '/';
        vector<string> parts = split(stringVertex, delim);
        // Check for just position - v1
        if (parts.size() == 1){
            vertex.position = getElement(positions, parts[0]);
            vertex.textureCoords = Vector2f(0, 0);
            noNormal = true;
            vertices.push_back(vertex);
            verticesIndex.push_back(stoi(parts[0]));
        }
        // Check for position & texture - v1/vt1
        else if (parts.size() == 2){
            vertex.position = getElement(positions, parts[0]);
            vertex.textureCoords = getElement(tCoords, parts[1]);
            noNormal = true;
            vertices.push_back(vertex);
            verticesIndex.push_back(stoi(parts[0]));
        }
        // Check for Position, Texture and Normal - v1/vt1/vn1
        // or if Position and Normal - v1//vn1
        else if (parts.size() == 3){
            if (parts[1] != "")
            {
                vertex.position = getElement(positions, parts[0]);
                vertex.textureCoords = getElement(tCoords, parts[1]);
                vertex.normal = getElement(normals, parts[2]);
                vertices.push_back(vertex);
                verticesIndex.push_back(stoi(parts[0]));
            }
            else
            {
                vertex.position = getElement(positions, parts[0]);
                vertex.textureCoords = Vector2f(0, 0);
                vertex.normal = getElement(normals, parts[2]);
                vertices.push_back(vertex);
                verticesIndex.push_back(stoi(parts[0]));
            }
        }
    }
    if (noNormal){
        Vector3f a = vertices[0].position - vertices[1].position;
        Vector3f b = vertices[2].position - vertices[1].position;
        Vector3f normal = a.cross(b);

        for (int i = 0; i < vertices.size(); i++){
            vertices[i].normal = normal;
        }
    }
    return make_tuple(vertices, verticesIndex);
}

Material LoadMaterial(string path, string fileName){
    if(fileName.substr(fileName.size() - 4, 4) != ".mtl"){
        fileName.append(".mtl");
    }
    std::ifstream file(path + fileName);
    if(!file.is_open()){
        string badFile = "can't open file ";
        badFile.append(path + fileName);
        throw std::runtime_error(badFile); 
    }

    Material material;
    // Go through each line looking for material variables
    string currentLine;
    while (getline(file, currentLine)){
        // new material and material name
        istringstream iss(currentLine);
        string prefix;
        iss >> prefix;
        // Ambient Color
        if (prefix == "Ka"){
            Vector3f light;
            iss >> light[0] >> light[1] >> light[2];
            material.ka = light;
        }
        // Diffuse Color
        if (prefix == "Kd"){
            Vector3f light;
            iss >> light[0] >> light[1] >> light[2];
            material.kd = light;
        }
        // Specular Color
        if (prefix == "Ks"){
            Vector3f light;
            iss >> light[0] >> light[1] >> light[2];
            material.ks = light;
        }
        // Specular Exponent
        if (prefix == "Ns"){
            float ns;
            iss >> ns;
            material.shininessExponant = ns;
        }
        // Optical Density
        if (prefix == "Ni"){
            float ni;
            iss >> ni;
            material.lightAbsorption = ni;
        }
        // Dissolve
        if (prefix == "d"){
            //don't know what dissolve is or if we need it/
        }
        // Illumination
        if (prefix == "illum"){
            float illum;
            iss >> illum;
            material.lightEmission = illum;
        }
        // Ambient Texture Map
        if (prefix == "map_Ka"){
            string map;
            iss >> map;
            map = path + map; 
            material.ambiantTextureFile = map;
        }
        // Diffuse Texture Map
        if (prefix == "map_Kd"){
            string map;
            iss >> map;
            map = path + map; 
            material.diffuseTextureFile = map;
        }
        // Specular Texture Map
        if (prefix == "map_Ks"){
            string map;
            iss >> map;
            map = path + map; 
            material.specularTextureFile = map;
        }
        // Specular Hightlight Map
        if (prefix == "map_Ns"){
            string map;
            iss >> map;
            map = path + map; 
            material.specularShininessTextureFile = map;
        }
        // Alpha Texture Map
        if (prefix == "map_d"){
            string map;
            iss >> map;
            map = path + map; 
            material.opacityTextureFile = map;
        }
        // Bump Map
        if (prefix == "map_Bump" || prefix == "map_bump" || prefix == "bump")
        {
            string map;
            iss >> map;
            map = path + map; 
            material.bumpMapTextureFile = map;
        }
    }
    return material;
}

Mesh loadFile(string path){
    // If the file is not an .obj file return false
    cerr << "entered loadFile\n";
    Mesh mesh;
    Material material;
    if(path.substr(path.size() - 4, 4) != ".obj"){
        string badFile = "file not .obj ";
        badFile.append(path);
        throw std::runtime_error(badFile); 
    }

    std::ifstream file(path);

    if(!file.is_open()){
        string badFile = "can't open file ";
        badFile.append(path);
        throw std::runtime_error(badFile); 
    }

    vector<Vector3f> positions;
    vector<Vector2f> tCoords;
    vector<Vector3f> normals;
    vector<RawVertex> vertices;
    vector<vector<int>> faces;
    string currentLine;
    int lineNum = 0;
    while(getline(file, currentLine)){
        istringstream iss(currentLine);
        string prefix;
        iss >> prefix;
        // Generate a Vertex Position
        if(prefix == "v"){
            Vector3f vertex;
            iss >> vertex[0] >> vertex[1] >> vertex[2];
            positions.push_back(vertex);
        }
        if (prefix == "vt"){
            Vector2f vertex;
            iss >> vertex[0] >> vertex[1];;
            tCoords.push_back(vertex);
            }
        // Generate a Vertex Normal;
        else if(prefix == "vn"){
            Vector3f vertex;
            iss >> vertex[0] >> vertex[1] >> vertex[2];

            normals.push_back(vertex);
        }
        // Generate a Face (vertices & indices)
        else if(prefix == "f"){
            // Generate the vertices
            vector<RawVertex> verts;
            vector<string> stringFace;
            for(string vertData; iss >> vertData;){
                stringFace.push_back(vertData);
            }
            auto [rawVertices, face] = getVertexInfo(stringFace, positions, tCoords, normals);

            // Add Vertices
            for(int i = 0; i < int(rawVertices.size()); i++){
                vertices.push_back(rawVertices[i]);
            }
            //add face
            faces.push_back(face);
        }
        else if(prefix == "usemtl"){
            string matFile;
            iss >> matFile;
            string directory;
            const size_t lastSlashIndex = path.rfind('/');
            if (std::string::npos != lastSlashIndex)
            {
                directory = path.substr(0, lastSlashIndex);
            }
            directory += "/";
            material = LoadMaterial(directory, matFile);
        }
    }
    mesh.vertices = vertices;
    mesh.faces = faces;
    mesh.material = material;

    file.close();
    return mesh;
}

Object meshToHalfEdge(const Mesh& mesh) {
    Object object;
    object.material = mesh.material;

    vector<shared_ptr<Vertex>> vertices(mesh.vertices.size(), nullptr);
    vector<shared_ptr<HalfEdge>> halfEdges;
    int vertexIdCounter = 0;
    int halfEdgeIdCounter = 0;
    int faceIdCounter = 0;
    map<int, vector<shared_ptr<HalfEdge>>> halfEdgesByDestination;

    for(int i = 0; i < mesh.vertices.size(); ++i){
        shared_ptr<Vertex> vertex = make_shared<Vertex>(vertexIdCounter++);
        vertex->position = mesh.vertices[i].position;
        vertex->normal = mesh.vertices[i].normal;
        vertex->textureCoordinates = mesh.vertices[i].textureCoords;
        // We'll set the halfEdge pointer later
        vertices[i] = vertex;
        object.vertices.push_back(vertex);
    }

    for(vector<int> faceIndices : mesh.faces){
        shared_ptr<Face> face = make_shared<Face>(Face(faceIdCounter++));
        shared_ptr<HalfEdge> previous = nullptr;
        shared_ptr<Vertex> firstVertex;
        shared_ptr<HalfEdge> firstHalfEdge;

        for(int vertexIndex: faceIndices){
            shared_ptr<HalfEdge> halfEdge = make_shared<HalfEdge>(HalfEdge(faceIdCounter++));
            if(previous != nullptr){
                previous->next = halfEdge;
                halfEdge->previous = previous;
                if (halfEdgesByDestination.find(vertexIndex) == halfEdgesByDestination.end()){
                    halfEdgesByDestination.insert({vertexIndex, vector<shared_ptr<HalfEdge>>()});
                }
                halfEdgesByDestination[vertexIndex].push_back(previous);
            }
            else{
                firstHalfEdge = halfEdge;
            }
            halfEdge->vertex = vertices[vertexIndex];
            halfEdge->face = face;
            halfEdges.push_back(halfEdge);
            object.halfEdges.push_back(halfEdge);
            previous = halfEdge;
        }

        previous->next = firstHalfEdge;
        firstHalfEdge->previous = previous;
        int id = firstHalfEdge->vertex->id;
        if (halfEdgesByDestination.find(id) == halfEdgesByDestination.end()){
            halfEdgesByDestination.insert({id, vector<shared_ptr<HalfEdge>>()});
        }
        halfEdgesByDestination[id].push_back(previous);
        face->halfEdge = firstHalfEdge;
        object.faces.push_back(face);
        // Calculate face normal
        Vector3f v0 = halfEdges[0]->vertex->position;
        Vector3f v1 = halfEdges[1]->vertex->position;
        Vector3f v2 = halfEdges[2]->vertex->position;
        Vector3f e1 = v1 - v0;
        Vector3f e2 = v2 - v0;
        face->normal = e1.cross(e2).normalized();
    }
    for(shared_ptr<HalfEdge> halfEdge : halfEdges){
        shared_ptr<HalfEdge> twin = halfEdge->twin.lock();
        if(!twin){
            int origin = halfEdge->vertex->id;
            int destination = halfEdge->next->vertex->id;
            for (shared_ptr<HalfEdge> twin : halfEdgesByDestination[origin]){
                if(twin->vertex->id == destination){
                    if(!twin->twin.lock()){
                        halfEdge->twin = twin;
                        twin->twin = halfEdge;
                        break;
                    }
                }
            }
        }
    }
    return object;
}

// Definition for void loadModel(map<string, Object> &objects, string fileLocation)
// Moved from original main.cpp context
void loadModel(map<string, Object> &objects, string fileLocation){
    cout << "loading model: " << fileLocation << "\n";
    char delim = '/';
    vector<string> fileSplit = split(fileLocation, delim); // 'split' is defined in this file
    string fileName = fileSplit.back();
    delim = '.';
    // Ensure there's at least one part after splitting by '.' to avoid error with files like "Makefile"
    vector<string> nameParts = split(fileName, delim);
    if (!nameParts.empty()) {
        fileName = nameParts[0];
    } else {
        // Keep original fileName if no '.' found, or handle as an error/log
        // For now, using original fileName if split by '.' yields nothing.
    }

    if(objects.count(fileName) > 0){
        throw std::runtime_error("multiple files with the same name: " + fileName);
    }
    cout << "mesh name: " << fileName << "\n";
    Object object = load(fileLocation, fileName); // 'load' is defined in this file
    cout << "successfully retreived object!\n";
    objects[fileName] = object;
}

Object load(string fileLocation, string fileName) {
    Object object;
    try{
        Mesh mesh = loadFile(fileLocation);
        cout << "\tOBJ file loaded successfully: " << fileLocation << std::endl;
        object = meshToHalfEdge(mesh);
        cout << "\tmesh successfully made for " << fileName << std::endl;

    }
    catch(const std::runtime_error& e){ // Catching runtime_error
        std::string message = "in loadModel " + std::string(e.what());
        throw std::runtime_error(message); // Re-throwing as runtime_error
    }
    catch(const std::string& message) { // Catching any remaining string throws (should be none from this file)
        std::string newMessage = "in loadModel (caught string): " + message;
        throw std::runtime_error(newMessage);
    }
    cout << "\tchecking consistency\n";
    if(!objectFacesConsistent){
        cout << "\tmaking " << fileName << " consistently faced: " << flush;
        makeObjectFacesConsistent(object);
        cout << "success\n";
    }
    cout << "\tchecking if object tri\n";
    if(!objectTri(object)){
        cout << "\tmaking " << fileName << " triangle mesh: " << flush;
        makeObjectTri(object);
        cout << "success\n";
    }
    cout << "\tchecking closed\n";
    if(!objectClosed){
        cerr << fileLocation << " not closed\n";
    }
    cout << "\tchecking connected\n";
    if(!objectConnected){
        cerr << fileLocation << " not fully connected\n";
    }
    cout << "\tchecking manifold\n";
    if(!objectManifold){
        cerr << fileLocation << " not manifold\n";
    }
    return object;
}