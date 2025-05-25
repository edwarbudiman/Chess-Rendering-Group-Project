#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <map>
#include "Eigen/Dense"
#include <fstream>
#include <set>

#include <meshChecker.hpp>
#include <object.hpp>
#include <loadModel.hpp>

using namespace std;
using namespace Eigen;

vector<string> split(const string &s, const char delim){
    vector<string> result;
    stringstream ss (s);
    string item;

    while(getline (ss, item, delim)) {
        result.push_back (item);
    }
    return result;
}

Material LoadMaterial(string path, string fileName){
    if(fileName.substr(fileName.size() - 4, 4) != ".mtl"){
        fileName.append(".mtl");
    }
    std::ifstream file(path + fileName);
    if(!file.is_open()){
        string badFile = "can't open file ";
        badFile.append(path + fileName);
        throw badFile; 
    }

    Material material;
    material.mtlFile = fileName;
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

vector<int> getVertexInfo(set<RawVertex> &vertices, const vector<string> &face, const vector<Vector3f> &positions, const vector<Vector2f> &tCoords, const vector<Vector3f> &normals){
    vector<int> verticesIndex;
    bool noNormal = false;
    int startIndex = vertices.size();
    vector<RawVertex> faceVertices;
    for(string stringVertex : face){
        RawVertex vertex;
        int positionIndex;
        int vertexType;
        char delim = '/';
        vector<string> parts = split(stringVertex, delim);
        // Check for just position - v1
        if (parts.size() == 1){
            vertex.position = positions[stoi(parts[0]) - 1];
            vertex.textureCoords = Vector2f(0, 0);
            noNormal = true;
        }
        // Check for position & texture - v1/vt1
        else if (parts.size() == 2){
            vertex.position = positions[stoi(parts[0]) - 1];
            vertex.textureCoords = tCoords[stoi(parts[1]) - 1];
            noNormal = true;
        }
        // Check for Position, Texture and Normal - v1/vt1/vn1
        // or if Position and Normal - v1//vn1
        else if (parts.size() == 3){
            if (parts[1] != ""){
                vertex.position = positions[stoi(parts[0]) - 1];
                vertex.textureCoords = tCoords[stoi(parts[1]) - 1];
                vertex.normal = normals[stoi(parts[2])];
            }
            else{
                vertex.position = positions[stoi(parts[0]) - 1];
                vertex.textureCoords = Vector2f(0, 0);
                vertex.normal = normals[stoi(parts[2])];
            }
        }
        faceVertices.push_back(vertex);
    }
    if (noNormal){
        Vector3f a = faceVertices[1].position - faceVertices[0].position;
        Vector3f b = faceVertices[2].position - faceVertices[0].position;
        Vector3f normal = a.cross(b);

        for (RawVertex &vertex : faceVertices){
            vertex.normal = normal;
        }
    }
    for(RawVertex &vertex : faceVertices){
        auto it = vertices.find(vertex);
        if(it == vertices.end()){
            int id = vertices.size();
            vertex.id = id;
            vertices.insert(vertex);
            verticesIndex.push_back(id);
        }
        else{
            verticesIndex.push_back(it->id);
        }
    }
    return verticesIndex;
}

Mesh loadFile(string path){
    // If the file is not an .obj file return false
    Mesh mesh;
    Material material;
    if(path.substr(path.size() - 4, 4) != ".obj"){
        string badFile = "file not .obj ";
        badFile.append(path);
        throw badFile; 
    }

    std::ifstream file(path);

    if(!file.is_open()){
        string badFile = "can't open file ";
        badFile.append(path);
        throw badFile; 
    }

    vector<Vector3f> positions;
    vector<Vector2f> tCoords;
    vector<Vector3f> normals;
    set<RawVertex> vertices;
    vector<vector<int>> faces;
    string currentLine;
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
            //update vertices and get face info
            vector<int> face = getVertexInfo(vertices, stringFace, positions, tCoords, normals);
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
    vector<RawVertex> verticesVector(vertices.size());
    for(RawVertex rawVertex : vertices){
        verticesVector[rawVertex.id] = rawVertex;
    }
    mesh.vertices = verticesVector;
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
            shared_ptr<HalfEdge> halfEdge = make_shared<HalfEdge>(HalfEdge(halfEdgeIdCounter++));
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

Object load(string fileLocation, string fileName) {
    Object object;
    try{
        Mesh mesh = loadFile(fileLocation);
        cout << "\tOBJ file loaded successfully: " << fileLocation << std::endl;
        object = meshToHalfEdge(mesh);
        object.objFile = fileName;
        cout << "\tmesh successfully made for " << fileName << std::endl;

    }
    catch(string message){
        message = "in loadModel " + message;
        throw message;
    }
    checkMesh(object, fileName);
    return object;
}