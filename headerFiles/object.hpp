#pragma once

#include <memory>
#include <vector>
#include <Eigen/Dense>
class Object;
class Face;
class HalfEdge;
class Vertex;

class HalfEdge{
    public:
        int id;
        std::shared_ptr<HalfEdge> next;
        std::weak_ptr<HalfEdge> previous; //make doubly linked, as being singly linked is stupid
        std::weak_ptr<HalfEdge> twin;
        std::shared_ptr<Vertex> vertex; //origin vertex
        std::shared_ptr<Face> face;
        HalfEdge(int _id): id(_id){}
};

class Face{
    public:
        int id;
        std::weak_ptr<HalfEdge> halfEdge;
        Eigen::Vector3f normal;
        //need to figure out how to represent material
        //auto material;
        Face(){}
        Face(int _id): id(_id){}
        std::vector<std::shared_ptr<HalfEdge>> getHalfEdges() const {
            std::vector<std::shared_ptr<HalfEdge>> halfEdges;
            std::shared_ptr<HalfEdge> currentHalfEdge = halfEdge.lock();
            if (!currentHalfEdge) return halfEdges; // Check if lock failed
            int halfEdgeId = currentHalfEdge->id;
            do{
                halfEdges.push_back(currentHalfEdge);
                currentHalfEdge = currentHalfEdge->next;
                if (!currentHalfEdge) break; // Safety break if list is malformed
            }
            while(currentHalfEdge->id != halfEdgeId);
            return halfEdges;
        }
        std::vector<std::shared_ptr<Vertex>> getVertices() const {
            std::vector<std::shared_ptr<Vertex>> vertices;
            std::shared_ptr<HalfEdge> currentHalfEdge = halfEdge.lock();
            if (!currentHalfEdge) return vertices; // Check if lock failed
            int halfEdgeId = currentHalfEdge->id;
            do{
                vertices.push_back(currentHalfEdge->vertex);
                currentHalfEdge = currentHalfEdge->next;
                if (!currentHalfEdge) break; // Safety break if list is malformed
            }
            while(currentHalfEdge->id != halfEdgeId);
            return vertices;
        }
};

class Vertex{
    public:
        int id;
        std::weak_ptr<HalfEdge> halfEdge;
        Eigen::Vector3f position;
        Eigen::Vector3f textureCoordinates;
        Eigen::Vector3f colour; //if not using texture, in range [0, 255]
        Vertex(int _id): id(_id){}
        std::vector<std::shared_ptr<Vertex>> getNeighbourVertices(){
            std::vector<std::shared_ptr<Vertex>> neighbourhood;
            std::shared_ptr<HalfEdge> currentHalfEdge = halfEdge.lock();
            int halfEdgeId = currentHalfEdge->id;
            if(!currentHalfEdge){
                return neighbourhood;
            }
            do {
                std::shared_ptr<HalfEdge> twin = currentHalfEdge->twin.lock();
                if(!currentHalfEdge || !twin || !twin->vertex){
                    break;
                }
                neighbourhood.push_back(twin->vertex);
                currentHalfEdge = twin->next;
            }
            while(currentHalfEdge->id != halfEdgeId);
            return neighbourhood; 
        }
};

//trying out memory managment techniques here, decided to try out weak pointers
//to get the contents you have to "lock it"
//E.g: weak_ptr<vector<Face>> weakFaces = object.getFaces();
// shared_ptr<vector<Face>> faces = weakFaces.lock();
// shared_ptr<HalfEdge> he = faces[0]->halfEdge;
//or shared_ptr<HalfEdge> he = object.getFaces().lock()[0]->halfEdge;
class Object{
    public:
        std::string textureFile;
        Object(){}
        std::weak_ptr<std::vector<Face>> getFaces() const {
            return std::make_shared<std::vector<Face>>(faces);
        }
        std::weak_ptr<std::vector<HalfEdge>> getHalfEdges() const {
            return std::make_shared<std::vector<HalfEdge>>(halfEdges);
        }
        std::weak_ptr<std::vector<Vertex>> getVertices() const {
            return std::make_shared<std::vector<Vertex>>(vertices);
        }
        void setFaces(std::vector<Face> _faces){
            faces = _faces;
        }
        void setHalfEdges(std::vector<HalfEdge> _halfEdges){
            halfEdges = _halfEdges;
        }
        void setVertices(std::vector<Vertex> _vertices){
            vertices = _vertices;
        }
    private:
        std::vector<Face> faces;
        std::vector<HalfEdge> halfEdges;
        std::vector<Vertex> vertices;

};
