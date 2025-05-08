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
            std::vector<std::shared_ptr<HalfEdge>> face_half_edges;
            std::shared_ptr<HalfEdge> start_he = halfEdge.lock();
            
            if (!start_he) return face_half_edges; // Check if lock failed or face has no HE

            std::shared_ptr<HalfEdge> current_he = start_he;
            do {
                face_half_edges.push_back(current_he);
                current_he = current_he->next;
                if (!current_he) { // Safety break if list is malformed (e.g., open loop)
                    // Optionally, log an error or warning here
                    break;
                }
            } while (current_he != start_he); // Compare shared_ptr directly
            return face_half_edges;
        }
        std::vector<std::shared_ptr<Vertex>> getVertices() const {
            std::vector<std::shared_ptr<Vertex>> face_vertices;
            std::shared_ptr<HalfEdge> start_he = halfEdge.lock();

            if (!start_he) return face_vertices; // Check if lock failed or face has no HE

            std::shared_ptr<HalfEdge> current_he = start_he;
            do {
                if (current_he->vertex) { // Ensure vertex pointer is valid
                    face_vertices.push_back(current_he->vertex);
                } else {
                    // Optionally, log an error or warning here
                }
                current_he = current_he->next;
                if (!current_he) { // Safety break if list is malformed
                    // Optionally, log an error or warning here
                    break;
                }
            } while (current_he != start_he); // Compare shared_ptr directly
            return face_vertices;
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
        std::vector<std::shared_ptr<Vertex>> getNeighbourVertices() const {
            std::vector<std::shared_ptr<Vertex>> neighbourhood;
            std::shared_ptr<HalfEdge> start_he = halfEdge.lock();

            if (!start_he) return neighbourhood; // Vertex might be isolated or not properly linked

            std::shared_ptr<HalfEdge> current_he = start_he;
            do {
                std::shared_ptr<HalfEdge> twin = current_he->twin.lock();
                if (twin && twin->vertex) { // Ensure twin and its vertex are valid
                    neighbourhood.push_back(twin->vertex);
                } else {
                    // This vertex is on a boundary or the mesh is not manifold here
                    // Optionally, log or handle this case
                }
                
                if (twin) { // Proceed around the vertex using the twin's next
                    current_he = twin->next;
                } else { // If no twin, we cannot continue around this vertex in this manner
                    current_he = nullptr; // Stop the loop
                }

                if (!current_he) { // Safety break if list is malformed or boundary reached
                    break;
                }
            } while (current_he != start_he); // Compare shared_ptr directly
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
        
        Object() :
            m_faces(std::make_shared<std::vector<Face>>()),
            m_halfEdges(std::make_shared<std::vector<HalfEdge>>()),
            m_vertices(std::make_shared<std::vector<Vertex>>())
        {}

        std::weak_ptr<std::vector<Face>> getFaces() const {
            return m_faces;
        }
        std::weak_ptr<std::vector<HalfEdge>> getHalfEdges() const {
            return m_halfEdges;
        }
        std::weak_ptr<std::vector<Vertex>> getVertices() const {
            return m_vertices;
        }

        void setFaces(std::shared_ptr<std::vector<Face>> _faces){
            m_faces = _faces;
        }
        void setHalfEdges(std::shared_ptr<std::vector<HalfEdge>> _halfEdges){
            m_halfEdges = _halfEdges;
        }
        void setVertices(std::shared_ptr<std::vector<Vertex>> _vertices){
            m_vertices = _vertices;
        }
    private:
        std::shared_ptr<std::vector<Face>> m_faces;
        std::shared_ptr<std::vector<HalfEdge>> m_halfEdges;
        std::shared_ptr<std::vector<Vertex>> m_vertices;

};
