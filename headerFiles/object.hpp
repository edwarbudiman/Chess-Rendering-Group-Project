#pragma once

#include <memory>
#include <vector>
#include <iostream>
#include <Eigen/Dense>
class Object;
class Material;
class Face;
class HalfEdge;
class Vertex;

class HalfEdge{
    public:
        int id; //unique within object
        std::shared_ptr<HalfEdge> next;
        std::weak_ptr<HalfEdge> previous; //make doubly linked, as being singly linked is stupid
        std::weak_ptr<HalfEdge> twin;
        std::shared_ptr<Vertex> vertex; //origin vertex
        std::shared_ptr<Face> face;
        HalfEdge(int _id): id(_id){}
        HalfEdge(){}
};

class Face{
    public:
        int id; //unique within object
        std::weak_ptr<HalfEdge> halfEdge;
        Eigen::Vector3f normal;
        Face(){}
        Face(int _id): id(_id){}
        std::vector<std::shared_ptr<HalfEdge>> getHalfEdges(){
            std::vector<std::shared_ptr<HalfEdge>> halfEdges;
            std::shared_ptr<HalfEdge> currentHalfEdge = halfEdge.lock();
            int halfEdgeId = currentHalfEdge->id;
            do{
                halfEdges.push_back(currentHalfEdge);
                currentHalfEdge = currentHalfEdge->next;
            }
            while(currentHalfEdge->id != halfEdgeId);
            return halfEdges;
        }
        std::vector<std::shared_ptr<Vertex>> getVertices(){
            std::vector<std::shared_ptr<Vertex>> vertices;
            std::shared_ptr<HalfEdge> currentHalfEdge = halfEdge.lock();
            int halfEdgeId = currentHalfEdge->id;
            do{
                vertices.push_back(currentHalfEdge->vertex);
                currentHalfEdge = currentHalfEdge->next;
            }
            while(currentHalfEdge->id != halfEdgeId);
            return vertices;
        }
};

class Vertex{
    public:
        int id; //unique within object
        std::weak_ptr<HalfEdge> halfEdge;
        Eigen::Vector3f position;
        Eigen::Vector3f normal = Eigen::Vector3f(0, 0, 0);
        Eigen::Vector2f textureCoordinates;
        Eigen::Vector3f colour; //if you are doing colour per vertex
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
        Vertex(){}

        // Get all the half edges that originate from this vertex
        std::vector<std::shared_ptr<HalfEdge>> neighbourHalfEdges() {
            std::vector<std::shared_ptr<HalfEdge>> neighbourhood;
            auto he = this->halfEdge;
            do {
                neighbourhood.push_back(he.lock());
                he = he.lock()->twin.lock()->next;
            }
            while(he.lock() != this->halfEdge.lock());
            return neighbourhood;
        }

        Eigen::Vector3f computeNormal() {
            if(this->normal == Eigen::Vector3f(0, 0, 0)) { // If normal is not already set
                Eigen::Vector3f accumulated_normal(0,0,0);
                std::shared_ptr<HalfEdge> start_he = halfEdge.lock();
                
                if (!start_he) { // Should not happen in a valid mesh structure
                    // Potentially return a default normal or handle error
                    return Eigen::Vector3f(0,0,1); // Default normal
                }

                std::shared_ptr<HalfEdge> current_he = start_he;
                bool first_iteration = true; // To handle do-while logic with a while loop or check before loop

                do {
                    if (!current_he || !current_he->face) {
                        // Invalid half-edge or face, skip or handle error
                        // This might indicate a boundary vertex if twin is null before next
                        // For now, let's assume manifold meshes for simplicity of circulation
                        if (current_he && current_he->twin.lock()) {
                             current_he = current_he->twin.lock()->next;
                        } else {
                            // Cannot circulate further, break or handle boundary
                            break; 
                        }
                        if (current_he == start_he && !first_iteration) break; // Avoid infinite loop on malformed geometry
                        continue;
                    }
                    
                    // Ensure face normal is valid (e.g., not zero)
                    // Face normals are assumed to be pre-calculated and normalized if needed.
                    // If face normals themselves can be zero, add a check here.
                    accumulated_normal += current_he->face->normal;

                    if (!current_he->twin.lock() || !current_he->twin.lock()->next) {
                        // Boundary edge, cannot continue circulation this way
                        // This means the vertex is on a boundary of the mesh.
                        // The loop should correctly terminate if start_he is part of a boundary.
                        // To correctly handle boundaries, we might need to iterate outgoing edges differently
                        // or accept partial smoothing. For now, this will sum available faces.
                        break; 
                    }
                    current_he = current_he->twin.lock()->next;
                    first_iteration = false;

                } while (current_he && current_he != start_he);

                // It's possible that for boundary vertices, the loop above doesn't add all faces.
                // A more robust way for boundaries is to iterate outgoing edges directly from the vertex if possible.
                // The current halfEdge circulation (he = he->twin->next) is standard for manifold, closed meshes.
                // For robustness, let's re-iterate if the first pass was partial due to boundary or if start_he was a boundary itself.
                // This part can be complex. The instruction was "he = he->twin.lock()->next;"
                // Let's stick to the simpler loop for now as specified and refine if issues arise.
                // The provided loop structure `he = he->twin.lock()->next;` implies we are iterating around the vertex.

                if (accumulated_normal.norm() > 0.0001f) { // Check if any normals were accumulated
                    this->normal = accumulated_normal.normalized();
                } else {
                    // Fallback if no face normals found (e.g. isolated vertex or all face normals are zero)
                    // This could also happen if start_he->face is null.
                    // Using one face normal as a last resort if start_he->face is valid.
                    if (start_he && start_he->face) {
                        this->normal = start_he->face->normal; // Original fallback
                    } else {
                        // Absolute fallback: a default normal or keep as zero
                        this->normal = Eigen::Vector3f(0,0,1); // Or (0,0,0) if preferred
                    }
                }
            }
            return this->normal;
        }
};

class Material{
    public:
        Eigen::Vector3f colour;
        std::string diffuseTextureFile;
        //below are optional
        std::string specularTextureFile;
        std::string ambiantTextureFile;
        std::string specularShininessTextureFile;
        std::string opacityTextureFile; //alpha
        std::string bumpMapTextureFile;
        float opacity;
        float ior; // index of refraction
        float shininessExponant;// Specular Exponent
        float lightEmission;
        float lightAbsorption; //ocular density
        bool textured;
        Eigen::Vector3f kd; //diffuse light
        Eigen::Vector3f ks; //specular light
        Eigen::Vector3f ka; //ambiant light

        Material(){
            kd = {0.8, 0.8, 0.8};
            ks = {0.2, 0.2, 0.2};
            ka = kd;
            shininessExponant = 25;
            lightEmission = 0;
            opacity = 1;
            lightAbsorption = 0.1;
            textured = false;
            ior = 2;
        }

        float getKR(const Eigen::Vector3f &incidentPoint, const Eigen::Vector3f &normal){
            float cosi = std::clamp(-1.0f, 1.0f, incidentPoint.dot(normal));
            float etai = 1, etat = ior;
            float kr;
            if (cosi > 0) {  std::swap(etai, etat); }
            // Compute sini using Snell's law
            float sint = etai / etat * sqrtf(std::max(0.f, 1 - cosi * cosi));
            // Total internal reflection
            if (sint >= 1) {
                kr = 1;
            }
            else {
                float cost = sqrtf(std::max(0.f, 1 - sint * sint));
                cosi = fabsf(cosi);
                float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
                float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
                kr = (Rs * Rs + Rp * Rp) / 2;
            }
            // As a consequence of the conservation of energy, transmittance is given by:
            // kt = 1 - kr;
            return kr;
        }
};

class Object{
    public:
        std::vector<std::shared_ptr<Face>> faces;
        std::vector<std::shared_ptr<HalfEdge>> halfEdges;
        std::vector<std::shared_ptr<Vertex>> vertices;
        Material material;
};
