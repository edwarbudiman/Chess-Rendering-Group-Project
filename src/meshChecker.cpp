#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <set>
#include <stack>
#include <map>
#include <algorithm>
#include <numeric>
#include <limits>
#include <utility>
#include <random>
#include <fstream>
#include <iomanip>
#include <filesystem>
#include <Eigen/Dense>

// On non-macOS platforms, we can include the GCC-specific header if needed
#if !defined(MACOS_BUILD) && !defined(__APPLE__)
    #include <bits/stdc++.h>
#endif

#include <meshChecker.hpp>
#include <object.hpp>
#include <loadModel.hpp>

using namespace std;
using namespace Eigen;

// Configuration for triangulation method
enum TriangulationMethod {
    SEIDEL_TRIANGULATION,
    FAN_TRIANGULATION,
    EAR_CLIPPING_TRIANGULATION
};

// Global configuration - can be changed to switch triangulation methods
TriangulationMethod g_triangulationMethod = SEIDEL_TRIANGULATION;

float round1e4(float value) {
    return round(value * 10000.0f) / 10000.0f;
  }
Vector3f round1e4(Vector3f value){
    return Vector3f(round1e4(value.x()), round1e4(value.y()), round1e4(value.z()));
}
Vector2f round1e4(Vector2f value){
    return Vector2f(round1e4(value.x()), round1e4(value.y()));
}

bool objectClosed(const Object &object){
    for(const shared_ptr<HalfEdge> &halfEdge : object.halfEdges){
        if(!halfEdge->face){
            return false;
        }
        shared_ptr<HalfEdge> twin = halfEdge->twin.lock();
        if(!twin){
            return false;
        }
        shared_ptr<HalfEdge> twinTwin = twin->twin.lock();
        if(!twinTwin || twinTwin->id != halfEdge->id){
            return false;
        }
    }
    return true;
}

bool objectConnected(const Object &object){
    set<int> visited;
    shared_ptr<Vertex> startingVert = object.vertices[0];
    if(!startingVert){
        return false;
    }

    stack<shared_ptr<Vertex>> vertexStack;
    vertexStack.push(startingVert);

    while (!vertexStack.empty()) {
        shared_ptr<Vertex> vertex = vertexStack.top(); 
        vertexStack.pop();
        if (!vertex || visited.count(vertex->id) > 0){
            continue;
        }
        visited.insert(vertex->id);
        for (const shared_ptr<Vertex> &neighbour : vertex->getNeighbourVertices()) {
            vertexStack.push(neighbour);
        }
    }

    int numVisited = visited.size();
    int numVertices = object.vertices.size();
    if(numVisited == numVertices){
        return true;
    }
    return false;
}

//tries to make object connected by deleting vertices which are not part of a face
void makeObjectConnected(Object &object){
    int startSize = object.vertices.size();
    auto toDelete = remove_if(
        object.vertices.begin(), object.vertices.end(), [](shared_ptr<Vertex> vertex) {return !vertex->halfEdge.lock();});
    object.vertices.erase(toDelete, object.vertices.end());
    int endSize = object.vertices.size();
    int numDeleted = startSize - endSize;
    cout << "\t\tdeleted " << numDeleted << " vertices";
}

bool objectManifold(const Object &object){
    //to check edge manifoldness, make sure their are exactly 2 faces on each edge.
    //first check to make sure only 2 faces share an edge
    vector<vector<int>> numEdgesBetweenVertices(object.vertices.size(), vector<int>(object.vertices.size(), 0));
    for(const shared_ptr<Face> &face : object.faces){
        shared_ptr<HalfEdge> current = face->halfEdge.lock();
        int startID = current->id;
        do{
            if(!current->vertex || !current->next || !current->next->vertex){
                break;
            }
            shared_ptr<HalfEdge> previous = current;
            current = current->next;
            int currentID = current->vertex->id;
            int previousID = previous->vertex->id;
            numEdgesBetweenVertices[previousID][currentID] += 1;
            numEdgesBetweenVertices[currentID][previousID] += 1;
            if(numEdgesBetweenVertices[previousID][currentID] > 2){
                return false;
            }
        }
        while(current->id != startID);
    }
    for(vector<int> start : numEdgesBetweenVertices){
        for(int connections : start){
            if (connections > 0 && connections != 2){
                return false;
            }
        }
    }
    vector<vector<int>> startingPoints(object.vertices.size(), vector<int>());
    //then check if there are any edges with no face
    for(const shared_ptr<HalfEdge> &halfEdge : object.halfEdges){
        if(!halfEdge->face){
            return false;
        }
        int id = halfEdge->vertex->id;
        startingPoints[id].push_back(halfEdge->id);
    }
    //check vertex manifoldness by exploring vertex fans
    int index = 0;
    int maxFan = object.vertices.size();

    for(const shared_ptr<Vertex> &vertex : object.vertices){
        vector<int> currentFan;
        shared_ptr<HalfEdge> halfEdge = vertex->halfEdge.lock();
        shared_ptr<HalfEdge> twin;
        if(!vertex || !halfEdge){
            continue;
        }
        //get a halfedge on a random fan of the current vertex
        //get all halfedges which are part of the fan by traverseing it
        //first forward
        bool fullyExplored = false;
        while(true){
            if(find(currentFan.begin(), currentFan.end(), halfEdge->id) != currentFan.end()){
                fullyExplored = true;
                break;
            }
            currentFan.push_back(halfEdge->id);
            if(currentFan.size() > maxFan){
                return false;
            }
            twin = halfEdge->twin.lock();
            if(!twin || !twin->face || !twin->next){
                break;
            }
            halfEdge = twin->next;
        }

        //then backwards
        halfEdge = vertex->halfEdge.lock()->previous.lock();
        twin = halfEdge->twin.lock();
        if(!fullyExplored && twin && twin->face){
            if(!twin){
                break;
            }
            halfEdge = twin;
            while(true){
                if(find(currentFan.begin(), currentFan.end(), halfEdge->id) != currentFan.end()){
                    break;
                }
                currentFan.push_back(halfEdge->id);
                if(currentFan.size() > maxFan){
                    return false;
                }
                halfEdge = halfEdge->previous.lock();
                twin = halfEdge->twin.lock();
                if(!twin || !twin->face || !twin->next){
                    break;
                }
                halfEdge = twin;
            }
        }
        //if there is a half edge coming from this vertex
        //which has not been found in the fan traversal
        //then the vertex is not manifold
        for(int id : startingPoints[vertex->id]){
            if(find(currentFan.begin(), currentFan.end(), id) == currentFan.end()){
                return false;
            }
        }
    }
    return true;
}

//makes object psuedo manifold by connecting vertex fans even if there is a gap between them
//this allows functions like Face::getHalfEdges() to work
void makeObjectManifold(Object &object){
    vector<vector<shared_ptr<HalfEdge>>> startingPoints(object.vertices.size());
    for(const shared_ptr<HalfEdge> &halfEdge : object.halfEdges){
        int id = halfEdge->vertex->id;
        startingPoints[id].push_back(halfEdge);
    }
    for(const vector<shared_ptr<HalfEdge>> &halfEdges : startingPoints){
        vector<vector<shared_ptr<HalfEdge>>> fans;
        set<int> inFan;
        //get all the fans that come from this vertex
        for(const shared_ptr<HalfEdge> &halfEdge : halfEdges){
            if(inFan.count(halfEdge->id) != 0){
                continue;
            }
            vector<shared_ptr<HalfEdge>> currentFan;
            shared_ptr<HalfEdge> currentHalfEdge = halfEdge;
            shared_ptr<HalfEdge> twin;
            //get all halfedges which are part of this fan by traverseing it
            //first forward
            bool fullyExplored = false;
            while(true){
                if(inFan.count(currentHalfEdge->id) != 0){
                    fullyExplored = true;
                    break;
                }
                currentFan.push_back(currentHalfEdge);
                inFan.insert(currentHalfEdge->id);
                twin = currentHalfEdge->twin.lock();
                if(!twin || !twin->face || !twin->next){
                    break;
                }
                currentHalfEdge = twin->next;
            }

            //then backwards
            currentHalfEdge = halfEdge->previous.lock();
            if(currentHalfEdge){
                twin = currentHalfEdge->twin.lock();
                if(!fullyExplored && twin && twin->face){
                    currentHalfEdge = twin;
                    while(true){
                        if(inFan.count(currentHalfEdge->id) != 0){
                            break;
                        }
                        currentFan.insert(currentFan.begin(), currentHalfEdge);
                        inFan.insert(currentHalfEdge->id);
                        currentHalfEdge = currentHalfEdge->previous.lock();
                        if(!currentHalfEdge){
                            break;
                        }
                        twin = currentHalfEdge->twin.lock();
                        if(!twin || !twin->face || !twin->next){
                            break;
                        }
                        currentHalfEdge = twin;
                    }
                }
            }
            fans.push_back(currentFan);
        }
        //skip if already one fan
        if (fans.size() <= 1){
            continue;
        }
        //connect fans together
        for(int i = 0; i < fans.size(); ++i){
            shared_ptr<HalfEdge> towardsVertex;
            shared_ptr<HalfEdge> awayFromVertex;
            if(i == fans.size() - 1){
                towardsVertex = fans[0].front()->previous.lock();
                awayFromVertex = fans[i].back();
                
            }
            else{
                towardsVertex = fans[i + 1].front()->previous.lock();
                awayFromVertex = fans[i].back();
            }
            if(awayFromVertex){
                awayFromVertex->twin = towardsVertex;
            }
            if(towardsVertex){
                towardsVertex->twin = awayFromVertex;
            }
        }
    }
}

bool objectFacesConsistent(const Object &object){
    //two adjacent polygons have a consistant orientation if touching edges face opposite directions
    //make a bool for each origin vertex - destination vertex combo and set it to false
    vector<vector<bool>> originToDest(object.vertices.size(), vector<bool>(object.vertices.size(), false));
    for(const shared_ptr<HalfEdge> &halfEdge : object.halfEdges){
        int origin = halfEdge->vertex->id;
        int destination = halfEdge->next->vertex->id;
        //if the current half edge's origin and destination have been encountered return false
        if(originToDest[origin][destination]){
            return false;
        }
        //set the current origin and destination combo to true
        originToDest[origin][destination] = true;
    }
    return true;
}

void makeObjectFacesConsistent(Object &object){
    vector<shared_ptr<Face>> consistentFaces;
    vector<shared_ptr<HalfEdge>> consistentHalfEdges;
    set<int> visited;
    int numVertices = object.vertices.size();
    //get a matrix of what faces connect to what edges
    vector<vector<vector<shared_ptr<Face>>>> edges(numVertices, vector<vector<shared_ptr<Face>>>(numVertices));
    vector<shared_ptr<Face>> faces = object.faces;
    for(const shared_ptr<Face> &face : object.faces){
        shared_ptr<HalfEdge> halfEdge = face->halfEdge.lock();
        shared_ptr<HalfEdge> previous;
        int startID = halfEdge->id;
        do{
            previous = halfEdge;
            halfEdge = halfEdge->next;
            int v1 = previous->vertex->id;
            int v2 = halfEdge->vertex->id;
            edges[v1][v2].push_back(face);
            edges[v2][v1].push_back(face);
        }
        while(halfEdge->id != startID);
    }

    //recursion lambda
    auto exploreFaces = [&](const auto &self, shared_ptr<Face> face, bool reverse) -> void {
        if(visited.find(face->id) != visited.end()){
            //if face needs to be reversed but it has already been visited that is a contradiction
            if(reverse){
                throw runtime_error(string("can't make faces consistently faced"));
            }
            //otherwise have reached end of this branch
            return;
        }
        //directions - [from, to]
        vector<vector<int>> directions; 

        vector<shared_ptr<HalfEdge>> clones;
        shared_ptr<HalfEdge> current = face->halfEdge.lock();
        int startID = current->id;
        do {
            HalfEdge newHalfEdge = *current;
            shared_ptr<HalfEdge> newHalfEdgePointer = make_shared<HalfEdge>(newHalfEdge);
            clones.push_back(newHalfEdgePointer);
            consistentHalfEdges.push_back(newHalfEdgePointer);
            current = current->next;
        } 
        while (current->id != startID);

        int n = clones.size();
        for(int i = 0; i < n; ++i){
            clones[i]->twin.lock() = nullptr;
            if(reverse){
                clones[i]->next = clones[(i - 1 + n) % n];
                clones[i]->previous = clones[(i + 1 + n) % n];
            }
            else{
                clones[i]->next = clones[(i + 1) % n];
                clones[i]->previous = clones[(i - 1 + n) % n];
            }
            directions.push_back({clones[i]->vertex->id, clones[i]->next->vertex->id});
        }
        face->halfEdge = clones[0];
        visited.insert(face->id);
        consistentFaces.push_back(face);
        
        for(const vector<int> &directedEdge : directions){
            int from = directedEdge[0];
            int to = directedEdge[1];
            vector<shared_ptr<Face>> adjacent = edges[from][to];
            for(const shared_ptr<Face> &adj : adjacent){
                if(adj->id == face -> id){
                    continue;
                }
                current = adj->halfEdge.lock();
                startID = current->id;
                do{
                    shared_ptr<HalfEdge> previous = current;
                    current = current->next;
                    int adjFrom = previous->vertex->id;
                    int adjTo = current->vertex->id;
                    if(from == adjFrom && to == adjTo){
                        self(self, adj, true);
                        break;
                    }
                    else if(from == adjTo && to == adjFrom){
                        self(self, adj, false);
                        break;
                    }
                }
                while(current->id != startID);
            }
        }
        return;
    };
    //handle disconnected faces by starting in multiple spots
    //(faces will only be consistent with faces they are connected to)
    for(const shared_ptr<Face> &face : faces){
        if(visited.count(face->id) == 0){
            exploreFaces(exploreFaces, face, false);
        }
    }

    for(int i = 0; i < edges.size(); ++i){
        for(int j = i + 1; j < edges.size(); ++j){
            vector<shared_ptr<Face>> currentEdgeFaces = edges[i][j];
            //assume there are only at most 2 entries in currentEdgeFaces
            vector<shared_ptr<HalfEdge>> adjacentHalfEdges;
            int numIterations = 0;
            for(const shared_ptr<Face> &face : currentEdgeFaces){
                numIterations += 1;
                if(numIterations > 2){
                    break;
                }
                shared_ptr<HalfEdge> current = face->halfEdge.lock();
                int startID = current->id;
                do{
                    shared_ptr<HalfEdge> previous = current;
                    current = current->next;
                    int from = previous->vertex->id;
                    int to = current->vertex->id;
                    if((i == from && j == to) || (j == to && i == from)){
                        adjacentHalfEdges.push_back(previous);
                        break;
                    }
                }
                while(current->id != startID);

            }
            if(adjacentHalfEdges.size() == 2){
                adjacentHalfEdges[0]->twin = adjacentHalfEdges[1];
                adjacentHalfEdges[1]->twin = adjacentHalfEdges[0];
            }
        }
    }
    object.faces = consistentFaces;
    object.halfEdges = consistentHalfEdges;
}

bool objectTri(const Object &object){
    for(int i = 0; i < object.faces.size(); ++i){
        if(object.faces[i]->getHalfEdges().size() != 3){
            return false;
        }
    }
    return true;
}

bool objectQuad(const Object &object){
    for(int i = 0; i < object.faces.size(); ++i){
        if(object.faces[i]->getHalfEdges().size() != 4){
            return false;
        }
    }
    return true;
}

namespace seidel{
    struct Point{
        Vector3f position;
        Vector3f colour;
        Vector2f textureCoords;
        Vector3f normal;
        Point(const shared_ptr<Vertex> &vert){
            position = vert->position;
            colour = vert->colour;
            textureCoords = vert->textureCoordinates;
            normal = vert->normal;
        }
        Point(Vector3f _position, Vector3f _colour, Vector2f _textureCoords, Vector3f _normal):
         position(round1e4(_position)), colour(round1e4(_colour)), textureCoords(round1e4(_textureCoords)), normal(round1e4(_normal)){}
        Point(){}
        bool operator==(const Point &other) const{
            auto lhs = tie(
                position.x(), position.y(), position.z(),
                colour.x(),   colour.y(),   colour.z(),
                textureCoords.x(), textureCoords.y(),
                normal.x(),   normal.y(),   normal.z());
            auto rhs = tie(
                other.position.x(), other.position.y(), other.position.z(),
                other.colour.x(),   other.colour.y(),   other.colour.z(),
                other.textureCoords.x(), other.textureCoords.y(),
                other.normal.x(),   other.normal.y(),   other.normal.z());
            return lhs == rhs;
        }
        bool operator<(const Point &other) const{
            auto lhs = tie(
                position.x(), position.y(), position.z(),
                colour.x(),   colour.y(),   colour.z(),
                textureCoords.x(), textureCoords.y(),
                normal.x(),   normal.y(),   normal.z());
            auto rhs = tie(
                other.position.x(), other.position.y(), other.position.z(),
                other.colour.x(),   other.colour.y(),   other.colour.z(),
                other.textureCoords.x(), other.textureCoords.y(),
                other.normal.x(),   other.normal.y(),   other.normal.z());
            return lhs < rhs;
        }
    };

    struct PlaneSegment{
        Vector3f normal;
        Vector3f planePoint;
        shared_ptr<HalfEdge> start;
        shared_ptr<HalfEdge> End;
    };

    Eigen::Vector3f getValue(int key,  std::map<int, Eigen::Vector3f> &rotatedPos){
        auto it = rotatedPos.find(key);
        assert(it != rotatedPos.end());
        return it->second;
    }

    vector<vector<Point>> triangulateQuad(Point A, Point B, Point C, Point D){
        vector<vector<Point>> triangles;
        //maintain orientation
            //check if already triangle
        if(A == B){
            triangles.push_back({B, C, D});
        }
        else if(A == C){
            triangles.push_back({B, C, D});
        }
        else if(A == D){
            triangles.push_back({A, B, C});
        }
        else if(B == C){
            triangles.push_back({D, A, B});
        }
        else if(B == D){
            triangles.push_back({A, B, C});
        }
        else if(C == D){
            triangles.push_back({A, B, C});
        }
        else{
            triangles.push_back({A, B, C});
            triangles.push_back({A, C, D});
        }
        return triangles;
    }    

    Vector3f tieBreak(const Vector3f a, const Vector3f b){
        if(a.y() > b.y()){
            return a;
        }
        else if(a.y() < b.y()){
            return b;
        }
        else{
            if(a.x() > b.x()){
                return a;
            }
            else if(a.x() < b.x()){
                return b;
            }
            else{
                if(a.z() > b.z()){
                    return a;
                }
                else{
                    return b;
                }
            }
        }
    }
    
    shared_ptr<Node> processVertexNode(const shared_ptr<Vertex> &vert, shared_ptr<Node> &tree, vector<shared_ptr<HalfEdge>> faceEdges, int &nodeID, int &trapID, map<int, Vector3f> &rotatedPos){
        if(tree == nullptr){
            //nodeType only root when tree is empty
            shared_ptr<SeidelRay> ray = make_shared<SeidelRay>(vert, faceEdges, rotatedPos);
            tree = make_shared<Node>(nodeID++);
            tree->nodeType = Node::VERTEX;
            tree->vertex = vert;
    
            tree->left = make_shared<Node>(Node(nodeID++));
            tree->left->parent = tree;
            std::shared_ptr<Trapezoid> bottomTrap = make_shared<Trapezoid>(tree->left, nullptr, ray, trapID++);
            tree->left->nodeType = Node::TRAPAZOID;
            tree->left->trapezoid = bottomTrap;
    
            tree->right = make_shared<Node>(nodeID++);
            tree->right->parent = tree;
            std::shared_ptr<Trapezoid> topTrap = make_shared<Trapezoid>(tree->right, ray, nullptr, trapID++);
            tree->right->nodeType = Node::TRAPAZOID;
            tree->right->trapezoid = topTrap;
            topTrap->down.push_back(bottomTrap);
            bottomTrap->up.push_back(topTrap);
            return tree;
        }
        shared_ptr<Node> currentNode = tree;
        while(true){
            if(currentNode->nodeType == Node::VERTEX){
                Vector3f a = getValue(vert->id, rotatedPos);
                Vector3f b = getValue(currentNode->vertex.lock()->id, rotatedPos);
                if(a == b){
                    return currentNode;
                }
                else if(a.y() > b.y() || (a.y() == b.y() && tieBreak(a, b) == a)){
                    currentNode = currentNode->right;
                }
                else{
                    currentNode = currentNode->left;
                }
            }
            else if(currentNode->nodeType == Node::EDGE){
                //need to interpolate to find which side of an edge a vertex is
                shared_ptr<HalfEdge> e = currentNode->edge;
                Vector3f origin = getValue(e->vertex->id, rotatedPos);
                Vector3f dest = getValue(e->next->vertex->id, rotatedPos);
                float yValue = rotatedPos[vert->id].y();
                Vector3f unitVector = (dest - origin).normalized();
                float mult = (yValue - origin.y()) / unitVector.y();
                Vector3f intercept = origin + (unitVector * mult);
                Vector3f a = getValue(vert->id, rotatedPos);
                Vector3f b = intercept;
                    if(a.x() > b.x() || (a.x() == b.x() && tieBreak(a, b) == a)){
                        currentNode = currentNode->right;
                    }
                    else{
                        currentNode = currentNode->left;
                    }
            }
            else{
                //if not vertex or edge must be trapezoid
                shared_ptr<Trapezoid> old = currentNode->trapezoid;
                shared_ptr<SeidelRay> ray = make_shared<SeidelRay>(vert, faceEdges, rotatedPos);
                shared_ptr<SeidelRay> oldHighRay = old->highRay;
                shared_ptr<SeidelRay> oldLowRay = old->lowRay;
                currentNode->nodeType = Node::VERTEX;
                currentNode->trapezoid = nullptr;
                currentNode->vertex = vert;
                currentNode->left = make_shared<Node>(nodeID++);
                currentNode->left->parent = currentNode;
                std::shared_ptr<Trapezoid> bottomTrap = make_shared<Trapezoid>(currentNode->left, oldLowRay, ray, trapID++);
                currentNode->left->nodeType = Node::TRAPAZOID;
                currentNode->left->trapezoid = bottomTrap;
                currentNode->right = make_shared<Node>(nodeID++);
                currentNode->right->parent = currentNode;
                std::shared_ptr<Trapezoid> topTrap = make_shared<Trapezoid>(currentNode->right, ray, oldHighRay, trapID++);
                currentNode->right->nodeType = Node::TRAPAZOID;
                currentNode->right->trapezoid = topTrap;
                topTrap->up = old->up;
                bottomTrap->down = old->down;
                topTrap->down.push_back(bottomTrap);
                bottomTrap->up.push_back(topTrap);
                for(const shared_ptr<Trapezoid> &above : topTrap->up){
                    replace(above->down.begin(), above->down.end(), old, topTrap);
                }
                for(const shared_ptr<Trapezoid> &below : bottomTrap->down){
                    replace(below->up.begin(), below->up.end(), old, bottomTrap);
                }
                return currentNode;
            }
        }
    }
    
    void insertEdge(const shared_ptr<Node> &node, const shared_ptr<HalfEdge> &halfEdge, int &nodeID, int &trapID, map<int, Vector3f> &rotatedPos){
        shared_ptr<Trapezoid> old = node->trapezoid;
        shared_ptr<SeidelRay> highRay = old->highRay;
        shared_ptr<SeidelRay> lowRay = old->lowRay;
        node->nodeType = Node::EDGE;
        node->trapezoid = nullptr;
        node->edge = halfEdge;
        node->left = make_shared<Node>(nodeID++);
        node->left->parent = node;
        std::shared_ptr<Trapezoid> bottomTrap = make_shared<Trapezoid>(node->left, lowRay, highRay, trapID++);
        node->left->nodeType = Node::TRAPAZOID;
        node->left->trapezoid = bottomTrap;
        node->right = make_shared<Node>(Node(nodeID++));
        node->right->parent = node;
        std::shared_ptr<Trapezoid> topTrap = make_shared<Trapezoid>(node->right, lowRay, highRay, trapID++);
        node->right->nodeType = Node::TRAPAZOID;
        node->right->trapezoid = topTrap;
        Vector3f a;
        Vector3f b;
        vector<shared_ptr<Node>> children = {node->left, node->right};
        //get new ups and downs, and set neighbours
        for(const shared_ptr<Trapezoid> &above : old->up){
            auto iter = find(above->down.begin(), above->down.end(), old);
            if ( iter != above->down.end() ) {
                above->down.erase(iter);
            }
            //get lower left above x coord
            float lowerLeftAbove;
            shared_ptr<HalfEdge> lseg = above->lseg;
            if(!lseg){
                lowerLeftAbove = -FLT_MAX;
            }
            else{
                a = getValue(lseg->vertex->id, rotatedPos);
                b = getValue(lseg->next->vertex->id, rotatedPos);
                if(a.y() < b.y() || (a.y() == b.y() && tieBreak(a, b) == a)){
                    lowerLeftAbove = a.x();
                }
                else{
                    lowerLeftAbove = b.x();
                }
            }
    
            //get lower right above x coord
            float lowerRightAbove;
            shared_ptr<HalfEdge> rseg = above->rseg;
            if(!rseg){
                lowerRightAbove = FLT_MAX;
            }
            else{
                a = getValue(rseg->vertex->id, rotatedPos);
                b = getValue(rseg->next->vertex->id, rotatedPos);
                if(a.y() < b.y() || (a.y() == b.y() && tieBreak(a, b) == a)){
                    lowerRightAbove = a.x();
                }
                else{
                    lowerRightAbove = b.x();
                }
            }
            for(const shared_ptr<Node> &child : children){
                //get upper left child x coord
                float upperLeftChild;
                lseg = child->trapezoid->lseg;
                if(!lseg){
                    upperLeftChild = -FLT_MAX;
                }
                else{
                    a = getValue(lseg->vertex->id, rotatedPos);
                    b = getValue(lseg->next->vertex->id, rotatedPos);
                    if(a.y() > b.y() || (a.y() == b.y() && tieBreak(a, b) == a)){
                        upperLeftChild = a.x();
                    }
                    else{
                        upperLeftChild = b.x();
                    }
                }
                //get upper right child x coord
                float upperRightChild;
                rseg = child->trapezoid->rseg;
                if(!rseg){
                    upperRightChild = FLT_MAX;
                }
                else{
                    a = getValue(rseg->vertex->id, rotatedPos);
                    b = getValue(rseg->next->vertex->id, rotatedPos);
                    if(a.y() > b.y() || (a.y() == b.y() && tieBreak(a, b) == a)){
                        upperRightChild = a.x();
                    }
                    else{
                        upperRightChild = b.x();
                    }
                }
                if(lowerLeftAbove < upperRightChild && lowerRightAbove > upperLeftChild){
                    above->down.push_back(child->trapezoid);
                }
            }
        }
        for(const shared_ptr<Trapezoid> &below : old->down){
            auto iter = find(below->up.begin(), below->up.end(), old);
            if ( iter != below->up.end() ) {
                below->up.erase(iter);
            }
            //get upper left below x coord
            float upperLeftBelow;
            shared_ptr<HalfEdge> lseg = below->lseg;
            if(!lseg){
                upperLeftBelow = -FLT_MAX;
            }
            else{
                a = getValue(lseg->vertex->id, rotatedPos);
                b = getValue(lseg->next->vertex->id, rotatedPos);
                if(a.y() > b.y() || (a.y() == b.y() && tieBreak(a, b) == a)){
                    upperLeftBelow = a.x();
                }
                else{
                    upperLeftBelow = b.x();
                }
            }
            //get upper right below x coord
            float upperRightBelow;
            shared_ptr<HalfEdge> rseg = below->rseg;
            if(!rseg){
                upperRightBelow = FLT_MAX;
            }
            else{
                a = getValue(rseg->vertex->id, rotatedPos);
                b = getValue(rseg->next->vertex->id, rotatedPos);
                if(a.y() > b.y() || (a.y() == b.y() && tieBreak(a, b) == a)){
                    upperRightBelow = a.x();
                }
                else{
                    upperRightBelow = b.x();
                }
            }
            for(const shared_ptr<Node> &child : children){
                //get lower left child x coord
                float lowerLeftChild;
                lseg = child->trapezoid->lseg;
                if(!lseg){
                    lowerLeftChild = -FLT_MAX;
                }
                else{
                    a = getValue(lseg->vertex->id, rotatedPos);
                    b = getValue(lseg->next->vertex->id, rotatedPos);
                    if(a.y() < b.y() || (a.y() == b.y() && tieBreak(a, b) == a)){
                        lowerLeftChild = a.x();
                    }
                    else{
                        lowerLeftChild = b.x();
                    }
                }
                //get lower right child x coord
                float lowerRightChild;
                rseg = child->trapezoid->rseg;
                if(!rseg){
                    lowerRightChild = FLT_MAX;
                }
                else{
                    a = getValue(rseg->vertex->id, rotatedPos);
                    b = getValue(rseg->next->vertex->id, rotatedPos);
                    if(a.y() < b.y() || (a.y() == b.y() && tieBreak(a, b) == a)){
                        lowerRightChild = a.x();
                    }
                    else{
                        lowerRightChild = b.x();
                    }
                }
                if(upperLeftBelow < lowerRightChild && upperRightBelow > lowerLeftChild){
                    below->up.push_back(child->trapezoid);
                }
            }
        }
    }
    
    void processEdgeNode(const shared_ptr<HalfEdge> &halfEdge, const shared_ptr<Node> &lower, const shared_ptr<Node> &upper, int &nodeID, int &trapID, map<int, Vector3f> &rotatedPos){
        int placeInserted;
        //insert edge at upper
        shared_ptr<Node> currentNode = upper->left;
        while(true){
            if(currentNode->nodeType == Node::VERTEX){
                currentNode = currentNode->right;
            }
            else if(currentNode->nodeType == Node::EDGE){
                //compare top vertices
                shared_ptr<Vertex> targetTop;
                Vector3f o = getValue(halfEdge->vertex->id, rotatedPos);
                Vector3f n = getValue(halfEdge->next->vertex->id, rotatedPos);
                if(o.y() > n.y() || (o.y() == n.y() && tieBreak(o, n) == o)){
                    targetTop = halfEdge->vertex;
                }
                else{
                    targetTop = halfEdge->next->vertex;
                }
                shared_ptr<Vertex> currentTop;
                shared_ptr<HalfEdge> edge = currentNode->edge;
                o = getValue(edge->vertex->id, rotatedPos);
                n = getValue(edge->next->vertex->id, rotatedPos);
                if(o.y() > n.y() || (o.y() == n.y() && tieBreak(o, n) == o)){
                    currentTop = edge->vertex;
                }
                else{
                    currentTop = edge->next->vertex;
                }
                Vector3f a = getValue(targetTop->id, rotatedPos);
                Vector3f b = getValue(currentTop->id, rotatedPos);
                if(a.x() > b.x() || (a.x() == b.x() && tieBreak(a, b) == a)){
                    currentNode = currentNode->right;
                }
                else{
                    currentNode = currentNode->left;
                }
            }
            else{
                placeInserted = currentNode->id;
                insertEdge(currentNode, halfEdge, nodeID, trapID, rotatedPos);
                break;
            }
        }
        //insert edge at lower
        currentNode = lower->right;
        while(true){
            if(currentNode->id == placeInserted){
                break;
            }
            if(currentNode->nodeType == Node::VERTEX){
                currentNode = currentNode->left;
            }
            else if(currentNode->nodeType == Node::EDGE){
                //compare bottom vertices
                shared_ptr<Vertex> targetBottom;
                Vector3f o = getValue(halfEdge->vertex->id, rotatedPos);
                Vector3f n = getValue(halfEdge->next->vertex->id, rotatedPos);
                if(o.y() < n.y() || (o.y() == n.y() && tieBreak(o, n) == o)){
                    targetBottom = halfEdge->vertex;
                }
                else{
                    targetBottom = halfEdge->next->vertex;
                }
                shared_ptr<Vertex> currentBottom;
                shared_ptr<HalfEdge> edge = currentNode->edge;
                o = getValue(edge->vertex->id, rotatedPos);
                n = getValue(edge->next->vertex->id, rotatedPos);
                if(o.y() < n.y() || (o.y() == n.y() && tieBreak(o, n) == o)){
                    currentBottom = edge->vertex;
                }
                else{
                    currentBottom = edge->next->vertex;
                }
                Vector3f a = getValue(targetBottom->id, rotatedPos);
                Vector3f b = getValue(currentBottom->id, rotatedPos);
                if(a.x() > b.x() || (a.x() == b.x() && tieBreak(a, b) == a)){
                    currentNode = currentNode->right;
                }
                else{
                    currentNode = currentNode->left;
                }
            }
            else{
                insertEdge(currentNode, halfEdge, nodeID, trapID, rotatedPos);
                break;
            }
        }
    }

    inline Point yInterceptFinder(const shared_ptr<Vertex> origin, const shared_ptr<Vertex> dest, float yIntercept, bool fromPX, map<int, Vector3f> &rotatedPos){
        Vector3f rotatedOrigin = getValue(origin->id, rotatedPos);
        Vector3f rotatedDest = getValue(dest->id, rotatedPos);
        Vector3f edgeVector = rotatedDest - rotatedOrigin;
        Vector3f position;
        Vector3f colour;
        Vector2f textureCoords;
        Vector3f normal;
        const float eps = 1e-6f * edgeVector.norm();  
        if (fabs(edgeVector.y()) < eps) {
            if(fromPX){
                if(rotatedOrigin.x() > rotatedDest.x()){
                    position = origin->position;
                    colour = origin->colour;
                    textureCoords = origin->textureCoordinates;
                    normal = origin->normal;
                }
                else{
                    position = dest->position;
                    colour = dest->colour;
                    textureCoords = dest->textureCoordinates;
                    normal = dest->normal;
                }
            }
            else{
                if(rotatedOrigin.x() < rotatedDest.x()){
                    position = origin->position;
                    colour = origin->colour;
                    textureCoords = origin->textureCoordinates;
                    normal = origin->normal;
                }
                else{
                    position = dest->position;
                    colour = dest->colour;
                    textureCoords = dest->textureCoordinates;
                    normal = dest->normal;
                }
            }
        }
        else{
            //in rotated coords
            float mult = (yIntercept - rotatedOrigin.y()) / (edgeVector.y());
            mult = clamp(mult, 0.0f, 1.0f);//then clamp to [0,1]
            
            //in world coords
            position = origin->position + (mult * (dest->position - origin->position));
            colour = (origin->colour * (1 - mult)) + (dest->colour * mult);
            textureCoords = (origin->textureCoordinates * (1 - mult)) + (dest->textureCoordinates * mult);
            normal = (origin->normal * (1 - mult)) + (dest->normal * mult);
        }
        return Point(position, colour, textureCoords, normal);
    }
    
    Matrix4f getRotation(const shared_ptr<Face> &face){
        Vector3f normal = face->normal.normalized();
        static const Vector3f desiredFacing = Vector3f(0, 0, -1);
        float dot = normal.dot(desiredFacing);
        Matrix4f rotation = Matrix4f::Identity();
        if (fabs(dot) < 1 - FLT_EPSILON) {
            Quaternionf q = Quaternionf::FromTwoVectors(normal, desiredFacing);
            Matrix3f rotation3 = q.toRotationMatrix();
            rotation.block<3,3>(0,0) = rotation3;
        }
        // Already within epsilon of ±Z, so just return identity.
        return rotation;
    }

    //Seidel's algorithm for decomposing to trapezoids (quadralaterials)
    vector<vector<Point>> seidel(const shared_ptr<Face> &face){
        vector<vector<Point>> triangles;
        int trapID = 0;
        int nodeID = 0;
        //I use SEIDEL'S algorithm to perform trapezoidation
        //source: http://www.polygontriangulation.com/2018/07/triangulation-algorithm.html
        Matrix4f rotationMatrix = getRotation(face);
        Matrix4f inverseRotation = Matrix4f::Identity();
        inverseRotation.block<3,3>(0,0) = rotationMatrix.block<3,3>(0,0).transpose();

        vector<shared_ptr<HalfEdge>> faceEdges = face->getHalfEdges();
        //randomise edge order
        unsigned seed = chrono::system_clock::now().time_since_epoch().count();
        map<int, Vector3f> rotatedPos;
        shuffle(faceEdges.begin(), faceEdges.end(), default_random_engine(seed));
        for(const shared_ptr<HalfEdge> &halfEdge : faceEdges){
            Vector4f position4f;
            position4f.head(3) = halfEdge->vertex->position;
            position4f[3] = 1;
            position4f = rotationMatrix * position4f;
            rotatedPos[halfEdge->vertex->id] = position4f.head(3);
        }
        shared_ptr<Node> tree;
        //get arbitrary edge from list of edges
        for(const shared_ptr<HalfEdge> &edge : faceEdges){
            //get edge vertices
            shared_ptr<Vertex> v1 = edge->vertex;
            shared_ptr<Vertex> v2 = edge->next->vertex;
            //find higher and lower vertex
            Vector3f a = getValue(v1->id, rotatedPos);
            Vector3f b = getValue(v2->id, rotatedPos);
            shared_ptr<Vertex> higher;
            shared_ptr<Vertex> lower;
            if(a.y() > b.y() || (a.y() == b.y() && tieBreak(a, b) == a)){
                higher = v1;
                lower = v2;
            }
            else{
                higher = v2;
                lower = v1;
            }
            //traverse tree and place higher vertex and lower vertex
            shared_ptr<Node> upperNode = processVertexNode(higher, tree, faceEdges, nodeID, trapID, rotatedPos);
            shared_ptr<Node> lowerNode = processVertexNode(lower, tree, faceEdges, nodeID, trapID, rotatedPos);
            //place edge below upper node but above lower node
            processEdgeNode(edge, lowerNode, upperNode, nodeID, trapID, rotatedPos);
        }
        //get list of trapezoids and make sure their edges are finalised
        //and set their validatiy
        vector<shared_ptr<Trapezoid>> trapezoids;
        stack<shared_ptr<Node>> nodeStack;
        nodeStack.push(tree);
        while(!nodeStack.empty()) {
            shared_ptr<Node> node = nodeStack.top(); 
            nodeStack.pop();
            if(node->nodeType == Node::TRAPAZOID){
                node->trapezoid->setEdges(node);
                node->trapezoid->inPolygon();
                trapezoids.push_back(node->trapezoid);
            }
            else{
                nodeStack.push(node->left);
                nodeStack.push(node->right);
            }
        }
        //transform trapezoids into triangles
        for(const shared_ptr<Trapezoid> &trap : trapezoids){
            if(!trap->validState){
                continue;
            }
            //get vertices of trapezoid
            //use rays and side segments to interporlate the four vertices
            float highY = trap->highRay->yValue;
            float lowY = trap->lowRay->yValue;
            shared_ptr<HalfEdge> lseg = trap->lseg;
            shared_ptr<HalfEdge> rseg = trap->rseg;
            shared_ptr<Vertex> leftO = lseg->vertex;
            shared_ptr<Vertex> leftD = lseg->next->vertex;
            shared_ptr<Vertex> rightO = rseg->vertex;
            shared_ptr<Vertex> rightD = rseg->next->vertex;
            
            Point A = yInterceptFinder(leftO, leftD, lowY, true, rotatedPos);
            Point B = yInterceptFinder(rightO, rightD, lowY, false, rotatedPos);
            Point C = yInterceptFinder(rightO, rightD, highY, false, rotatedPos);
            Point D = yInterceptFinder(leftO, leftD, highY, true, rotatedPos);

            vector<vector<Point>> currentTriangles = triangulateQuad(A, B, C, D);
            triangles.insert(triangles.end(), currentTriangles.begin(), currentTriangles.end());
        }
        return triangles;
    }

    bool facePlanar(PlaneSegment &plane, const shared_ptr<HalfEdge> &halfEdge, set<int> visited = {}){
        shared_ptr<HalfEdge> adjacentHalfEdge;
        Vector3f currentVector = halfEdge->next->vertex->position - halfEdge->vertex->position;
        Vector3f planePoint = halfEdge->vertex->position;
        Vector3f adjacentVector, normal;
        adjacentHalfEdge = halfEdge->next;
        bool checkReverse = false;
        bool faceLine = true;
        //get closest halfedge that hasn't already been visited and is not colinear
        //first look forwards
        do{
            if(visited.count(adjacentHalfEdge->id) == 0){
                //get the equation for the current plane using the half edges as vectors
                adjacentVector = adjacentHalfEdge->next->vertex->position - adjacentHalfEdge->vertex->position;
                normal = currentVector.cross(adjacentVector).normalized();
                if(normal.squaredNorm() > FLT_EPSILON * FLT_EPSILON){
                    faceLine = false;
                    break;
                }
                adjacentHalfEdge = adjacentHalfEdge->next;
            }
            else{
                checkReverse = true;
                break;
            }
        }while(adjacentHalfEdge->id != halfEdge->id);
        //then if havn't found good halfEdge look backwards
        if(checkReverse){
            adjacentHalfEdge = halfEdge->previous.lock();
            do{
                if(visited.count(adjacentHalfEdge->id) == 0){
                    adjacentVector = adjacentHalfEdge->next->vertex->position - adjacentHalfEdge->vertex->position;
                    normal = currentVector.cross(adjacentVector).normalized();
                    if(normal.squaredNorm() > FLT_EPSILON * FLT_EPSILON){
                        faceLine = false;
                        break;
                    }
                    adjacentHalfEdge = adjacentHalfEdge->previous.lock();
                }
                else{
                    throw(string("no way to make face planar"));
                }
            }while(adjacentHalfEdge->id != halfEdge->id);
        }
        //deal with case where face is close to being a line
        if(faceLine){
            plane.End = halfEdge;
            plane.start = halfEdge;
            plane.planePoint = halfEdge->vertex->position;
            Vector3f line = halfEdge->next->vertex->position - halfEdge->vertex->position;
            if(line.cross(Vector3f(1, 0, 0)).squaredNorm() > FLT_EPSILON * FLT_EPSILON){
                plane.normal = line.cross(Vector3f(1, 0, 0)).normalized();
            }
            else{
                plane.normal = line.cross(Vector3f(0, 1, 0)).normalized();
            }
            return true;
        }

        //now that we have the plane we need to find all the start and ends points of the segment
        shared_ptr<HalfEdge> start;
        shared_ptr<HalfEdge> end;
        shared_ptr<HalfEdge> current = halfEdge;
        int currentID = current->id;
        bool fullLoop = true;
        //first find the end of this planar section if it exists
        do{
            end = current;
            Vector3f nextPos = current->next->next->vertex->position;
            if((nextPos - planePoint).dot(normal) > FLT_EPSILON){
                fullLoop = false;
                break;
            }
            current = current->next;
        }while(currentID != current->id);
        if(!fullLoop){
             //then if the section is not closed find the start of this planar section
             current = halfEdge;
             do{
                start = current;
                shared_ptr<HalfEdge> previous = current->previous.lock();
                Vector3f previousPos = previous->vertex->position;
                if((previousPos - planePoint).dot(normal) > FLT_EPSILON){
                    break;
                }
                current = previous;
             }while(currentID != current->id);
        }
        else{
            start = end;
        }
        plane.End = end;
        plane.start = start;
        plane.normal = normal;
        plane.planePoint = halfEdge->vertex->position;
        if(start->id == end->id){
            return true;
        }
        else{
            return false;
        }
    }

    //breaks a face into its planar segments
    vector<shared_ptr<Face>> planarDecompose(const shared_ptr<Face> &face, int &oldFaceID){
        //first check if face is planar
        vector<shared_ptr<Vertex>> vertices = face->getVertices();
        Vector3f pNormal = (vertices[1]->position - vertices[0]->position).cross(vertices[2]->position - vertices[0]->position);
        bool planar = true;
        for(int i = 3; i < vertices.size(); ++i){
            if((vertices[i]->position - vertices[0]->position).dot(pNormal) > FLT_EPSILON){
                planar = false;
                break;
            }
        }
        //if face is already planar just return the face
        PlaneSegment t;
        if(facePlanar(t, face->halfEdge.lock())){
            return {face};
        }
        //get max halfEdgeid number
        int halfEdgeID = 0;
        for(const shared_ptr<HalfEdge> &halfEdge : face->getHalfEdges()){
            int tempID = halfEdge->id;
            if(tempID > halfEdgeID){
                halfEdgeID = tempID;
            }
        }
        halfEdgeID += 1;

        set<int> visited;
        stack<shared_ptr<HalfEdge>> halfEdgeStack;
        vector<shared_ptr<Face>> planarFaces;
        while(!halfEdgeStack.empty()){
            shared_ptr<HalfEdge> currentHalfEdge = halfEdgeStack.top();
            halfEdgeStack.pop();
            //if already done this halfEdge continue
            if(visited.count(currentHalfEdge->id) != 0){
                continue;
            }
            visited.insert(currentHalfEdge->id); 
            PlaneSegment currentPlane;
            bool restPlanar = facePlanar(currentPlane, currentHalfEdge, visited);
                       
            if(!restPlanar){
                //then if the section is not closed find the start of this planar section
                shared_ptr<HalfEdge> remainderStart = currentPlane.End->next;
                shared_ptr<HalfEdge> remainderEnd = currentPlane.start->previous.lock();
                //close new face and old face
                shared_ptr<HalfEdge> newPlanarHalfEdge = make_shared<HalfEdge>(HalfEdge(halfEdgeID++));
                shared_ptr<HalfEdge> newRemainderHalfEdge = make_shared<HalfEdge>(HalfEdge(halfEdgeID++));
                //make planar half edge
                newPlanarHalfEdge->previous = currentPlane.End;
                newPlanarHalfEdge->next = currentPlane.start;
                newPlanarHalfEdge->vertex = remainderStart->vertex;
                currentPlane.End->next = newPlanarHalfEdge;
                currentPlane.start->previous = newPlanarHalfEdge;
                newPlanarHalfEdge->twin = newRemainderHalfEdge;
                //make half edge for remainder of old face
                newRemainderHalfEdge->previous = remainderEnd;
                newRemainderHalfEdge->next = remainderStart;
                newRemainderHalfEdge->vertex = currentPlane.start->vertex;
                remainderEnd->next = newRemainderHalfEdge;
                remainderStart->previous = newRemainderHalfEdge;
                newRemainderHalfEdge->twin = newPlanarHalfEdge;

                //add remainder start and remainder end to stack
                halfEdgeStack.push(remainderStart);
                halfEdgeStack.push(remainderEnd);
            }
            shared_ptr<Face> planarFace = make_shared<Face>(oldFaceID++);
            planarFace->normal = currentPlane.normal;
            planarFace->halfEdge = currentHalfEdge;
            shared_ptr<HalfEdge> current = currentPlane.start;
            int startID = current->id;
            do{
                current->face = planarFace;
                current = current->next;
            }while(current->id != startID);
        }
        return planarFaces;
    }

    inline Vector3f xInterceptFinder(Vector3f origin, Vector3f dest, float xIntercept){
        float dx = dest.x() - origin.x();
        if (std::fabs(dx) < FLT_EPSILON){
            //not quite accurate, but close enough for edge case
            return origin;
        }
        float mult = (xIntercept - origin.x()) / (dest.x() - origin.x());
        return origin + (mult * (dest - origin));
    }

    //checks if a given face has a self intersection and returns a list of the points they occur
    //and the edges they occur on
    vector<pair<vector<int>, Vector3f>> selfIntersectChecker(const shared_ptr<Face> &face){
        struct Edge{
            int id, nextID;
            Vector3f a, b;
            float minX, maxX;
            Edge(int _id, int _nID, Vector3f _a, Vector3f _b): id(_id), nextID(_nID), a(_a), b(_b){
                minX = min({a.x(), b.x()});
                maxX = max({a.x(), b.x()});
            }
            bool operator<(Edge const& o) const { 
                return minX < o.minX;
              }
        };
        //using alogrithm found here:
        //https://www.wyzant.com/resources/answers/696697/check-if-polygon-is-self-intersecting
        //with some adjustments for efficency

        vector<pair<vector<int>, Vector3f>> selfInteceptPoints;
        //intersection can only occur with 4 or more vertices
        if(face->getVertices().size() < 4){
            return selfInteceptPoints;
        }
        //make Edges
        vector<Edge> edges;
        for(const shared_ptr<HalfEdge> &halfEdge: face->getHalfEdges()){
            edges.push_back(Edge(halfEdge->id, halfEdge->next->id, halfEdge->vertex->position, halfEdge->next->vertex->position));
        }
        sort(edges.begin(), edges.end(),
              [](auto const& e1, auto const& e2){
                  return e1.minX < e2.minX;
              });

        // active set sorted by maxX
        set<Edge> active(edges.begin(), edges.end());
        
        for (const Edge &edge : edges) {
            // Evict edges that end before this one starts
            vector<Edge> toErase;
            for(const Edge &aEdge : active) {
                if(aEdge.maxX < edge.minX){
                    toErase.push_back(aEdge);
                }
            }
            for(const Edge &eEdge : toErase){
                active.erase(eEdge);
            }
            // Test edge against everything left in active
            for (const Edge &aEdge : active) {
                //check to make sure edges aren't neighbours
                if(edge.id == aEdge.nextID || edge.nextID == edge.id || edge.id == aEdge.id){
                    continue;
                }
                if (aEdge.minX > edge.maxX) {
                    continue;
                }

                float lowerMax = max({edge.minX, aEdge.minX});
                float upperMin = min({edge.maxX, aEdge.maxX});

                float edgeDx = edge.b.x() - edge.a.x();
                float edgeDy = edge.b.y() - edge.a.y();
                float aEdgeDx = aEdge.b.x() - aEdge.a.x();
                float aEdgeDy = aEdge.b.y() - aEdge.a.y();
                float m1 = edgeDy / edgeDx;
                float m2 = aEdgeDy / aEdgeDx;
                float x;
                //first find at what x coord a possible intersection would have to be
                if(edgeDx == 0){
                    //if one of the sides is straight up and down the x intersection is given by that side
                    x = edge.a.x();
                }
                else if(aEdgeDx == 0){
                    //if one of the sides is straight up and down the x intersection is given by that side
                    x = aEdge.a.x();
                }
                else if(m1 == m2){
                    //if the lines are parrallel the x can be any x between lowerMax and upperMin
                    float difference = upperMin - lowerMax;
                        x = lowerMax + (0.5 * difference);
                }
                else{
                    //otherwise we know the lines are not parrellel so we just need to solve for x
                    //using equations:
                    // y = (m1 * x) + b1
                    // y = (m2 * x) + b2
                    //where m = rise / run and b can be found by subsituting 1 end of each side
                    //we can solve for x using subsitution
                    float b1 = edge.a.y() - (m1 * edge.a.x());
                    float b2 = aEdge.a.y() - (m2 * aEdge.a.x());
                    x = (b1 - b2) / (m1 - m2);
                }
                Vector3f s1Intercept = xInterceptFinder(edge.a, edge.b, x);
                Vector3f s2Intercept = xInterceptFinder(aEdge.a, aEdge.b, x);
                //just need to check to see if intercepts are the same to see if there is a self intersection here
                if((s1Intercept - s2Intercept).norm() <= FLT_EPSILON){
                    selfInteceptPoints.push_back({{edge.id, aEdge.id}, s1Intercept});
                }
            }
            active.insert(edge);
        }
        return selfInteceptPoints;
    }

    float polygonArea(const shared_ptr<Face> &face, Matrix4f rotation){
        vector<Vector2f> points;
        for(const shared_ptr<HalfEdge> halfEdge : face->getHalfEdges()){
            Vector4f temp;
            temp.head(3) = halfEdge->vertex->position;
            temp[3] = 1;
            temp = rotation * temp;
            points.push_back(Vector2f(temp.x(), temp.y()));
        }
        int divid = points.size();
        float a1 = 0;
        float a2 = 0;
        for(int i = 0; i < divid; ++i){
            a1 += points[i].x() * points[(i + 1) % divid].y();
            a2 += points[i].y() * points[(i + 1) % divid].x();
        }
        float area = 0.5 * fabs(a1 - a2);
        return area;
    }

    float polygonArea(const vector<Point> &face, Matrix4f rotation){
        vector<Vector2f> points;
        for(const Point  &vertex : face){
            Vector4f temp;
            temp.head(3) = vertex.position;
            temp[3] = 1;
            temp = rotation * temp;
            points.push_back(Vector2f(temp.x(), temp.y()));
        }
        int divid = points.size();
        float a1 = 0;
        float a2 = 0;
        for(int i = 0; i < divid; ++i){
            a1 += points[i].x() * points[(i + 1) % divid].y();
            a2 += points[i].y() * points[(i + 1) % divid].x();
        }
        float area = 0.5 * fabs(a1 - a2);
        return area;
    }

    // Fan triangulation - simple and fast alternative to Seidel
    vector<vector<Point>> fanTriangulate(const shared_ptr<Face> &face){
        vector<vector<Point>> triangles;
        auto vertices = face->getVertices();
        
        if(vertices.size() < 3) return triangles; // Invalid face
        
        // Fast path for triangles - same as Seidel optimization
        if(vertices.size() == 3) {
            Point A = Point(vertices[0]);
            Point B = Point(vertices[1]);
            Point C = Point(vertices[2]);
            triangles.push_back({A, B, C});
            return triangles;
        }
        
        // Fast path for quads - same as Seidel optimization
        if(vertices.size() == 4) {
            Point A = Point(vertices[0]);
            Point B = Point(vertices[1]);
            Point C = Point(vertices[2]);
            Point D = Point(vertices[3]);
            triangles = triangulateQuad(A, B, C, D);
            return triangles;
        }
        
        // Fan triangulation for complex polygons: connect vertex 0 to all non-adjacent vertices
        for(size_t i = 1; i < vertices.size() - 1; i++) {
            Point A = Point(vertices[0]);
            Point B = Point(vertices[i]);
            Point C = Point(vertices[i + 1]);
            triangles.push_back({A, B, C});
        }
        return triangles;
    }

    // Ear clipping triangulation - good balance between simplicity and robustness
    vector<vector<Point>> earClippingTriangulate(const shared_ptr<Face> &face){
        vector<vector<Point>> triangles;
        auto vertices = face->getVertices();
        
        if(vertices.size() < 3) return triangles; // Invalid face
        
        // Fast path for triangles
        if(vertices.size() == 3) {
            Point A = Point(vertices[0]);
            Point B = Point(vertices[1]);
            Point C = Point(vertices[2]);
            triangles.push_back({A, B, C});
            return triangles;
        }
        
        // Fast path for quads
        if(vertices.size() == 4) {
            Point A = Point(vertices[0]);
            Point B = Point(vertices[1]);
            Point C = Point(vertices[2]);
            Point D = Point(vertices[3]);
            triangles = triangulateQuad(A, B, C, D);
            return triangles;
        }
        
        // Ear clipping for complex polygons
        vector<Point> polygon;
        for(auto vertex : vertices) {
            polygon.push_back(Point(vertex));
        }
        
        // Helper functions for ear clipping
        auto isConvex = [](const Vector3f& a, const Vector3f& b, const Vector3f& c) -> bool {
            Vector3f v1 = b - a;
            Vector3f v2 = c - b;
            Vector3f cross = v1.cross(v2);
            return cross.z() > 0; // Assuming CCW winding
        };
        
        auto pointInTriangle = [](const Vector3f& p, const Vector3f& a, const Vector3f& b, const Vector3f& c) -> bool {
            Vector3f v0 = c - a;
            Vector3f v1 = b - a;
            Vector3f v2 = p - a;
            
            float dot00 = v0.dot(v0);
            float dot01 = v0.dot(v1);
            float dot02 = v0.dot(v2);
            float dot11 = v1.dot(v1);
            float dot12 = v1.dot(v2);
            
            float invDenom = 1 / (dot00 * dot11 - dot01 * dot01);
            float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
            float v = (dot00 * dot12 - dot01 * dot02) * invDenom;
            
            return (u >= 0) && (v >= 0) && (u + v <= 1);
        };
        
        auto isEar = [&](int i, const vector<Point>& poly) -> bool {
            int n = poly.size();
            int prev = (i - 1 + n) % n;
            int next = (i + 1) % n;
            
            // Check if ear is convex
            if (!isConvex(poly[prev].position, poly[i].position, poly[next].position)) {
                return false;
            }
            
            // Check if any other vertex is inside the ear triangle
            for (int j = 0; j < n; j++) {
                if (j == prev || j == i || j == next) continue;
                if (pointInTriangle(poly[j].position, poly[prev].position, poly[i].position, poly[next].position)) {
                    return false;
                }
            }
            return true;
        };
        
        // Ear clipping algorithm
        while(polygon.size() > 3) {
            bool earFound = false;
            for(int i = 0; i < polygon.size(); i++) {
                if(isEar(i, polygon)) {
                    int prev = (i - 1 + polygon.size()) % polygon.size();
                    int next = (i + 1) % polygon.size();
                    
                    // Add triangle
                    triangles.push_back({polygon[prev], polygon[i], polygon[next]});
                    
                    // Remove ear vertex
                    polygon.erase(polygon.begin() + i);
                    earFound = true;
                    break;
                }
            }
            
            // Fallback to prevent infinite loop
            if(!earFound) {
                // Use fan triangulation as fallback
                for(size_t i = 1; i < polygon.size() - 1; i++) {
                    triangles.push_back({polygon[0], polygon[i], polygon[i + 1]});
                }
                break;
            }
        }
        
        // Add final triangle
        if(polygon.size() == 3) {
            triangles.push_back({polygon[0], polygon[1], polygon[2]});
        }
        
        return triangles;
    }

    vector<vector<Point>> triangulate(const shared_ptr<Face> &face, int &oldFaceID){
        vector<vector<Point>> triangles;
        //check how many sides current face has
        shared_ptr<HalfEdge> HalfEdge = face->halfEdge.lock();
        auto current = HalfEdge;
        int startID = current->id;
        int numEdges = 0;
        do{
            current = current->next;
            numEdges += 1;
        }
        while(current->id != startID);
        vector<pair<vector<int>, Vector3f>> intersections = selfIntersectChecker(face);
        //if it is already a triangle, return unchanged
        if(numEdges == 3){
            Point A = Point(HalfEdge->vertex);
            Point B = Point(HalfEdge->next->vertex);
            Point C = Point(HalfEdge->next->next->vertex);
            triangles.push_back({A, B, C});
        }
        //if it is a quad and has no self intersections use basic quad triangulation
        else if (numEdges == 4 && intersections.size() == 0){
            //for left and right edges all that matters is that they are opposite edges actual orientation doesn't matter
            Point A = Point(HalfEdge->vertex);
            Point B = Point(HalfEdge->next->vertex);
            Point C = Point(HalfEdge->next->next->vertex);
            Point D = Point(HalfEdge->next->next->next->vertex);
            triangles = triangulateQuad(A, B, C, D);
        }
        //else use Seidel's
        else{
            //first break face into planar faces
            for(const shared_ptr<Face> &planarFace : planarDecompose(face, oldFaceID)){
                //then triangulate planar faces
                vector<vector<Point>> temp = seidel(planarFace);
                triangles.insert(triangles.end(), temp.begin(), temp.end());
            }
        }
        vector<vector<Point>> goodTriangles;
        for(vector<Point> tri : triangles){
            if(tri[0] == tri[1] || tri[0] == tri[2] || tri[1] == tri[2]){
                continue;
            }
            goodTriangles.push_back(tri);
        }
        return goodTriangles;
    }
}

void makeObjectTri(Object &object){
    vector<shared_ptr<Vertex>> vertices;
    vector<shared_ptr<Face>> faces;
    vector<shared_ptr<HalfEdge>> halfEdges;
    map<seidel::Point, int> vertexIndex;
    map<int, seidel::Point> indexVertex;
    int vertexID, faceID, halfEdgeID;
    vertexID = faceID = halfEdgeID = 0;
    map<int, vector<shared_ptr<HalfEdge>>> halfEdgesByDestination;

    ///get max Faceid number
    int oldFaceID = 0;
    for(const shared_ptr<Face> &face : object.faces){
        int tempID = face->id;
        if(tempID > oldFaceID){
            oldFaceID = tempID;
        }
    }
    oldFaceID += 1;

    auto makeFaceTri = [&](const shared_ptr<Face> face) -> void{
        vector<vector<seidel::Point>> faceTriangles;
        
        // Choose triangulation method based on configuration
        if (g_triangulationMethod == SEIDEL_TRIANGULATION) {
            faceTriangles = seidel::triangulate(face, oldFaceID);
        } else if (g_triangulationMethod == FAN_TRIANGULATION) {
            faceTriangles = seidel::fanTriangulate(face);
        } else if (g_triangulationMethod == EAR_CLIPPING_TRIANGULATION) {
            faceTriangles = seidel::earClippingTriangulate(face);
        }
        for(const vector<seidel::Point> &triangle : faceTriangles){
            for(const seidel::Point &point : triangle){
                if(vertexIndex.count(point) == 0){
                    vertexIndex.insert({point, vertexID});
                    vertexID += 1;
                }
            }
        }
        vertices.resize(vertexIndex.size());

        for(const vector<seidel::Point> &faceCoords : faceTriangles){
            shared_ptr<Face> face = make_shared<Face>(Face(faceID++));
            shared_ptr<HalfEdge> previous = nullptr;
            shared_ptr<Vertex> firstVertex;
            shared_ptr<HalfEdge> firstHalfEdge;
            for(const seidel::Point &point: faceCoords){
                int vertexNum =vertexIndex[point];
                shared_ptr<HalfEdge> halfEdge = make_shared<HalfEdge>(HalfEdge(halfEdgeID++));
                if(previous != nullptr){
                    previous->next = halfEdge;
                    halfEdge->previous = previous;
                    if (halfEdgesByDestination.find(vertexNum) == halfEdgesByDestination.end()){
                        halfEdgesByDestination.insert({vertexNum, vector<shared_ptr<HalfEdge>>()});
                    }
                    halfEdgesByDestination[vertexNum].push_back(previous);
                }
                else{
                    firstHalfEdge = halfEdge;
                }
                shared_ptr<Vertex> vertex;
                if(vertices[vertexNum] == nullptr){
                    vertex = make_shared<Vertex>(Vertex(vertexNum));
                    vertex->position = point.position;
                    vertex->colour = point.colour;
                    vertex->textureCoordinates = point.textureCoords;
                    vertex->normal = point.normal;
                    vertex->halfEdge = halfEdge;
                    vertices[vertexNum] = vertex;
                }
                else{
                    vertex = vertices[vertexNum];
                }
                halfEdge->vertex = vertex;
                halfEdge->face = face;
                halfEdges.push_back(halfEdge);
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
            faces.push_back(face);
        }
    };
    for(int i = 0; i < object.faces.size(); ++i){
        makeFaceTri(object.faces[i]);
    }
    for(const shared_ptr<HalfEdge> &halfEdge : halfEdges){
        shared_ptr<HalfEdge> twin = halfEdge->twin.lock();
        if(!twin){
            int origin = halfEdge->vertex->id;
            int destination = halfEdge->next->vertex->id;
            for(const shared_ptr<HalfEdge> &twin : halfEdgesByDestination[origin]){
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
    for(int i = 0; i < vertices.size(); ++i){
        if(vertices[i] == nullptr){
            vertices[i] = make_shared<Vertex>(i);
            vertices[i]->position = indexVertex[i].position;
            vertices[i]->colour = indexVertex[i].colour;
            vertices[i]->textureCoordinates = indexVertex[i].textureCoords;
        }
    }
    object.vertices = vertices;
    object.faces = faces;
    object.halfEdges = halfEdges;
}

// Function to set triangulation method
void setTriangulationMethod(int method) {
    if (method == 0) g_triangulationMethod = SEIDEL_TRIANGULATION;
    else if (method == 1) g_triangulationMethod = FAN_TRIANGULATION;
    else if (method == 2) g_triangulationMethod = EAR_CLIPPING_TRIANGULATION;
    else g_triangulationMethod = SEIDEL_TRIANGULATION; // default
    
    cout << "Triangulation method set to: " << getTriangulationMethod() << "\n";
}

// Function to get current triangulation method
string getTriangulationMethod() {
    switch(g_triangulationMethod) {
        case SEIDEL_TRIANGULATION: return "Seidel";
        case FAN_TRIANGULATION: return "Fan";
        case EAR_CLIPPING_TRIANGULATION: return "Ear Clipping";
        default: return "Unknown";
    }
}

void checkMesh(Object &object, const string &fileName){
    cout << "checking characteristics of " << fileName << "\n";
    cout << "\tchecking consistency";
    if(!objectFacesConsistent){
        cout << " - not consistent\n";
        cout << "\t\tmaking object consistently faced: " << flush;
        makeObjectFacesConsistent(object);
        cout << "success\n";
    }
    else{
        cout << " - consistent\n";
    }
    cout << "\tchecking if object tri";
    if(!objectTri(object)){
        cout << " - not tri\n";
        if(objectQuad(object)){
            cout << "\t\tobject quad\n";
        }
        cout << "\t\tmaking object triangle mesh: " << flush;
        makeObjectTri(object);
        cout << "success\n";
    }
    else{
        cout << " - tri\n";
    }
    cout << "\tchecking connected";
    if(!objectConnected){
        cout << " - not connected\n";
        cout << "\t\tmaking object connected: " << flush;
        makeObjectConnected(object);
        cout << "success\n";
    }
    else{
        cout << " - connected\n";
    }
    cout << "\tchecking manifold";
    if(!objectManifold){
        cout << " - not manifold\n";
        cout << "\t\tmaking object manifold: " << flush;
        makeObjectManifold(object);
        cout << "success\n";
    }
    else{
        cout << " - manifold\n";
    }
    cout << "\tchecking closed";
    if(!objectClosed){
        cout << " - not closed\n";
    }
    else{
        cout << " - closed\n";
    }
}

struct LexographicLess {
    template<class T>
    bool operator()(T const& lhs, T const& rhs) const {
      return lexicographical_compare(lhs.begin(), lhs.end(), rhs.begin(), rhs.end());
    }
};

bool saveMeshASOBJ(const Object &object, string fileNameQualifier){
    int positionCounter = 1;
    int textureCounter = 1;
    int normalCounter = 1;
    map<int, Vector3f> indexToPosition;
    map<int, Vector2f> indexToTexture;
    map<int, Vector3f> indexToNormal;
    map<Vector3f, int, LexographicLess> positionToIndex;
    map<Vector2f, int, LexographicLess> textureToIndex;
    map<Vector3f, int, LexographicLess> normalToIndex;

    for(const shared_ptr<Vertex> &vertex : object.vertices){
        if(positionToIndex.find(vertex->position) == positionToIndex.end()){
            positionToIndex[vertex->position] = positionCounter;
            indexToPosition[positionCounter] = vertex->position;
            positionCounter += 1;
        }
        if(textureToIndex.find(vertex->textureCoordinates) == textureToIndex.end()){
            textureToIndex[vertex->textureCoordinates] = textureCounter;
            indexToTexture[textureCounter] = vertex->textureCoordinates;
            textureCounter += 1;
        }
        if(normalToIndex.find(vertex->normal) == normalToIndex.end()){
            normalToIndex[vertex->normal] = normalCounter;
            indexToNormal[normalCounter] = vertex->normal;
            normalCounter += 1;
        }
    }
    //make new file name
    filesystem::path p(object.objFile);
    filesystem::path filePath = p.parent_path() / (fileNameQualifier + p.filename().string());
    ofstream outFile(filePath);
    if(!outFile.is_open()){
        cerr << "Failed to open file: " << filePath << "\n";
        return false;
    }
    outFile << "mtllib " << object.material.mtlFile << "\n";
    outFile << "\n\n";

    int precision = 4;
    outFile << fixed << setprecision(precision);

    //write the vertices
    int numVertices = 0;
    for(auto indexPosition : indexToPosition){
        numVertices += 1;
        Vector3f position = indexPosition.second;
        outFile << "v " << position.x() << " " << position.y() << " " << position.z() << "\n";
    }
    for(auto indexNormal : indexToNormal){
        Vector3f normal = indexNormal.second;
        outFile << "vn " << normal.x() << " " << normal.y() << " " << normal.z() << "\n";
    }
    for(auto indexTexture : indexToTexture){
        Vector2f texture = indexTexture.second;
        outFile << "vt " << texture.x() << " " << texture.y() << " 0.0\n";
    }
    outFile << "\n# num vertices: " << numVertices << "\n";
    //add some space
    outFile << "\n\n";
    //then the mtl file
    outFile << "usemtl " << filesystem::path(object.material.mtlFile).stem().string() << "\n";
    //then some more space
    outFile << "\n\n";

    //then add the faces
    for(const shared_ptr<Face> &face : object.faces){
        outFile << "f";
        shared_ptr<HalfEdge> startHalfEdge = face->halfEdge.lock();
        if(!startHalfEdge){
            cerr << "Warning: face with expired halfEdge, skipping\n";
            continue;
        }
        shared_ptr<HalfEdge> current = startHalfEdge;
        do{
            shared_ptr<Vertex> vertex = current->vertex;
            int posIndex = positionToIndex[vertex->position];
            int texIndex = textureToIndex[vertex->textureCoordinates];
            int normIndex = normalToIndex[vertex->normal];
            outFile << " " << posIndex << "/" << texIndex << "/" << normIndex;
            current = current->next;
        }while(current != startHalfEdge);
        outFile << "\n";
    }
    outFile << "\n# num faces: " << object.faces.size();
    return true;
}

int main(int argc, char** argv){
    string fileString;
    if(argc >= 2){
        fileString = argv[1];
    }
    filesystem::path filePath = fileString;
    if(filePath.extension() != ".obj"){
        throw runtime_error("not an obj file");
    }
    Mesh mesh;
    Object object;
    try{
        cout << "running\n";
        cout << "\tloading " << filePath << "\n";
        mesh = loadFile(filePath);
        cout << "converting mesh to halfEdge\n";
        object = meshToHalfEdge(mesh);
        object.objFile = filePath;
        cout << "saving mesh for testing\n";
        saveMeshASOBJ(object, "beforeCheck_");
        checkMesh(object, filePath);
        cout << "saving new obj\n";
        saveMeshASOBJ(object, "afterCheck_");
        cout << "done\n";
    }
    catch(string message){
        cerr << "exception message: " << message << "\n";
    }
}