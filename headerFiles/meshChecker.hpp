#pragma once
#include <object.hpp>
#include <cfloat>

bool objectClosed(const Object &object);

Object makeObjectClosed(Object object);

bool facesClosed(Object object);

bool objectConnected(Object object);

Object makeObjectConnected(Object object);

bool objectManifold(Object object);

Object makeObjectManifold(Object object);

bool objectTri(Object object);

Object makeObjectTri(Object object);

class Node;
class Trapezoid;
class SeidelRay;

class Node{
    public:
        enum NodeType{
            ROOT,
            VERTEX,
            EDGE,
            TRAPAZOID,
        };
        enum NodeType nodeType;
        std::weak_ptr<Vertex> vertex; //the vertex this represents if nodetype is vertex
        std::weak_ptr<HalfEdge> edge; //the edge this represents if nodetype is edge
        std::shared_ptr<Trapezoid> trapezoid; //the trapezoid this represents if nodetype is trapezoid
        const int id; //unique node id
        std::shared_ptr<Node> left; //left child node
        std::shared_ptr<Node> right; //right child node
        std::weak_ptr<Node> parent; //parent node
        Node(const int _id): id(_id){}
};

class Trapezoid{
    public:
        std::vector<std::shared_ptr<Trapezoid>> up; //trapezoids directly above this one
        std::vector<std::shared_ptr<Trapezoid>> down; //trapezoids directly below this one
        std::weak_ptr<HalfEdge> lseg; //left edge
        std::weak_ptr<HalfEdge> rseg; //right edge
        std::weak_ptr<Node> sink; //Position of trapezoid in tree structure
        bool validState; //Represents validity of trapezoid (Inside or outside)
        const int id; //unique ID for this Trapezoid
        std::shared_ptr<SeidelRay> highRay; //the ray which bounds the top of the trapezoid
        std::shared_ptr<SeidelRay> lowRay; //the ray which bounds the bottom of the trapezoid
        Trapezoid(std::weak_ptr<Node> weakNode, std::shared_ptr<SeidelRay> lowRay, std::shared_ptr<SeidelRay> highRay, int id): id(id){
            this->sink = weakNode.lock()->parent;
            this->highRay = highRay;
            this->lowRay = lowRay;
            setLeftEdge(weakNode);
            setRightEdge(weakNode);
        }
        void setLeftEdge(const std::weak_ptr<Node> &weakNode){
            std::shared_ptr<Node> node = weakNode.lock();
            int previous = node->id;
            std::shared_ptr<Node> current = node->parent.lock();
            std::weak_ptr<HalfEdge> edge;
            if(!current){
                return;
            }
            while(true){
                if(current->nodeType == Node::EDGE){
                    if(current->left == nullptr || current->left->id != previous){
                        edge = current->edge;
                        break;
                    }
                }
                previous = current->id;
                current = current->parent.lock();
                if(!current){
                    return;
                }
            };
            this->lseg = edge;
        }
        void setRightEdge(const std::weak_ptr<Node> &weakNode){
            std::shared_ptr<Node> node = weakNode.lock();
            int previous = node->id;
            std::shared_ptr<Node> current = node->parent.lock();
            std::weak_ptr<HalfEdge> edge;
            if(!current){
                return;
            }
            while(true){
                if(current->nodeType == Node::EDGE){
                    if(current->right == nullptr || current->right->id != previous){
                        edge = current->edge;
                        break;
                    }
                }
                previous = current->id;
                current = current->parent.lock();
                if(!current){
                    return;
                }
            };
            this->rseg = edge;
        }
        bool inPolygon(){
            if(lseg.lock() == nullptr || rseg.lock() == nullptr){
                validState = false;
                return false;
            }
            //get current node from trapezoid
            std::shared_ptr<Node> currentNode = sink.lock();
            //traverse upward until left or right edge found
            bool directionLeft;
            while(true){
                int previous = currentNode->id;
                currentNode = currentNode->parent.lock();
                if(currentNode == nullptr){
                    return false;
                }
                if(currentNode->nodeType == Node::EDGE){
                    if(currentNode->right != nullptr && currentNode->right->id == previous){
                        directionLeft = true;
                    }
                    else{
                        directionLeft = false;
                    }
                    break;
                }
            }
            int counter = 0;
            findAdjacentTrapezoid(currentNode, directionLeft, counter);
            if(counter % 2 == 0){
                validState = false;
                return false;
            }
            else{
                validState = true;
                return true;
            }
        }
    private:
        void findAdjacentTrapezoid(std::shared_ptr<Node> currentNode, bool directionLeft, int &counter){
            bool subDirectionLeft = true;
            //get node according to direction
            if(directionLeft){
                currentNode = currentNode->left;
            }
            else{
                currentNode = currentNode->right;
            }
            //find the closest trapezoid from that node
            while(true){
                if(currentNode->nodeType == Node::TRAPAZOID){
                    counter += 1;
                    //check if tree outside polygon
                    if(currentNode->trapezoid->lseg.lock() && currentNode->trapezoid->rseg.lock()){
                        //get segment according to direction
                        while(true){
                            int previous = currentNode->id;
                            currentNode = currentNode->parent.lock();
                            if(!currentNode){
                                return;
                            }
                            if(currentNode->nodeType == Node::EDGE){
                                if(directionLeft && currentNode->right && currentNode->right->id == previous){
                                    break;
                                }
                                else if(!directionLeft && currentNode->left && currentNode->left->id == previous){
                                    break;
                                }
                            }
                        }
                        findAdjacentTrapezoid(currentNode, directionLeft, counter);
                    }
                    break;
                }
                //we are trying to move just one spot in direction
                else if(currentNode->nodeType == Node::EDGE){
                    if(directionLeft){
                        currentNode = currentNode->right;
                    }
                    else{
                        currentNode = currentNode->left;
                    }
                }
                //this works due to the fact that higher nodes are added first and are thus higher in the tree
                else if(currentNode->nodeType == Node::VERTEX){
                    if(subDirectionLeft){
                        currentNode = currentNode->left;
                        subDirectionLeft = false;
                    }
                    else{
                        currentNode = currentNode->right;
                        subDirectionLeft = true;
                    }
                }
            }
        }
};

class SeidelRay{
    public:
        std::shared_ptr<Vertex> start; //origin vertex of ray
        Eigen::Vector3f leftEnd; //point ray terminates in -x direction
        Eigen::Vector3f rightEnd; //point ray terminates in +x direction
        float yValue; //the y value of the ray
        SeidelRay(std::shared_ptr<Vertex> start, std::vector<std::shared_ptr<HalfEdge>> faceEdges){
            int yValue = start->position.y();
            float rightX = FLT_MAX;
            Eigen::Vector3f rightEnd = start->position;
            float leftX = -FLT_MAX;
            Eigen::Vector3f leftEnd = start->position;
            for(std::shared_ptr<HalfEdge> halfEdge : faceEdges){
                Eigen::Vector3f origin = halfEdge->vertex->position;
                Eigen::Vector3f dest = halfEdge->next->vertex->position;
                //if yValue between origin and dest
                if((origin.y() > yValue && yValue > dest.y()) || (origin.y() < yValue && yValue < dest.y())){
                    float yOrigin = origin.y();
                    float yDest = dest.y();
                    float dy = yDest - yOrigin;
                    if(std::abs(dy) < 1e-8f){
                        if(yValue == yOrigin || yValue == yDest){
                            if(origin.x() < dest.x()){
                                if(origin.x() > start->position.x() && origin.x() < rightX){
                                    rightEnd = origin;
                                }
                                if(dest.x() < start->position.x() && dest.x() > leftX){
                                    leftEnd = dest;
                                }
                            }
                        }
                    }
                    else{
                        float mult = (yValue - yOrigin) / dy;
                        Eigen::Vector3f intercept = origin + (mult * (dest - origin));
                        if(intercept.x() > start->position.x() && intercept.x() < rightX){
                            rightEnd = intercept;
                        }
                        if(intercept.x() < start->position.x() && intercept.x() > leftX){
                            leftEnd = intercept;
                        }
                    }
                }
            }
            this->start = start;
            this->yValue = yValue;
            this->rightEnd = rightEnd;
            this->leftEnd = leftEnd;
        }
};