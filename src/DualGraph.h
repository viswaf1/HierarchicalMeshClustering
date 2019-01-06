#include <eigen3/Eigen/Dense>
#include <queue>
#include <map>
#include <set>
#include <iostream>
#include "MatVecsMulBatched.h"
#include "Utils.h"
#include "MutablePriorityQueue.h"

template<class T>
class DualGraphEdge;

template<class T>
class DualGraphNode
{
public:
    DualGraphNode(std::vector<size_t>& triVertIndices,
                std::vector<size_t>& faceIndices, Eigen::Matrix<T, 3, 1>& triNormal, 
                std::vector<Eigen::Matrix<T, 3, 1>>* verticesPtr)
    {   
        for(auto &vert : triVertIndices)
        {
            m_vertexIndices.push_back(vert);
        }
        for(auto &faceIndex : faceIndices)
        {
            m_faceIndices.push_back(faceIndex);
        }
        m_verticesPtr = verticesPtr;

        //Initialze the error quadrics
        Eigen::Matrix<T, 3, 1> pt = m_verticesPtr->at(m_vertexIndices[0]);
        T a,b,c,d;
        a = triNormal[0]; b = triNormal[1]; c = triNormal[2];
        d = -(a*pt[0] + b*pt[1] + c*pt[2]);

        m_quadric << a*a, a*b, a*c, a*d,
                    a*b, b*b, b*c, b*d,
                    a*c, b*c, c*c, c*d,
                    a*d, b*d, c*d, d*d; 


    };

    DualGraphNode(std::vector<size_t>& triVertIndices,
                std::vector<size_t>& faceIndices, Eigen::Matrix<T, 4, 4>& errorQuadric, 
                std::vector<Eigen::Matrix<T, 3, 1>>* verticesPtr)
    {   
        for(auto &vert : triVertIndices)
        {
            m_vertexIndices.push_back(vert);
        }
        for(auto &faceIndex : faceIndices)
        {
            m_faceIndices.push_back(faceIndex);
        }
        m_verticesPtr = verticesPtr;

        //Initialze the error quadrics
        m_quadric = errorQuadric;
    };

    ~DualGraphNode(){};

    void AddEdge(DualGraphEdge<T>* edge)
    {
        m_edges.push_back(edge);
    };

    std::vector<size_t> m_vertexIndices;
    std::vector<size_t> m_faceIndices;
    std::vector<Eigen::Matrix<T, 3, 1>>* m_verticesPtr;
    std::vector<DualGraphEdge<T>*> m_edges;
    Eigen::Matrix<T, 4, 4> m_quadric;

};

template<class T>
class DualGraphEdge
{
public:
    DualGraphEdge(){};
    DualGraphEdge(DualGraphEdge<T>& other)
    {
        *this = other;
    };
    DualGraphEdge<T>& operator=(DualGraphEdge<T>& other)
    {
        m_vertex1 = other.m_vertex1;
        m_vertex2 = other.m_vertex2;
        m_cost = other.m_cost;
    };
    DualGraphEdge(DualGraphNode<T>* v1, DualGraphNode<T>* v2):
        m_vertex1(v1), m_vertex2(v2)
    {
        UpdateEdgeCost();
    };
    
    DualGraphNode<T>* GetNeighbor(DualGraphNode<T>* vertex)
    {
        if(m_vertex1 == vertex)
        {
            return m_vertex2;
        }
        else
        {
            return m_vertex1;
        }
    }

    bool UpdateEdgeCost()
    {
        // std::cout << "Adding " << std::endl << m_vertex1->m_quadric << " and " << std::endl << m_vertex2->m_quadric << std::endl;
        //calculate the new error quadric 
        Eigen::Matrix<T, 4, 4> newQuadric = m_vertex1->m_quadric +
                                             m_vertex2->m_quadric;
        std::vector<size_t> allInds;
        allInds.reserve(m_vertex1->m_vertexIndices.size() +
                        m_vertex2->m_vertexIndices.size());
        allInds.insert(allInds.end(), m_vertex1->m_vertexIndices.begin(),
                        m_vertex1->m_vertexIndices.end());
        allInds.insert(allInds.end(), m_vertex2->m_vertexIndices.begin(),
                        m_vertex2->m_vertexIndices.end());
        
        std::vector<T> opResults;
        VecTMatVecMulBatched(m_vertex1->m_verticesPtr,
                            allInds, newQuadric,
                            opResults, -1);
        T quadricError;
        for(T &vertErr : opResults)
        {
            quadricError += vertErr;
        }
        quadricError /= static_cast<float>(opResults.size());
        m_cost = quadricError;
        // std::cout << "Edge cost " << m_cost << std::endl;
        return true;

    }

    bool CollapseEdge(DualGraphNode<T>* newVertex,
                    std::vector<DualGraphEdge<T>*>& touchedEdgePtrs)
    {   
        std::vector<size_t> combVertexIndices;
        combVertexIndices.insert(combVertexIndices.end(),
                m_vertex1->m_vertexIndices.begin(),
                m_vertex1->m_vertexIndices.end());
        combVertexIndices.insert(combVertexIndices.end(),
                m_vertex2->m_vertexIndices.begin(),
                m_vertex2->m_vertexIndices.end());
        
        std::vector<size_t>combinedFaceIndices;
        combinedFaceIndices.insert(combinedFaceIndices.end(),
                m_vertex1->m_faceIndices.begin(),
                m_vertex1->m_faceIndices.end());
        combinedFaceIndices.insert(combinedFaceIndices.end(),
                m_vertex2->m_faceIndices.begin(),
                m_vertex2->m_faceIndices.end());

        Eigen::Matrix<T, 4, 4> newQuadric = m_vertex1->m_quadric +
                                             m_vertex2->m_quadric;

        newVertex = new DualGraphNode<T>(combVertexIndices, 
                                    combinedFaceIndices,
                                    newQuadric,
                                    m_vertex1->m_verticesPtr);
        for(auto &edgePtr : m_vertex1->m_edges)
        {
            if(edgePtr != this)
            {
                //Update the vertex pointer of the edges
                //to point to the new vertex
                if(edgePtr->m_vertex1 == m_vertex1)
                {
                    edgePtr->m_vertex1 = newVertex;
                }
                else
                {
                    edgePtr->m_vertex2 = newVertex;
                }

                edgePtr->UpdateEdgeCost();
                newVertex->AddEdge(edgePtr);
                touchedEdgePtrs.push_back(edgePtr);
            }
        }
        for(auto &edgePtr : m_vertex2->m_edges)
        {
            if(edgePtr != this)
            {
                if(edgePtr->m_vertex1 == m_vertex2)
                {
                    edgePtr->m_vertex1 = newVertex;
                }
                else
                {
                    edgePtr->m_vertex2 = newVertex;
                }

                edgePtr->UpdateEdgeCost();
                newVertex->AddEdge(edgePtr);
                touchedEdgePtrs.push_back(edgePtr);
            }
        }

        std::cout << "Collapsed edge " << std::endl;
        return true;
    }

    DualGraphNode<T>* m_vertex1;
    DualGraphNode<T>* m_vertex2;
    float m_cost;
};

template<class T>
struct DualGraphEdgeComp
{
    bool operator()(DualGraphEdge<T>& dge1, DualGraphEdge<T>& dge2)
    {
        return (dge1.m_cost < dge2.m_cost);
    }
};

//Comparision class for the min queue of edges
template<class T>
class PQueueCompare
{
public:
    bool operator()(const DualGraphEdge<T>* lEdge, 
                    const DualGraphEdge<T>* rEdge)
    {
        return (lEdge->m_cost > rEdge->m_cost);
    }
};

template<class T>
class DualGraph
{
public:
    DualGraph(std::vector<Eigen::Matrix<T, 3, 1>>* vertices,
                std::vector<std::vector<size_t>> faces,
                int numThreads = -1)
    {   
        if(numThreads <= 0)
        {
            numThreads = std::thread::hardware_concurrency(); 
        }
        m_inputNormals = false;
        normals = new std::vector<Eigen::Matrix<T, 3, 1>>;
        TriangleNormalBatched(vertices, &faces, normals, numThreads);
        
        //map between an edge pair and the index in m_graphNodes
        std::map<std::pair<size_t, size_t>, std::vector<DualGraphNode<T>*>> edgeNodeMap;

        // std::vector<DualGraphNode<T>*> graphNodePtrs;

        //Iterate over each edge in all the triangle faces and create a graph node
        //for each face, and add them to the corresponding edge in the edgeNodeMap.
        for(size_t faceInd = 0; faceInd < faces.size(); faceInd++)
        {
            std::vector<size_t>& face = faces.at(faceInd);
            std::vector<size_t> faceInds(1, faceInd);
            
            DualGraphNode<T>* currNodePtr = new DualGraphNode<T>(face, faceInds, 
                                                            normals->at(faceInd), vertices);
            m_graphNodePtrs.insert(currNodePtr);

            // m_graphNodes.push_back(DualGraphNode<T>(face, faceInd, 
            //             normals->at(faceInd), vertices));
            size_t currNodeInd = faceInd;

            for(size_t edgeInd = 0; edgeInd < face.size(); edgeInd++)
            {
                std::pair<size_t, size_t> edgePair;
                int vInd1, vInd2;
                vInd1 = face[edgeInd];
                vInd2 = face[(edgeInd+1)%face.size()];
                if(vInd1 < vInd2)
                {
                    edgePair = std::pair<size_t, size_t>(vInd1, vInd2);
                }
                else
                {
                    edgePair = std::pair<size_t, size_t>(vInd2, vInd1);
                }
                edgeNodeMap[edgePair].push_back(currNodePtr);
            }
        }

        //Iterate over the edgeNodeMap and for each edge that has more than one graph nodes,
        //Initialize a DualGraphEdge with the nodes
        for(auto nodeMapIter = edgeNodeMap.begin(); nodeMapIter != edgeNodeMap.end();
                nodeMapIter++ )
        {
            if(nodeMapIter->second.size() > 1)
            {
                for(size_t v1 = 0; v1 < nodeMapIter->second.size() - 1; v1++)
                {
                    for(size_t v2 = v1+1; v2 < nodeMapIter->second.size(); v2++)
                    {
                        DualGraphEdge<T> currEdge(nodeMapIter->second[v1],
                                nodeMapIter->second[v2]);
                        DualGraphEdge<T>* currEdgePtr = m_edgeHeap.Insert(currEdge);
                        
                        //m_edgePriorityQueue.push(currEdgePtr);

                        nodeMapIter->second[v1]->AddEdge(currEdgePtr);
                        nodeMapIter->second[v2]->AddEdge(currEdgePtr);

                        // m_edgePriorityQueue.push(DualGraphEdge<T>(graphNodePtrs[nodeMapIter->second[v1]],
                        //         graphNodePtrs[nodeMapIter->second[v2]]));
                    }
                }
            }
        }



    };
    ~DualGraph()
    {
        if(!m_inputNormals)
        {
            delete normals;
        }
    };

    bool RunSegmentation(float errorThreshold, std::vector<std::vector<size_t>>& segmentFaces)
    {
        while(m_edgeHeap.size() > 0)
        {
            DualGraphEdge<T> topEdge;
            if(!m_edgeHeap.PopTop(topEdge))
            {
                break;
            }

            //Check the cost and collapse the edge
            if(topEdge.m_cost < errorThreshold)
            {
                DualGraphNode<T>* edgeNode1;
                DualGraphNode<T>* edgeNode2;
                edgeNode1 = topEdge.m_vertex1;
                edgeNode2 = topEdge.m_vertex2;

                std::vector<DualGraphEdge<T>*> touchedEdges;
                DualGraphNode<T>* newNode;
                topEdge.CollapseEdge(newNode, touchedEdges);

                //Erase the edge nodes and add the new node
                m_graphNodePtrs.erase(edgeNode1);
                m_graphNodePtrs.erase(edgeNode2);

                delete edgeNode1;
                delete edgeNode2;
                m_graphNodePtrs.insert(newNode);

                //Update the touched edges in the heap
                for(DualGraphEdge<T>* touchedEdge : touchedEdges)
                {
                    m_edgeHeap.UpdateVal(touchedEdge);
                }

            }
            else
            {
                break;
            }

        }

        //Iterate over the remaining graph nodes and generate the face segments
        for(auto graphNodeSetIter = m_graphNodePtrs.begin(); 
                graphNodeSetIter != m_graphNodePtrs.end();
                graphNodeSetIter++)
        {
            segmentFaces.push_back((*graphNodeSetIter)->m_faceIndices);
        }

        return true;
    }

    std::vector<Eigen::Matrix<T, 3, 1>>* vertices;
    std::vector<Eigen::Matrix<T, 3, 1>>* normals;

    // std::priority_queue<DualGraphEdge<T>*, std::vector<DualGraphEdge<T>*>, 
    //             PQueueCompare<T>> m_edgePriorityQueue;
    MutableHeap<DualGraphEdge<T>, DualGraphEdgeComp<T>> m_edgeHeap;
    std::set<DualGraphNode<T>*> m_graphNodePtrs;
    bool m_inputNormals;
    int m_numThreads;


};