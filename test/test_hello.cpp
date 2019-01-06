#include "DualGraph.h"
#include "Utils.h"
#include <iostream>
#include "MutablePriorityQueue.h"

#include <igl/readOBJ.h>
#include <igl/opengl/glfw/Viewer.h>

// TEST(Hello, get_message)
// {
//     std::string m = get_message();
//     ASSERT_EQ("Hello, World!", m);

// }

struct HeapStruct
{
    int val;
};

struct StructComp
{ 
    bool operator()(HeapStruct& s1, HeapStruct& s2)
    {
        return (s1.val > s2.val);
    }
};

int main(int argc, char **argv)
{    

    //Mutable priority queue test
    std::vector<HeapStruct*> heapPtrs;
    MutableHeap<HeapStruct, StructComp> mheap;
    HeapStruct v1 = {5};
    heapPtrs.push_back(mheap.Insert(v1));
    v1.val = 7;
    heapPtrs.push_back(mheap.Insert(v1));
    v1.val = 1;
    heapPtrs.push_back(mheap.Insert(v1));
    v1.val = 32;
    heapPtrs.push_back(mheap.Insert(v1));

    HeapStruct top;
    mheap.PopTop(top);
    if(top.val != 32)
    {
        std::cout << "ERROR: wrong top value in heap" << std::endl;
    }
    else
    {
        std::cout << "MutableHeap Pop top element " << top.val << " Success" << std::endl;
    }
    heapPtrs.pop_back();
    HeapStruct* lastElemPtr = heapPtrs.back();
    lastElemPtr->val = 500;
    if(!mheap.UpdateVal(lastElemPtr))
    {
        std::cout << "ERROR: cannot update heap" << std::endl;
    }
    mheap.PopTop(top);
    if(top.val != 500)
    {
        std::cout << "ERROR: wrong top value in updated heap" << std::endl;
    }
    else
    {
        std::cout << "MutableHeap Pop top element after update" << top.val << " Success" << std::endl;
    }
    heapPtrs.pop_back();
    heapPtrs.back()->val = 200;
    mheap.UpdateVal(heapPtrs.back());
    HeapStruct peekTop;
    mheap.PeekTop(peekTop);
    if(peekTop.val != 200)
    {
        std::cout << "ERROR: wrong top value in updated heap" << std::endl;
    }
    else
    {
        std::cout << "MutableHeap Pop top element after update" << peekTop.val << " Success" << std::endl;
    }

    heapPtrs.back()->val = -20;
    mheap.PeekTop(peekTop);
    if(peekTop.val != -20)
    {
        std::cout << "ERROR: wrong top value in updated heap" << std::endl;
    }
    else
    {
        std::cout << "MutableHeap Pop top element after update" << peekTop.val << " Success" << std::endl;
    }

    ////Triange normal Test  
    std::cout << "Testing Triangle normal calc" << std::endl;
    std::vector<Eigen::Vector3f> verts(3);
    verts[0] << 0, 0, 0;
    verts[1] << 1, 0, 0;
    verts[2] << 0, 1, 0;

    std::vector<std::vector<size_t>> faces(2);
    std::vector<size_t> face1 = {0, 1, 2};
    std::vector<size_t> face2 = {0 ,2, 1};
    faces[0] = face1;
    faces[1] = face2;
    
    Eigen::Vector3f normal, correctNormal;
    std::vector<Eigen::Vector3f> correctNormals;
    correctNormal << 0, 0, 1;
    correctNormals.push_back(correctNormal);
    normal = TriangleNormal(&verts, faces[0]);
    std::cout << "Normal for triangle " << verts[faces[0][0]].transpose() << ", "
            << verts[faces[0][1]].transpose() << ", " << verts[faces[0][2]].transpose()
             << " is " <<
            normal.transpose() << std::endl;
    if( (normal - correctNormal).norm() > 0.0001 )
    {
        std::cout << "Normal calculation failed" << std::endl;
        std::cout << "!!!! Normal Evaluation FAIL !!!!!!" << std::endl;
    } 

    correctNormal << 0, 0, -1;
    correctNormals.push_back(correctNormal);
    normal = TriangleNormal(&verts, faces[1]);
    std::cout << "Normal for triangle " << verts[faces[1][0]].transpose() << ", "
            << verts[faces[1][1]].transpose() << ", " << verts[faces[1][2]].transpose()
             << " is " <<
            normal.transpose() << std::endl;
    if( (normal - correctNormal).norm() > 0.0001 )
    {
        std::cout << "Normal calculation failed" << std::endl;
        std::cout << "!!!! Normal Evaluation FAIL !!!!!!" << std::endl;
    } 
    std::cout << "***** Normal Evaluation PASS *******" << std::endl;

    std::cout << "Testing Batched normal estimation" << std::endl;
    std::vector<Eigen::Vector3f> batchNormals;
    TriangleNormalBatched(&verts, &faces, &batchNormals, 5);
    for(size_t normInd = 0; normInd < batchNormals.size(); normInd++)
    {
        if( (batchNormals[normInd] - correctNormals[normInd]).norm() > 0.0001 )
        {
            std::cout << "!!!! Batched Normal Evaluation FAIL !!!!!!" << std::endl;
        }
    }
    std::cout << "***** Batched Normal Evaluation PASS *******" << std::endl;
    /////////////////////////////////////////

    //std::vector<Eigen::Vector3f> vertices;
    // std::vector<std::vector<size_t>> faces2;
    DualGraph<float> dg(&verts, faces);

    const char* objFilesDir = std::getenv("TEST_OBJ_PATH");
    if(objFilesDir == nullptr)
    {
        std::cout << "Environment variable TEST_OBJ_PATH not set" << std::endl;
        return 1;
    }

    std::string testObjFile = std::string(objFilesDir)+std::string("/airboat.obj");
    std::cout << "Opening obj file " << testObjFile << std::endl;

    std::vector<std::vector<float>> readVerts;
    std::vector<std::vector<size_t>> readFaces;
    if(!igl::readOBJ(testObjFile, readVerts, readFaces))
    {
        std::cout << "Error reading obj file " << testObjFile << std::endl;
    }

    std::vector<Eigen::Vector3f> testVerts;
    for(auto &eachVecVert : readVerts)
    {   
        Eigen::Vector3f v;
        v << eachVecVert[0], eachVecVert[1], eachVecVert[2];
        testVerts.push_back(v);
    }


    DualGraph<float> testDualGraph(&testVerts, readFaces);
    std::vector<std::vector<size_t>> segmentFaces;
    testDualGraph.RunSegmentation(0.5, segmentFaces);


    return 0;
}