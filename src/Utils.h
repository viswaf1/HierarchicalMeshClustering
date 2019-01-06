#pragma once

#include <eigen3/Eigen/Dense>
#include <thread>

template<class T>
Eigen::Matrix<T, 3, 1> TriangleNormal(std::vector<Eigen::Matrix<T, 3, 1>>* vertices, 
                                    std::vector<size_t>& face)
{
    Eigen::Matrix<T, 3, 1> p1, p2, p3 , u, v, n;
    p1 = vertices->at(face[0]);
    p2 = vertices->at(face[1]);
    p3 = vertices->at(face[2]);

    u = p2 - p1;
    v = p3 - p1;
    n[0] = u[1]*v[2] - u[2]*v[1];
    n[1] = u[2]*v[0] - u[0]*v[2];
    n[2] = u[0]*v[1] - u[1]*v[0];

    return n;
}

template<class T>
bool TriangleNormalBatchedThread(std::vector<Eigen::Matrix<T, 3, 1>>* vertices, 
                           std::vector<std::vector<size_t>>* faces,
                           std::vector<Eigen::Matrix<T, 3, 1>>* normals,
                           int threadInd, int numThreads)
{
    for(size_t faceInd = threadInd; faceInd < faces->size(); faceInd += numThreads)
    {
       normals->at(faceInd) = TriangleNormal(vertices, faces->at(faceInd)); 
    }

    return true;
}

template<class T>
bool TriangleNormalBatched(std::vector<Eigen::Matrix<T, 3, 1>>* vertices, 
                           std::vector<std::vector<size_t>>* faces,
                           std::vector<Eigen::Matrix<T, 3, 1>>* normals,
                           int numThreads)
{
    normals->clear();
    normals->resize(faces->size());

    std::vector<std::thread> normThreads;
    for(int tc=0; tc < numThreads; tc++)
    {
        normThreads.push_back(std::thread(TriangleNormalBatchedThread<T>,
                                        vertices, faces, normals,
                                        tc, numThreads));
    }
    for(auto &th : normThreads)
    {
        if(th.joinable())
        {th.join();}
    }

    return true;
}