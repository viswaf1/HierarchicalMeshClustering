#include <eigen3/Eigen/Dense>
#include <thread>
#include <vector>

template<class T>
bool VecTMatVecMulThread(std::vector<Eigen::Matrix<T, 3, 1>>* allVecs,
                        std::vector<size_t>& vecInds,
                        Eigen::Matrix<T, 4, 4>& mat,
                        std::vector<T>& results,
                        int threadIndex, int numThreads);

//Given a set of points as Vec3 by a set of Vec3's and indices of the
//Vec3's for the set to use, and 4x4 Matrix,
//performs the operation v_Transpose * M * v on each Vec3
template<class T>
bool VecTMatVecMulBatched(std::vector<Eigen::Matrix<T, 3, 1>>* allVecs,
                            std::vector<size_t>& vecInds,
                            Eigen::Matrix<T, 4, 4>& mat,
                            std::vector<T>& result, 
                            int numThreads = -1)
{
    if(numThreads <= 0)
    {
        numThreads = std::thread::hardware_concurrency();
    }

    result.clear();
    result.resize(vecInds.size());
    std::vector<std::thread> opThreads;
    for(int tc = 0; tc < numThreads; tc++)
    {
        opThreads.push_back(std::thread(VecTMatVecMulThread<T>, 
                                        allVecs, std::ref(vecInds),
                                        std::ref(mat), std::ref(result),
                                        tc, numThreads));
    }

    for(auto &eachTh : opThreads)
    {
        if(eachTh.joinable())
        {
            eachTh.join();
        }
    }

    return true;
}


template<class T>
bool VecTMatVecMulThread(std::vector<Eigen::Matrix<T, 3, 1>>* allVecs,
                        std::vector<size_t>& vecInds,
                        Eigen::Matrix<T, 4, 4>& mat,
                        std::vector<T>& results,
                        int threadIndex, int numThreads)
{
    for(int vecIndInd = threadIndex; vecIndInd < vecInds.size(); 
                vecIndInd += numThreads)
    {
        results[vecIndInd] = (allVecs->at(vecInds[vecIndInd])).homogeneous().transpose() *
                                mat * (allVecs->at(vecInds[vecIndInd])).homogeneous();
    }
    return true;
}
