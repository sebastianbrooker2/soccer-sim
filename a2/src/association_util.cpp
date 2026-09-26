#include <cassert>
#include <limits>
#include <vector>
#include <iostream>
#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/features2d.hpp>
#include "GaussianInfo.hpp"
#include "SystemSLAM.h"
#include "Camera.h"
#include "association_util.h"

double snn(const SystemSLAM & system, const GaussianInfo<double> & featureBundleDensity, const std::vector<std::size_t> & idxLandmarks, const Eigen::Matrix<double, 2, Eigen::Dynamic> & Y, const Camera & camera, std::vector<int> & idxFeatures, bool enforceJointCompatibility)
{
    double nSigma = 5.0;

    assert(idxLandmarks.size() <= system.numberLandmarks());
    for (const auto & k : idxLandmarks)
    {
        assert(k < system.numberLandmarks());
    }

    const std::size_t & nL = idxLandmarks.size();
    assert(nL > 0);
    
    assert(Y.rows() == 2);
    assert(Y.cols() > 0);
    int m = Y.cols();

    idxFeatures.clear();
    idxFeatures.resize(nL, -1);

    std::vector<int> midx;
    midx.resize(m);
    for (int i = 0; i < m; ++i)
    {
        midx[i] = i;
    }

    double sU = std::log(camera.imageSize.width) + std::log(camera.imageSize.height);
    double s = nL*sU;
    double smin = std::numeric_limits<double>::infinity();
    std::vector<int> diff;
    std::vector<int>::iterator it, ls, space;
    diff.resize(m);
    
    for (int j = 0; j < nL; ++j)
    {
        GaussianInfo<double> featureDensity = featureBundleDensity.marginal(Eigen::seqN(2*j, 2));

        double dsmin = std::numeric_limits<double>::infinity();
        double scur = s;
        double snext = 0;
        bool jcnext;

        std::vector<int> idxcur;
        idxcur = idxFeatures;
        
        std::vector<int> assignedFeatures;
        for (int k = 0; k < j; ++k) {
            if (idxcur[k] >= 0) {
                assignedFeatures.push_back(idxcur[k]);
            }
        }
        std::sort(assignedFeatures.begin(), assignedFeatures.end());
        
        ls = std::set_difference(midx.begin(), midx.end(), 
                                assignedFeatures.begin(), assignedFeatures.end(), 
                                diff.begin());

        int bestDetection = -1;
        double bestSurprisal = std::numeric_limits<double>::infinity();
        
        for (it = diff.begin(); it < ls; ++it)
        {
            int i = *it;
            
            bool compat = individualCompatibility(Y.col(i), featureDensity, nSigma);
            
            if (!compat)
            {
                continue;
            }

            std::vector<int> idxnext = idxcur;
            idxnext[j] = i;

            jcnext = jointCompatibility(idxnext, sU, Y, featureBundleDensity, nSigma, snext);
            if (enforceJointCompatibility && !jcnext)
            {
                continue;
            }

            double ds = snext - scur;
            
            if (ds < dsmin)
            {
                idxFeatures = idxnext;
                dsmin = ds;
                s = snext;
                bestDetection = i;
                bestSurprisal = snext;
            }
        }
        
        std::vector<int> idxnext = idxcur;
        jointCompatibility(idxnext, sU, Y, featureBundleDensity, nSigma, snext);
        
        double ds = snext - scur;
        if (ds < dsmin)
        {
            idxFeatures = idxnext;
            s = snext;
        }
    }

    if (smin < std::numeric_limits<double>::infinity())
    {
        s = smin;
    }

    return s;
}

bool individualCompatibility(const int & i, const int & j, const Eigen::Matrix<double, 2, Eigen::Dynamic> & Y, const GaussianInfo<double> & density, const double & nSigma)
{
    GaussianInfo<double> marginal = density.marginal(Eigen::seqN(2*j, 2));
    return individualCompatibility(Y.col(i), marginal, nSigma);
}

bool individualCompatibility(const Eigen::Vector2d & y, const GaussianInfo<double> & marginal, const double & nSigma)
{
    return marginal.isWithinConfidenceRegion(y, nSigma);
}

bool jointCompatibility(const std::vector<int> & idx, const double & sU, const Eigen::Matrix<double, 2, Eigen::Dynamic> & Y, const GaussianInfo<double> & density, const double & nSigma, double & surprisal)
{
    int n = idx.size();

    std::vector<int> idxi;
    idxi.reserve(n);
    std::vector<int> idxyj;
    idxyj.reserve(2*n);

    for (int k = 0; k < n; ++k)
    {
        if (idx[k] >= 0)
        {
            idxi.push_back(idx[k]);
            idxyj.insert(idxyj.end(), {2*k, 2*k + 1});
        }
    }
    assert(2*idxi.size() == idxyj.size());

    int nA = idxi.size();
    int nU = n - nA;
  
    if (nA > 0)
    {
        Eigen::VectorXi idxyjEigen = Eigen::Map<const Eigen::VectorXi>(idxyj.data(), idxyj.size());
        GaussianInfo<double> marginalAssociated = density.marginal(idxyjEigen);

        Eigen::VectorXd yA(2*nA);
        for (int q = 0; q < nA; ++q){
            yA.segment<2>(2*q) = Y.col(idxi[q]);
        }

        bool isJointlyCompatible = marginalAssociated.isWithinConfidenceRegion(yA, nSigma);
        double surprisalAssociated = -marginalAssociated.log(yA);
        surprisal = nU*sU + surprisalAssociated;
        return isJointlyCompatible;
    }
    else
    {
        surprisal = nU*sU;
        return true;
    }
}
