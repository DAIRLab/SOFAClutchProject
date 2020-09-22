//This component is heavy based around the TriangularAnisotropicFEMField component, overriding and replacing some of it's behavior to allow for the clutching behavior. Many of the methods contained here copy the orginial methods and add the required clutching logic. 

#ifndef SOFA_COMPONENT_FORCEFIELD_TRIANGULARCLUTCHFEMFORCEFIELD_INL
#define SOFA_COMPONENT_FORCEFIELD_TRIANGULARCLUTCHFEMFORCEFIELD_INL

#include "TriangularClutchFEMForceField.h"
#include <sofa/core/visual/VisualParams.h>
#include <sofa/defaulttype/RGBAColor.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/core/topology/BaseMeshTopology.h>
#include <SofaBaseTopology/TopologyData.inl>
#include <fstream> // for reading the file
#include <iostream> //for debugging
#include <vector>
#include <algorithm>
#include <sofa/defaulttype/VecTypes.h>
#include <cassert>

namespace sofa
{

namespace component
{

namespace forcefield
{

template <class DataTypes>
TriangularClutchFEMForceField<DataTypes>::TriangularClutchFEMForceField()
   : AxialActivation(initData(&AxialActivation,helper::vector<bool>(1,false),"AxialClutchActivation","AxialClutchActivation","Indicies hold info on which elements are active"))
   , TransverseActivation(initData(&TransverseActivation,helper::vector<bool>(1,false),"TransverseClutchActivation","TransverseClutchActivation","Indicies hold info on which elements are active"))
{
    
}

//right now the destructor is unnecessary, but can be used in the future
/*
template <class DataTypes>
TriangularClutchFEMForceField<DataTypes>::~TriangularClutchFEMForceField()
{
}
*/


template< class DataTypes>
void TriangularClutchFEMForceField<DataTypes>::init()
{
    if (l_topology.empty())
    {
        msg_info() << "link to Topology container should be set to ensure right behavior. First Topology found in current context will be used.";
        l_topology.set(this->getContext()->getMeshTopologyLink());
    }

    m_topology = l_topology.get();
    msg_info() << "Topology path used: '" << l_topology.getLinkedPath() << "'";

    if (m_topology == nullptr)
    {
        msg_error() << "No topology component found at path: " << l_topology.getLinkedPath() << ", nor in current context: " << this->getContext()->name;
        sofa::core::objectmodel::BaseObject::d_componentstate.setValue(sofa::core::objectmodel::ComponentState::Invalid);
        return;
    }
    Inherited::init();
    
    // initialize clutch data
    helper::vector<TriangleInformation>& triangleInf = *(baseInherit::triangleInfo.beginEdit());
    clutchData.clear();
    for(int i = 0; i < triangleInf.size(); i++)
    {
        clutchData.push_back(ClutchData());
    }
    DataReady = true;
    reinit();
}

template <class DataTypes>
void TriangularClutchFEMForceField<DataTypes>::reinit()
{
    // need to set all clutch elements to false

    if(DataReady)
    {
        //check if input has changed and mark elements that need reevaluation;
        processClutchInput();
        Inherited::reinit();
        int count = 0;
        for(int i = 0; i < clutchData.size(); i++)
        {
            if(clutchData[i].isActive()){
                count++;
            }
        }
        // msg_error() << "number of active elements: " + std::to_string(count);
    }
}

// modified method from anisotropic component
template <class DataTypes>
void TriangularClutchFEMForceField<DataTypes>::initLarge(int i, Index&a, Index&b, Index&c)
{

    if(!(i<clutchData.size()))return;
    if(!(clutchData[i].isActive())) return;
    helper::vector<TriangleInformation>& triangleInf = *(baseInherit::triangleInfo.beginEdit());



    msg_error_when((unsigned int)i >= triangleInf.size())
            << "Try to access an element which indices bigger than the size of the vector: i=" << i << " and size=" << triangleInf.size() ;



    const VecCoord& currentPos = (this->mstate->read(core::ConstVecCoordId::position())->getValue());
    
    

    clutchData[i].setSetpoint(currentPos[a],currentPos[b],currentPos[c]);

    TriangleInformation *tinfo = &triangleInf[i];

    // Rotation matrix (initial triangle/world)
    // first vector on first edge
    // second vector in the plane of the two first edges
    // third vector orthogonal to first and second
    defaulttype::Mat<3, 3, Real > R_0_1;

    VecCoord temp{clutchData[i].v1Rest,clutchData[i].v2Rest,clutchData[i].v3Rest};
    
    baseInherit::computeRotationLarge( R_0_1, (temp), 0, 1, 2 );

    tinfo->initialTransformation = R_0_1;

    tinfo->rotatedInitialElements[0] = R_0_1 * (clutchData[i].v1Rest - clutchData[i].v1Rest);
    tinfo->rotatedInitialElements[1] = R_0_1 * (clutchData[i].v2Rest - clutchData[i].v1Rest);
    tinfo->rotatedInitialElements[2] = R_0_1 * (clutchData[i].v3Rest - clutchData[i].v1Rest);

    baseInherit::computeStrainDisplacement(tinfo->strainDisplacementMatrix, i, tinfo->rotatedInitialElements[0], tinfo->rotatedInitialElements[1], tinfo->rotatedInitialElements[2]);

    baseInherit::triangleInfo.endEdit();
}

//modified method originially from anisotropic material component to include clutch activation data
template <class DataTypes>
void TriangularClutchFEMForceField<DataTypes>::computeMaterialStiffness(int i, Index& v1, Index& v2, Index& v3)
{


    
    if(!(i<clutchData.size()))return;
    if(!(clutchData[i].isActive())) return;
   

    Real Q11, Q12, Q22, Q66;
    Coord fiberDirGlobal;  // orientation of the fiber in the global frame of reference

    Coord fiberDirLocalOrtho; //  // orientation of the fiber in the local orthonormal frame of the element
    defaulttype::Mat<3,3,Real> T, Tinv;

    helper::vector<TriangleInformation>& triangleInf = *(baseInherit::triangleInfo.beginEdit());

    TriangleInformation *tinfo = &triangleInf[i];

    const helper::vector<Real> & youngArray = baseInherit::f_young.getValue();
    const helper::vector<Real> & young2Array = Inherited::f_young2.getValue();
    const helper::vector<Real> & poissonArray = baseInherit::f_poisson.getValue();
    const helper::vector<Real> & poisson2Array = Inherited::f_poisson2.getValue();

    unsigned int y_index = 0;
    unsigned int y2_index = 0;
    unsigned int p_index = 0;
    unsigned int p2_index = 0;

    if (i < (int) youngArray.size() )
        y_index = i;
    if (i < (int) young2Array.size() )
        y2_index = i;
    if (i < (int) poissonArray.size() )
        p_index = i;
    if (i < (int) poisson2Array.size() )
        p2_index = i;
    
    Q11 = youngArray[y_index] /(1-poissonArray[p_index]*poisson2Array[p2_index]);
    Q12 = poissonArray[p_index]*young2Array[y2_index]/(1-poissonArray[p_index]*poisson2Array[p2_index]);
    Q22 = young2Array[y2_index]/(1-poissonArray[p_index]*poisson2Array[p2_index]);
    Q66 = (Real)(youngArray[y_index] / (2.0*(1 + poissonArray[p_index])));

    T[0] = clutchData[i].v2Rest-clutchData[i].v1Rest;
    T[1] = clutchData[i].v3Rest-clutchData[i].v1Rest;
    T[2] = cross(T[0], T[1]);

    if (T[2] == Coord())
    {
        msg_error() << "Cannot compute material stiffness for a flat triangle. Abort computation. ";
        return;
    }

    if (!this->f_fiberCenter.getValue().empty()) // in case we have concentric fibers
    {
        Coord tcenter = (clutchData[i].v1Rest+clutchData[i].v2Rest+clutchData[i].v3Rest)*(Real)(1.0/3.0);
        Coord fcenter = Inherited::f_fiberCenter.getValue()[0];
        fiberDirGlobal = cross(T[2], fcenter-tcenter);  // was fiberDir
    }
    else // for unidirectional fibers
    {
        double theta = (double)Inherited::f_theta.getValue()*M_PI/180.0;
        fiberDirGlobal = Coord((Real)cos(theta), (Real)sin(theta), 0); // was fiberDir
    }

    helper::vector<Deriv>& lfd = *(Inherited::localFiberDirection.beginEdit());

    if ((unsigned int)i >= lfd.size())
    {
        /* ********************************************************************************************
         * this can happen after topology changes
         * apparently, the topological changes are not propagated through localFiberDirection
         * that's why we resize this vector to triangleInf size to hack the crash when we're looking for
         * a element which index is more than the size
         * This hack is probably useless if there would be a good topological propagation
        ***********************************************************************************************/
        dmsg_warning() << "Get an element in localFiberDirection with index more than its size: i=" << i
                       << " and size=" << lfd.size() << ". The size should be "  <<  triangleInf.size() <<" (see comments in TriangularAnisotropicFEMForceField::computeMaterialStiffness)" ;
        lfd.resize(triangleInf.size() );
        dmsg_info() << "LocalFiberDirection resized to " << lfd.size() ;
    }
    else
    {
        Deriv& fiberDirLocal = lfd[i]; // orientation of the fiber in the local frame of the element (orthonormal frame)
        T.transpose();
        Tinv.invert(T);
        fiberDirLocal = Tinv * fiberDirGlobal;
        fiberDirLocal[2] = 0;
        fiberDirLocal.normalize();
    }

    T[0] = clutchData[i].v2Rest-clutchData[i].v1Rest;
    T[1] = clutchData[i].v3Rest-clutchData[i].v1Rest;
    T[2] = cross(T[0], T[1]);
    T[1] = cross(T[2], T[0]);
    T[0].normalize();
    T[1].normalize();
    T[2].normalize();
    T.transpose();
    Tinv.invert(T);
    fiberDirLocalOrtho = Tinv * fiberDirGlobal;
    fiberDirLocalOrtho[2] = 0;
    fiberDirLocalOrtho.normalize();

    Real c, s, c2, s2, c3, s3,c4, s4;
    c = fiberDirLocalOrtho[0];
    s = fiberDirLocalOrtho[1];

    c2 = c*c;
    s2 = s*s;
    c3 = c2*c;
    s3 = s2*s;
    s4 = s2*s2;
    c4 = c2*c2;
    Real K11= c4 * Q11 + 2 * c2 * s2 * (Q12+2*Q66) + s4 * Q22;
    Real K12 = c2 * s2 * (Q11+Q22-4*Q66) + (c4+s4) * Q12;
    Real K22 = s4* Q11 + 2 * c2 * s2 * (Q12+2*Q66) + c4 * Q22;
    Real K16 = s * c3 * (Q11-Q12-2*Q66) + s3 * c * (Q12-Q22+2*Q66);
    Real K26 = s3 * c * (Q11-Q12-2*Q66) + s * c3 * (Q12-Q22+2*Q66);
    Real K66 = c2 * s2 * (Q11+Q22-2*Q12-2*Q66) + (c4+s4) * Q66;

    tinfo->materialMatrix[0][0] = K11;
    tinfo->materialMatrix[0][1] = K12;
    tinfo->materialMatrix[0][2] = K16;
    tinfo->materialMatrix[1][0] = K12;
    tinfo->materialMatrix[1][1] = K22;
    tinfo->materialMatrix[1][2] = K26;
    tinfo->materialMatrix[2][0] = K16;
    tinfo->materialMatrix[2][1] = K26;
    tinfo->materialMatrix[2][2] = K66;

    Inherited::localFiberDirection.endEdit();
    baseInherit::triangleInfo.endEdit();
}

template<class DataTypes>
void TriangularClutchFEMForceField<DataTypes>::accumulateForceLarge(VecCoord &f, const VecCoord &p, Index elementIndex )
{
    if(!(elementIndex<clutchData.size()) || !clutchData[elementIndex].isActive()) return;

    // msg_error()<<"clutch accumulate forces";
    baseInherit::accumulateForceLarge(f,p,elementIndex);
}

// ----------------------------------------------------------------
// ---	Display
// ----------------------------------------------------------------
template <class DataTypes>
void TriangularClutchFEMForceField<DataTypes>::draw(const core::visual::VisualParams* vparams)
{
    baseInherit::draw(vparams);

    if (!vparams->displayFlags().getShowForceFields())
        return;
    // msg_error()<< "Drawing from clutch";
    helper::vector<Deriv>& lfd = *(Inherited::localFiberDirection.beginEdit());

    if (Inherited::showFiber.getValue() && lfd.size() >= (unsigned)m_topology->getNbTriangles())
    {
        
        vparams->drawTool()->saveLastState();
        sofa::defaulttype::RGBAColor lineColor(0, 0, 1.0, 1.0);
        std::vector<sofa::defaulttype::Vector3> lineVertices;

        const VecCoord& x = this->mstate->read(core::ConstVecCoordId::position())->getValue();
        int nbTriangles=m_topology->getNbTriangles();
        // msg_error()<<"Pass first if, clutch draw, number of triangles: " + std::to_string(nbTriangles) + ", lfd.size: " + std::to_string(lfd.size());
        
        sofa::defaulttype::RGBAColor color;
        std::vector<sofa::defaulttype::Vec4f> colorVector;
        std::vector<sofa::defaulttype::Vector3> vertices;

        for(int i=0; i<nbTriangles; ++i)
        {
            if(!(i<clutchData.size())) continue;
            if(!(clutchData[i].isActive())) continue;

            Index a = m_topology->getTriangle(i)[0];
            Index b = m_topology->getTriangle(i)[1];
            Index c = m_topology->getTriangle(i)[2];

            colorVector.push_back(sofa::defaulttype::RGBAColor(1,0,0,1));
            vertices.push_back(sofa::defaulttype::Vector3(x[a]));
            colorVector.push_back(sofa::defaulttype::RGBAColor(1,0,0,1));
            vertices.push_back(sofa::defaulttype::Vector3(x[b]));
            colorVector.push_back(sofa::defaulttype::RGBAColor(1,0,0,1));
            vertices.push_back(sofa::defaulttype::Vector3(x[c]));

        }
        vparams->drawTool()->disableLighting();

        vparams->drawTool()->drawTriangles(vertices,colorVector);
        vertices.clear();
        colorVector.clear();
        vparams->drawTool()->drawLines(lineVertices,1,lineColor);
        vparams->drawTool()->restoreLastState();
    }
    Inherited::localFiberDirection.endEdit();
}

} // namespace forcefield

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_FORCEFIELD_TRIANGULARANISOTROPICFEMFORCEFIELD_INL
