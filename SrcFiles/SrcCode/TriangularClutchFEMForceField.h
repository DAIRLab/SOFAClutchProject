

//This component is heavy based around the TriangularAnisotropicFEMField component, overriding and replacing some of it's behavior to allow for the clutching behavior. Many of the methods contained here copy the orginial methods and add the required clutching logic. 


#ifndef SOFA_COMPONENT_FORCEFIELD_TRIANGULARCLUTCHFEMFORCEFIELD_H
#define SOFA_COMPONENT_FORCEFIELD_TRIANGULARCLUTCHFEMFORCEFIELD_H
#include "config.h"



#include <sofa/core/behavior/ForceField.h>
#include <sofa/core/topology/BaseMeshTopology.h>
#include <sofa/defaulttype/Vec.h>
#include <sofa/defaulttype/Mat.h>
#include "TriangularFEMForceField.h"
#include "TriangularAnisotropicFEMForceField.h"
#include <SofaBaseTopology/TopologyData.h>
#include <newmat/newmat.h>
#include <newmat/newmatap.h>



namespace sofa
{
namespace component
{
namespace forcefield
{


template<class DataTypes>
class TriangularClutchFEMForceField : public sofa::component::forcefield::TriangularAnisotropicFEMForceField<DataTypes>
{

public:
    SOFA_CLASS(SOFA_TEMPLATE(TriangularClutchFEMForceField, DataTypes), SOFA_TEMPLATE(TriangularAnisotropicFEMForceField, DataTypes));

    typedef sofa::component::forcefield::TriangularAnisotropicFEMForceField<DataTypes> Inherited;
    typedef sofa::component::forcefield::TriangularFEMForceField<DataTypes> baseInherit;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef typename DataTypes::VecReal VecReal;
    typedef VecCoord Vector;
    typedef typename DataTypes::Coord    Coord   ;
    typedef typename DataTypes::Deriv    Deriv   ;
    typedef typename Coord::value_type   Real    ;
    typedef typename Inherited::TriangleInformation   TriangleInformation  ;

    typedef core::objectmodel::Data<VecCoord> DataVecCoord;
    typedef core::objectmodel::Data<VecDeriv> DataVecDeriv;

    typedef sofa::core::topology::BaseMeshTopology::index_type Index;
    typedef sofa::core::topology::BaseMeshTopology::Triangle Element;
    typedef sofa::core::topology::BaseMeshTopology::SeqTriangles VecElement;

    void init() override;
    void reinit() override;
    void draw(const core::visual::VisualParams* vparams) override;
protected:
    TriangularClutchFEMForceField();
    // ~TriangularClutchFEMForceField();
    void accumulateForceLarge(VecCoord &f, const VecCoord &p, Index elementIndex ) override;
    void initLarge(int i, Index&a, Index&b, Index&c) override;



public:
    void computeMaterialStiffness(int i, Index& a, Index& b, Index& c) override;
    


    /// Link to be set to the topology container in the component graph.
    using Inherit1::l_topology;
    using Inherit1::m_topology;

    class ClutchData
    {
        bool AxialActive;
        bool TransverseActive;

        //need to defer evaluation of new setpoint because indicies are not available in this context
        bool AxialEngaged;
        bool TransverseEngaged;

        


        public:
            Coord v1Rest;
            Coord v2Rest;
            Coord v3Rest;
            void setActivation(bool axial, bool transverse)
            {
                AxialEngaged = axial&&!AxialActive;
                TransverseEngaged = transverse &&! TransverseActive;
                AxialActive = axial;
                TransverseActive = transverse;
                 
            }

            void setSetpoint(Coord v1, Coord v2, Coord v3){
                if(AxialEngaged || TransverseEngaged){
                    v1Rest = v1;
                    v2Rest = v2;
                    v3Rest = v3;
                }
                AxialEngaged = false;
                TransverseEngaged = false;
            }

            bool isActive(){
                return AxialActive||TransverseActive;
            }

            

    };

    void processClutchInput()
    {
        helper::vector<bool> axial = AxialActivation.getValue();
        helper::vector<bool> transverse = TransverseActivation.getValue();
        for(int i = 0; i< std::max(axial.size(),transverse.size()); i++)
        {
            bool a_active = false;
            bool t_active = false;
            if(i<axial.size()){
                a_active = axial[i];
            }
            if(i<transverse.size()){
                t_active = transverse[i];
            }
            if(i<clutchData.size()){
                clutchData[i].setActivation(a_active,t_active);
            }
            else{
                //too many inputs were provided so end processing
                break;
            }
        }


    }

    

    helper::vector<ClutchData> clutchData;

    bool DataReady;

    //allows for input from controller, input is processed to determine activations and new rest positions
    Data<helper::vector<bool>> AxialActivation;
    Data<helper::vector<bool>> TransverseActivation;



};

#if  !defined(SOFA_COMPONENT_FORCEFIELD_TRIANGULARCLUTCHFEMFORCEFIELD_CPP)
extern template class SOFA_MISC_FEM_API TriangularClutchFEMForceField<defaulttype::Vec3Types>;

#endif

} // namespace forcefield

} // namespace component

} // namespace sofa

#endif
