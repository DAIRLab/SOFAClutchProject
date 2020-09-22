
//This component is heavy based around the TriangularAnisotropicFEMField component, overriding and replacing some of it's behavior to allow for the clutching behavior. Many of the methods contained here copy the orginial methods and add the required clutching logic. 

#define SOFA_COMPONENT_FORCEFIELD_TRIANGULARCLUTCHFEMFORCEFIELD_CPP
#include "TriangularClutchFEMForceField.inl"
#include <sofa/core/ObjectFactory.h>


namespace sofa
{

namespace component
{

namespace forcefield
{

using namespace sofa::defaulttype;

// Register in the Factory
int TriangularClutchFEMForceFieldClass = core::RegisterObject("Triangular finite element model using anisotropic material to simulate embedded clutches")
        .add< TriangularClutchFEMForceField<Vec3Types> >()

        ;

template class SOFA_MISC_FEM_API TriangularClutchFEMForceField<Vec3Types>;



} // namespace forcefield

} // namespace component

} // namespace sofa
