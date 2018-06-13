DefKit v0.1
Deformable Bodies Toolkit for Unity3D
support: korzenio [at] gmail.com

DefKit is the simplest solution for adding deformable objects (a.k.a. softbodies) to your games. 

With just one click:
First, DefKit tetrahedralizes the 3D object, i.e. splits it's volume into a large number of small tetrahedrons (tets, for short).
Next, for each corner of every tet, DefKit generates a small rigidbody. 
Finally, it connects all these rigidbodies along the tet edges by spring joints creating a, so called, mass-spring model (MSM).

DefKit relies on Unity's built-in physics and runs on all available platforms.
The included scripts help with visual mesh updates and control of mechanical parameters such as springs stiffness or friction coefficients.
DefKit gives access to raw tetrahedralized meshes (nodes, links, tets). These can be use, for example, in simulation using finite element method (FEM) 
This asset is still in development. Any bug reports and feature requests are more than welcome. 

Manual:

Watch the QuickStart video: https://www.youtube.com/watch?v=gbRLMiWbXU0

After packege import:
1. Add LayeresMask at slot 11 called DefKitMeshColliders 
2. Add LayeresMask at slot 12 called DefKitNodes
3. In Edit/Project Settings/Physics uncheck the collision between ALL these layers (also self-collisions i.e. DefKitNodes vs DefKitNodes)
4. In Edit/Project Settings/Time decrease the Fixed Timestep to below 0.01


Release Notes:

v0.1
- Initial release
- Mesh tetrahedralization
- Generating PhysX object 
- Visual mesh and collider update

Future Work::
- Fast, multi-threaded physics implementation
- Generated mesh quality control
- Tetrahedron volume constraints
- Strain based dynamics
- Finite element method
- Shape matching