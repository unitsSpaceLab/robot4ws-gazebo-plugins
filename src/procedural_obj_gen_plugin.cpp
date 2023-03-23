/*
@author: Matteo Caruso
@email: matteo.caruso@phd.units.it
@email: matteo.caruso1993@gmail.com
*/

#include "../include/procedural_obj_gen_plugin.h"

using namespace gazebo;

// Constructor
proceduralObjGen::proceduralObjGen()
{
    
}

// Destructor
proceduralObjGen::~proceduralObjGen()
{
    
}

void proceduralObjGen::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    // Loading plugin
    this -> model = _parent;
    this -> sdf = _sdf;
    this -> world = this -> model -> GetWorld(); // Get pointer to the simulation world

    this -> terrainModel = this -> world -> ModelByName("DEM_100x20_v1"); //Get pointer to the terrain model
    this -> linkTerrain = this -> terrainModel -> GetLink("link_0"); //Get pointer to the terrain link
    physics::CollisionPtr collision_terrain = this -> linkTerrain -> GetCollision(0);
}

void proceduralObjGen::OnUpdate(void)
{

}

bool proceduralObjGen::spawnObj(double x, double y)
{

}

bool proceduralObjGen::getPolygon(double x, double y)
{
    
}





























