/**
* @author Marco Giberna
* @email: marco.giberna@studenti.units.it
* @email: gibimarco@gmail.com
**/

#include "../include/archimede_apply_force.h"

using namespace gazebo;

ArchimedeApplyForce::ArchimedeApplyForce() : ModelPlugin()
{
    //Store the pointer to the model
    if (this -> print_debug_)
    {
        std::cout << "Archimede Apply Force Plugin Destructor called...\n";
    }
}

ArchimedeApplyForce::~ArchimedeApplyForce()
{
    if (this -> print_debug_)
    {
        std::cout << "Archimede Apply Force Plugin Destructor called...\n";
    }
}

void ArchimedeApplyForce::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    this->model = _parent;
    this->sdf = _sdf;
    this->world = this->model->GetWorld();

    if (this->sdf->HasElement("link_name"))
    {
        this->link_name_ = this->sdf->GetElement("link_name")->Get<std::string>();
    } else 
    {
        return;
    }

    if (this->sdf->HasElement("Force"))
    {
        this->commanded_force = this->sdf->GetElement("Force")->Get<ignition::math::Vector3<double>>();
    } else
    {
        this->commanded_force = ignition::math::Vector3d();
    }

    this->link_ = this->model->GetLink(this->link_name_);
    if (this->link_ != nullptr)
    {
        std::cout << "Found Link With Name " << this->link_name_ << std::endl;
    } else
    {
        std::cout << "No Link Found " << this->link_name_ << std::endl;
        return;
    }

    this -> connection = event::Events::ConnectWorldUpdateBegin(std::bind(&ArchimedeApplyForce::OnUpdate, this));

}

void ArchimedeApplyForce::OnUpdate(void)
{
//    std::cout << "Updating" << std::endl;
    this->link_-> AddForce(this->commanded_force);

}