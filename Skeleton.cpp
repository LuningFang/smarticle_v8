#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"


#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono_thirdparty/filesystem/path.h"
#include <cmath>
using namespace chrono;

// collision::ChCollisionSystemType collision_type = collision::ChCollisionSystemType::BULLET;

collision::ChCollisionSystemType collision_type = collision::ChCollisionSystemType::CHRONO;

std::shared_ptr<ChMaterialSurface> DefaultContactMaterial(ChContactMethod contact_method) {
    float mu = 0.37f;   // coefficient of friction

    float cr = 0.9f;   // coefficient of restitution
    float Y = 2e7f;    // Young's modulus
    float nu = 0.3f;   // Poisson ratio

    switch (contact_method) {
        case ChContactMethod::NSC: {
            auto matNSC = chrono_types::make_shared<ChMaterialSurfaceNSC>();
            matNSC->SetFriction(mu);
            matNSC->SetRestitution(cr); // Askah, would you want restitution during contact for lcp solver? 
            return matNSC;
        }
        case ChContactMethod::SMC: {
            auto matSMC = chrono_types::make_shared<ChMaterialSurfaceSMC>();
            matSMC->SetFriction(mu);
            matSMC->SetRestitution(cr);
            matSMC->SetYoungModulus(Y);
            matSMC->SetPoissonRatio(nu);
            return matSMC;
        }
        default:
            return std::shared_ptr<ChMaterialSurface>();
    }
}



// left motor function 
class ChFunction_LeftMotor : public ChFunction {
    public:
   // ChFunction_LeftMotor() : m_speed(CH_C_1_PI/0.4f) {}
       ChFunction_LeftMotor() : m_speed(CH_C_1_PI/4.0f){}

    ChFunction_LeftMotor(double speed): m_speed(speed){}


    virtual ChFunction_LeftMotor* Clone() const override { return new ChFunction_LeftMotor(); }

    // define your own speed function ( x is time, Get_y return speed)
    virtual double Get_y(double x) const override { 

       // int g = int((x+0.2f)/0.4f) % 4;
        int g = int(x / 4.0f) % 4;
        double velo;
        switch (g) {
            case 0:
                return 0;
            case 1:
                return m_speed;
            case 2:
                return 0;
            case 3:
                return -m_speed;
        }
    }


    private:
    double m_speed;
};

class ChFunction_RightMotor : public ChFunction {
    public:
    //ChFunction_RightMotor() : m_speed(CH_C_1_PI/0.4f) {}
    ChFunction_RightMotor() : m_speed(CH_C_1_PI/4.0f){}

    ChFunction_RightMotor(double speed): m_speed(speed){}


    virtual ChFunction_RightMotor* Clone() const override { return new ChFunction_RightMotor(); }

    virtual double Get_y(double x) const override { 

       // int g = int(x/0.4f) % 4;
        int g = int(x / 4.0f) % 4;
        double velo;
        switch (g) {
            case 0:
                return m_speed;
            case 1:
                return 0;
            case 2:
                return -m_speed;
            case 3:
                return 0;
        }
    }


    private:
    double m_speed;
};


class Skeleton{
    public:
        Skeleton(ChVector<double> skeleton_center,
                 double belly_rotation,
                 double alpha1,  
                 double alpha2,  // see Akash paper for how alpha defined definition
                 int id){

            m_skeleton_center = skeleton_center;
            m_belly_rotation = belly_rotation;


            m_left_angle = CH_C_PI - alpha1;
            m_right_angle = alpha2;
            no_contact_family_id = id;        

            m_contact_method = ChContactMethod::SMC; // default smc method

        };

        void Initialize(){


            double mass_small = 0.003186;
            double mass_large = 0.0248;
            ChVector<double> arm_size(0.05, 0.023, 0.003);
            ChVector<double> belly_size(0.054, 0.027, 0.022);

            auto link_mat = DefaultContactMaterial(m_contact_method);

            ChVector<double>  left_motor_pos(-belly_size.x()/2 * std::cos(m_belly_rotation), 0,  belly_size.x()/2 * std::sin(m_belly_rotation));
            ChVector<double> right_motor_pos( belly_size.x()/2 * std::cos(m_belly_rotation), 0, -belly_size.x()/2 * std::sin(m_belly_rotation));



            ChVector<double> arm1_pos(left_motor_pos.x() + arm_size.x()/2.0f * std::cos(m_left_angle + m_belly_rotation), 0, left_motor_pos.z() - arm_size.x()/2.0f * std::sin(m_left_angle + m_belly_rotation));

            // rotate angle of theta+alpha_1 with respect to y axis
            ChQuaternion<double> arm1_rot(Q_from_AngAxis(m_left_angle + m_belly_rotation, VECT_Y));

            // -----------------------------------------------------
            // left arm
            // -----------------------------------------------------
            left_arm = chrono_types::make_shared<ChBodyEasyBox>(arm_size.x(), 
                                                                arm_size.y(), 
                                                                arm_size.z(),  // x,y,z size
                                                                100,        // density
                                                                link_mat,
                                                                collision_type);     // collision?
            left_arm->SetPos(arm1_pos + m_skeleton_center);
            left_arm->SetRot(arm1_rot);
            left_arm->SetBodyFixed(false);
            left_arm->SetCollide(true);
            left_arm->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(no_contact_family_id);
            left_arm->SetNameString(m_skeleton_name + " link1");

            // -----------------------------------------------------
            // center belly
            // -----------------------------------------------------
            ChQuaternion<double> center_body_rotation = Q_from_AngAxis(m_belly_rotation, VECT_Y);
            std::shared_ptr<collision::ChCollisionModel> collision_model2 =
                            chrono_types::make_shared<collision::ChCollisionModelChrono>();

            center_body = chrono_types::make_shared<ChBodyEasyBox>(belly_size.x(), 
                                                                  belly_size.y(), 
                                                                  belly_size.z(),  // x,y,z size
                                                                  100,        // density
                                                                  link_mat,
                                                                  collision_type);     // collision?
            center_body->SetPos(m_skeleton_center);
            center_body->SetRot(center_body_rotation);
            center_body->SetBodyFixed(false);
            center_body->SetMass(mass_large);
            center_body->SetCollide(true);
            center_body->GetCollisionModel()->SetFamily(no_contact_family_id);
            center_body->SetNameString(m_skeleton_name + " link2");



            // -----------------------------------------------------
            // Create a motor between left arm and center body
            // -----------------------------------------------------
            left_motor = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
            left_motor->Initialize(left_arm, center_body, ChFrame<>(left_motor_pos + m_skeleton_center, Q_from_AngAxis(CH_C_PI_2, VECT_X)));
            // initialize motor velocity function, speed of pi/4
            //auto left_motor_velo_func = chrono_types::make_shared<ChFunction_LeftMotor>(CH_C_PI/0.4f);
            auto left_motor_velo_func = chrono_types::make_shared<ChFunction_LeftMotor>(CH_C_PI / 4.0f);
            left_motor->SetSpeedFunction(left_motor_velo_func);


            ChVector<double> arm2_pos(right_motor_pos.x() + arm_size.x()/2.0f * std::cos(m_right_angle + m_belly_rotation), 0, right_motor_pos.z() - arm_size.x()/2.0f * std::sin(m_right_angle + m_belly_rotation));
            ChQuaternion<double> arm2_rot(Q_from_AngAxis(m_right_angle + m_belly_rotation, VECT_Y));

            std::shared_ptr<collision::ChCollisionModel> collision_model3 =
                            chrono_types::make_shared<collision::ChCollisionModelChrono>();

            // -----------------------------------------------------
            // right arm
            // -----------------------------------------------------
            right_arm = chrono_types::make_shared<ChBodyEasyBox>(arm_size.x(), 
                                                                arm_size.y(), 
                                                                arm_size.z(),  // x,y,z size
                                                                100,        // density
                                                                link_mat,
                                                                collision_type);     // collision?
            right_arm->SetPos(arm2_pos + m_skeleton_center);
            right_arm->SetRot(arm2_rot);
            right_arm->SetBodyFixed(false);  // the truss does not move!
            right_arm->SetMass(mass_small);
            right_arm->SetCollide(true);
            right_arm->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(no_contact_family_id);
            right_arm->SetNameString(m_skeleton_name + " link3");

            // -----------------------------------------------------
            // Create a motor between right arm and center body
            // -----------------------------------------------------
            right_motor = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
            right_motor->Initialize(right_arm, center_body, ChFrame<>(right_motor_pos + m_skeleton_center, Q_from_AngAxis(-CH_C_PI_2, VECT_X)));

           // auto right_motor_velo_func = chrono_types::make_shared<ChFunction_RightMotor>(CH_C_PI/0.4f);
            auto right_motor_velo_func = chrono_types::make_shared<ChFunction_RightMotor>(CH_C_PI / 4.0f);
            right_motor->SetSpeedFunction(right_motor_velo_func);

        };

        void AddSkeleton(ChSystem* sys){
            sys->AddBody(left_arm);
            sys->AddBody(center_body);
            sys->AddBody(right_arm);

            sys->AddLink(left_motor);            
            sys->AddLink(right_motor);

        }
    
        void SetContactMethod(ChContactMethod contact_method){
            m_contact_method = contact_method;
        }

        void SetSkeletonName(std::string name){
            m_skeleton_name = name;
        }

        ChVector<double> GetPos(){
            return center_body->GetPos();
        };

        std::shared_ptr<ChLinkMotorRotationSpeed> GetRightMotor(){return right_motor;};

        std::shared_ptr<ChLinkMotorRotationSpeed> GetLeftMotor(){return left_motor;};

        // return alpha1 angle 
        double GetAlpha1() {return left_motor->GetMotorRot();};

        // return alpha2 angle 
        double GetAlpha2() {return right_motor->GetMotorRot();};

        // return theta
        double GetTheta() {
            // get the X column of orientation matrix A of center body
            ChVector<double> x_axis = center_body->GetA().Get_A_Xaxis();
            double theta = std::acos(x_axis.x());
            if (x_axis.z() < 0){
                theta = - theta;
            }
            return theta;
        }

        std::shared_ptr<ChBodyEasyBox> GetLeftArm(){return left_arm;};
        std::shared_ptr<ChBodyEasyBox> GetRightArm(){return right_arm;};
        std::shared_ptr<ChBodyEasyBox> GetBelly(){return center_body;};
        

    private:
        ChVector<double> m_skeleton_center;
        double m_belly_rotation;
        double m_left_angle;
        double m_right_angle;
        int no_contact_family_id;
        std::string m_skeleton_name; // skeleton name
        std::shared_ptr<ChBodyEasyBox> left_arm;
        std::shared_ptr<ChBodyEasyBox> center_body;
        std::shared_ptr<ChBodyEasyBox> right_arm;
        std::shared_ptr<ChLinkMotorRotationSpeed> left_motor;
        std::shared_ptr<ChLinkMotorRotationSpeed> right_motor;
        ChContactMethod m_contact_method;

};

