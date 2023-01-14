// =============================================================================
// Initialize Smarticles from Command Line
// Input from text file
#include "chrono/core/ChStream.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#include "chrono/collision/ChCollisionSystemChrono.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono_thirdparty/filesystem/path.h"
#include "Skeleton.cpp"
#include "chrono/core/ChVector.h"
#include <cmath>
#include <sstream>
using namespace chrono;
using namespace chrono::irrlicht;

using namespace irr;
using namespace irr::core;

// -----------------------------------------------------------------------------
// Callback class for contact reporting
// -----------------------------------------------------------------------------
class ContactReporter : public ChContactContainer::ReportContactCallback {
  public:
    ContactReporter(){
        // initialize output stream
        txt.stream().setf(std::ios::scientific | std::ios::showpos);
        txt.stream().precision(8);
    }

    bool WriteContactInfo(std::string filename){
        // size of txt output stream
        int size = txt.stream().str().length();

        // only write to file if there is arm-arm contact
        if (size > 0){
            txt.write_to_file(filename);            
        }
        return true;
    }

  private:
    virtual bool OnReportContact(const ChVector<>& pA,  // contact point on body A (global)
                                 const ChVector<>& pB,  // contact point on body B 
                                 const ChMatrix33<>& plane_coord,
                                 const double& distance,
                                 const double& eff_radius,
                                 const ChVector<>& cforce,
                                 const ChVector<>& ctorque,
                                 ChContactable* modA,
                                 ChContactable* modB) override {
        // Check if contact involves box1
        // if (modA == m_box1.get() && modB == m_box2.get()) {
            std::string bodyA_name = modA->GetPhysicsItem()->GetNameString();
            if (cforce.Length() > 1e-8 && bodyA_name != "ground 0" ){
                txt << modA->GetPhysicsItem()->GetNameString();
                txt << modB->GetPhysicsItem()->GetNameString();

                txt << pA;
                txt << pB;


                const ChVector<>& nrm = plane_coord.Get_A_Xaxis();
                txt << nrm;
                ChVector<double> global_force = plane_coord * cforce;

                txt << global_force;
                txt << std::endl;
            }

        return true;
    }
    utils::CSV_writer txt{" "};
};


void AddContainer(ChSystem* sys) {

    ChContactMethod contact_method = sys->GetContactMethod();

    // Contact and visualization materials for container
    auto ground_mat = DefaultContactMaterial(contact_method);;
    auto ground_mat_vis = chrono_types::make_shared<ChVisualMaterial>(*ChVisualMaterial::Default());
    ground_mat_vis->SetKdTexture(GetChronoDataFile("textures/concrete.jpg"));


    // Create the floor of the rectangular container, using fixed rigid bodies of 'box' type
    auto floorBody = chrono_types::make_shared<ChBodyEasyBox>(5, 0.02, 5, 1000, ground_mat, collision_type);
    floorBody->SetPos(ChVector<>(0, -0.025, 0));
    floorBody->SetBodyFixed(true);
    floorBody->GetVisualShape(0)->SetMaterial(0, ground_mat_vis);
    floorBody->SetCollide(true);
    floorBody->SetNameString("ground 0");
    sys->Add(floorBody);


}



int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";
    SetChronoDataPath(CHRONO_DATA_DIR);

    bool useSMC;
    double S1_pos_x, S1_pos_z, S1_theta, S2_pos_x, S2_pos_z, S2_theta;

    if (argc != 4){
        std::cout << "useage: ./main <0/1 - NSC/SMC> + <S1 pos_x,y,theta, S2 pos_x,y,theta> + <folder/subfolder> \n";
        return 0;
    }

    useSMC = atoi(argv[1]);

    // parse initial position 
    std::stringstream ss(argv[2]);
    std::vector<double> initial_pos;
    for (double i; ss >> i;){
        initial_pos.push_back(i);    
        if (ss.peek() == ',')
            ss.ignore();
    }

    S1_pos_x = initial_pos[0];
    S1_pos_z = initial_pos[1];
    S1_theta = initial_pos[2];
    S2_pos_x = initial_pos[3];
    S2_pos_z = initial_pos[4];
    S2_theta = initial_pos[5];
    
    // parse output folder string, assuming in the format of folder/subfolder
    std::string filenames = argv[3];
    if (filesystem::create_subdirectory(filesystem::path(filenames))) {
        std::cout << "  ...Created nested subdirectories" << std::endl;
    } else {
        std::cout << "  ...Error creating subdirectories" << std::endl;
        return 1;        
    }
    // std::string output_folder_name = filenames.substr(0, filenames.find('/'));
    // std::string subfolder_name = filenames.substr(filenames.find('/')+1, filenames.size());

    // std::cout << "folder " << output_folder_name << ", subfolder: " << subfolder_name << std::endl;

    // create folder and subfolder if they do not exist 
    // filesystem::create_directory(filesystem::path(output_folder_name));
    // output_folder_name = output_folder_name + '/' + subfolder_name;
    // filesystem::create_directory(filesystem::path(output_folder_name));


    ChSystem *sys;
    double step_size;

    if (useSMC){
        ChSystemSMC *my_sys = new ChSystemSMC();
        sys = my_sys;
        step_size = 1e-4;
    } else {
        ChSystemNSC *my_sys = new ChSystemNSC();
        sys = my_sys;
        //step_size = 1e-3;    
        step_size = 1e-4;
    }

    sys->SetCollisionSystemType(collision_type);


    // double time_end = 16*30;
    // time for testing
    // time : 480sec
    double time_end = 3;

    AddContainer(sys);

    // ChVector<float> gravity(0, 0, 9.81);
    ChVector<float> gravity(0, -9.81, 0);
    sys->Set_G_acc(gravity);
    // double step_size = 1e-3; // 1e-3 in world.skel which one?

    // actuator (type: servo not velocity)
    // force uppser limit (0, 3.0e-2)
    // force lower limit (0, -3.0e-2)
    // position limit enforced
    // point limit: angle: (0, 1.5708 to 0, -1.5708) 

    // Parameters;
    // location of the belly at -0.02, 0, -0.006
    // theta, oritentation of the belly
    // alpha1 and alpha2
    // body id, make sure it's different for every body
    // 
   // Skeleton skeleton1(ChVector<double>(-0.02f, 0.0f, -0.006f), 0, -CH_C_PI_2, -CH_C_PI_2, 11);
    //Skeleton skeleton1(ChVector<double>(0.00f, 0.0f, 0.0f), 3.001966e+00, -CH_C_PI_2, -CH_C_PI_2, 11);
    //Skeleton skeleton1(ChVector<double>(0.00f, 0.0f, 0.0f), 7.330383e-01, -CH_C_PI_2, -CH_C_PI_2, 11);
    // Skeleton skeleton1(ChVector<double>(0.00f, 0.0f, 0.00f), -1.954769e+00, -CH_C_PI_2, -CH_C_PI_2, 11);
    Skeleton skeleton1(ChVector<double>(S1_pos_x, 0.0f, S1_pos_z), S1_theta, -CH_C_PI_2, -CH_C_PI_2, 11);


    skeleton1.SetContactMethod(sys->GetContactMethod());
    skeleton1.SetSkeletonName("skel1");
    skeleton1.Initialize();
    
    // Add body skeleton 1 to sys
    skeleton1.AddSkeleton(sys);


    // Test one skeleton first ... then writes API that sets phase shift
    //Skeleton skeleton2(ChVector<double>(0.02, 0.0f, 0.06f), CH_C_PI, -CH_C_PI_2, -CH_C_PI_2, 12);
    //Skeleton skeleton2(ChVector<double>(-2.967361e-02, 0.0f, -6.908784e-02), 6.143559e+00, -CH_C_PI_2, -CH_C_PI_2, 12);
   // Skeleton skeleton2(ChVector<double>(7.199819e-02, 0.0f, 2.167750e-02), 3.874631e+00, -CH_C_PI_2, -CH_C_PI_2, 12);  // Unstable with NSC Stable with SMC
    // Skeleton skeleton2(ChVector<double>(-7.421433e-02, 0.0f, 1.207832e-02), 1.186824e+00, -CH_C_PI_2, -CH_C_PI_2, 12); 
    Skeleton skeleton2(ChVector<double>(S2_pos_x, 0.0f, S2_pos_z), S2_theta, -CH_C_PI_2, -CH_C_PI_2, 12); 

    skeleton2.SetContactMethod(sys->GetContactMethod());
    skeleton2.SetSkeletonName("skel2");
    skeleton2.Initialize();
    skeleton2.AddSkeleton(sys);

    // Create the Irrlicht visualization system
    /*
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(sys);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("smarticle test");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector<>(0, 0.5, 0));
    vis->AddTypicalLights();
    */
    int frame = 0;

    // initialize the csv writer to write the output file
    utils::CSV_writer txt(" ");
    txt.stream().setf(std::ios::scientific | std::ios::showpos);
    txt.stream().precision(8);

    int output_freq = 200;
    // while (vis->Run()) {
    while (true){
        // visualization
        // compute dynamics
        sys->DoStepDynamics(step_size);

        /*
        if (frame % int(output_frame_step/step_size) == 0){
            char png_filename[100];
            sprintf(png_filename, "img_%01d.jpg", int(frame/100));
            vis->WriteImageToFile(png_filename);

        }
        */

        if (frame % output_freq == 0){
            /*
            vis->BeginScene();
            vis->Render();
            vis->ShowInfoPanel(true);
            vis->EndScene();
            */
            txt << sys->GetChTime() << skeleton1.GetPos() << skeleton1.GetTheta() << skeleton1.GetAlpha1() << skeleton1.GetAlpha2();


            txt << skeleton2.GetPos() << skeleton2.GetTheta() << skeleton2.GetAlpha1() << skeleton2.GetAlpha2();

            txt << std::endl;

            auto creporter = chrono_types::make_shared<ContactReporter>();
            sys->GetContactContainer()->ReportAllContacts(creporter);
            char contact_file_name[500];
            if (useSMC){
                sprintf(contact_file_name, "%sSMC_contact%06d.txt", filenames.c_str(), int(frame/output_freq));
            }
            else {
                sprintf(contact_file_name, "%sNSC_contact%06d.txt", filenames.c_str(), int(frame/output_freq));
            }

            creporter->WriteContactInfo(std::string(contact_file_name));



        }

        frame++;


        if (sys->GetChTime()>time_end){

            break;
        }

    }

    std::string txt_output;
    if (useSMC){
        txt_output = "SMC_coordinates.txt";
    }
    else {
        txt_output = "NSC_coordinates.txt";

    }
    txt.write_to_file(filenames + '/' + txt_output);


    return 0;
}
