#include <openvr.h>
#include <iostream>

// Main
int main(int argc, char** argv){

  vr::EVRInitError eError = vr::VRInitError_None;
  std::cerr << "a0" << std::endl;
  vr::IVRSystem *pHMD_;
  pHMD_ = vr::VR_Init( &eError, vr::VRApplication_Background );
  //pHMD_ = vr::VR_Init( &eError, vr::VRApplication_Scene );
  
  std::cerr << "a1" << std::endl;
  if (eError != vr::VRInitError_None)
  {
    std::cerr << "b0" << std::endl;
    pHMD_ = NULL;
    //std::string err_msg = "VR_Init Failed. EVRInitError = "+eError;
    std::string err_msg(vr::VR_GetVRInitErrorAsEnglishDescription( eError ));
    std::cerr << err_msg << std::endl;
    return false;
  }

  return 0;
}
