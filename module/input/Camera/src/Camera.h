#ifndef MODULE_INPUT_CAMERA_H
#define MODULE_INPUT_CAMERA_H

#include <stdexcept>
#include <system_error>
#include <string>
#include <sstream>

#include <fcntl.h>
#include <linux/videodev2.h>
#include <jpeglib.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <Spinnaker.h>
#include <SpinGenApi/SpinnakerGenApi.h>

#include <nuclear>

#include "extension/Configuration.h"

#include "message/input/Image.h"
#include "message/input/CameraParameters.h"

#include "utility/vision/fourcc.h"

#include "V4L2Camera.h"
#include "SpinnakerCamera.h"

namespace module
{
	namespace input
	{

	    class Camera : public NUClear::Reactor {

	    public:
	        /// @brief Called by the powerplant to build and setup the Camera reactor.
	        explicit Camera(std::unique_ptr<NUClear::Environment> environment);

	    private:
	    	// V4L2 Camera details
	    	V4L2Camera initiateV4L2Camera(const ::extension::Configuration& config);
			void ShutdownV4L2Camera();

	        ReactionHandle V4L2FrameRateHandle; 
	        ReactionHandle V4L2SettingsHandle; 

	        std::map<std::string, V4L2Camera> V4L2Cameras;


	    	// Spinnaker Camera details
		    void initiateSpinnakerCamera(const ::extension::Configuration& config);
	        void resetSpinnakerCamera(std::map<std::string, std::unique_ptr<SpinnakerImageEvent>>::iterator& camera, const ::extension::Configuration& config);
	        void ShutdownSpinnakerCamera();

	        Spinnaker::SystemPtr SpinnakerSystem;
	        Spinnaker::CameraList SpinnakerCamList;
	        std::map<std::string, std::unique_ptr<SpinnakerImageEvent>> SpinnakerCameras;

	        static uint cameraCount;
	    };
	}
}

#endif  // MODULE_INPUT_CAMERA_H
