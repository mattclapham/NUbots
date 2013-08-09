/*! @file DarwinCamera.cpp
    @brief Implementation of Darwin camera Reactor class

    @author Jason Kulk

  Copyright (c) 2009 Jason Kulk

 This file is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This file is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with NUbot.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "DarwinCamera.h"

#include <cstring>
#include <sstream>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <cerrno>
#include <cstdlib>
#include <unistd.h> /* Mitchell Metcalfe, 31-01-13, for GCC4.7, Ubuntu 12.10 compatibility */

#undef __STRICT_ANSI__
#include <linux/videodev2.h>
#include <linux/version.h>
//#include <linux/i2c-dev.h>
#define __STRICT_ANSI__

#ifndef V4L2_CID_AUTOEXPOSURE
#  define V4L2_CID_AUTOEXPOSURE     (V4L2_CID_BASE+32)
#endif

#ifndef V4L2_CID_CAM_INIT
#  define V4L2_CID_CAM_INIT         (V4L2_CID_BASE+33)
#endif

#ifndef V4L2_CID_AUDIO_MUTE
#  define V4L2_CID_AUDIO_MUTE       (V4L2_CID_BASE+9)
#endif
/*
#ifndef V4L2_CID_POWER_LINE_FREQUENCY
#  define V4L2_CID_POWER_LINE_FREQUENCY  (V4L2_CID_BASE+24)
enum v4l2_power_line_frequency {
  V4L2_CID_POWER_LINE_FREQUENCY_DISABLED  = 0,
  V4L2_CID_POWER_LINE_FREQUENCY_50HZ  = 1,
  V4L2_CID_POWER_LINE_FREQUENCY_60HZ  = 2,
};

#define V4L2_CID_HUE_AUTO      (V4L2_CID_BASE+25)
#define V4L2_CID_WHITE_BALANCE_TEMPERATURE  (V4L2_CID_BASE+26)
        #define V4L2_CID_SHARPNESS      (V4L2_CID_BASE+27)
#define V4L2_CID_BACKLIGHT_COMPENSATION   (V4L2_CID_BASE+28)

#define V4L2_CID_CAMERA_CLASS_BASE     (V4L2_CTRL_CLASS_CAMERA | 0x900)
#define V4L2_CID_CAMERA_CLASS       (V4L2_CTRL_CLASS_CAMERA | 1)

#define V4L2_CID_EXPOSURE_AUTO      (V4L2_CID_CAMERA_CLASS_BASE+1)
enum  v4l2_exposure_auto_type {
  V4L2_EXPOSURE_MANUAL = 0,
  V4L2_EXPOSURE_AUTO = 1,
  V4L2_EXPOSURE_SHUTTER_PRIORITY = 2,
  V4L2_EXPOSURE_APERTURE_PRIORITY = 3
};
#define V4L2_CID_EXPOSURE_ABSOLUTE    (V4L2_CID_CAMERA_CLASS_BASE+2)
#define V4L2_CID_EXPOSURE_AUTO_PRIORITY    (V4L2_CID_CAMERA_CLASS_BASE+3)

#define V4L2_CID_PAN_RELATIVE      (V4L2_CID_CAMERA_CLASS_BASE+4)
#define V4L2_CID_TILT_RELATIVE      (V4L2_CID_CAMERA_CLASS_BASE+5)
#define V4L2_CID_PAN_RESET      (V4L2_CID_CAMERA_CLASS_BASE+6)
#define V4L2_CID_TILT_RESET      (V4L2_CID_CAMERA_CLASS_BASE+7)

#define V4L2_CID_PAN_ABSOLUTE      (V4L2_CID_CAMERA_CLASS_BASE+8)
#define V4L2_CID_TILT_ABSOLUTE      (V4L2_CID_CAMERA_CLASS_BASE+9)

#define V4L2_CID_FOCUS_ABSOLUTE      (V4L2_CID_CAMERA_CLASS_BASE+10)
#define V4L2_CID_FOCUS_RELATIVE      (V4L2_CID_CAMERA_CLASS_BASE+11)
#define V4L2_CID_FOCUS_AUTO      (V4L2_CID_CAMERA_CLASS_BASE+12)

#endif // LINUX_VERSION_CODE < KERNEL_VERSION(2,6,26)
*/

namespace modules {
    
/*
	namespace
	{
		std::string controlName(unsigned int id)
		{
			std::string name;
			switch(id)
			{
			case V4L2_CID_BRIGHTNESS:
				name = "Brightness";
				break;
			case V4L2_CID_CONTRAST:
				name = "Contrast";
				break;
			case V4L2_CID_SATURATION:
				name = "Saturation";
				break;
			case V4L2_CID_AUTO_WHITE_BALANCE:
				name = "Auto White Balance";
				break;
			case V4L2_CID_POWER_LINE_FREQUENCY:
				name = "Power Line Frequency";
				break;
			case V4L2_CID_WHITE_BALANCE_TEMPERATURE:
				name = "White Balance Temperature";
				break;
			case V4L2_CID_SHARPNESS:
				name = "Sharpness";
				break;
			case V4L2_CID_EXPOSURE_AUTO:
				name = "Exposure Auto";
				break;
			case V4L2_CID_EXPOSURE_ABSOLUTE:
				name = "Absolute Exposure";
				break;
			case V4L2_CID_EXPOSURE_AUTO_PRIORITY:
				name = "Exposure Auto Priority";
				break;
			case V4L2_CID_GAIN:
				name = "Gain";
				break;
			default:
				name = "Unknown";
			}
			return name;
		}
	}
*/

    DarwinCamera::DarwinCamera(NUClear::PowerPlant& plant):
		Reactor(plant),
		currentBuf(nullptr),
		timeStamp(0)
	{
        on<Trigger<Every<1000 / FRAMERATE, std::chrono::milliseconds>>>([this](const time_t& time)
		{
			emit(grabNewImage());
        });

		on<Trigger<Shutdown>>([this](const Shutdown& shutdown)
		{
#if DEBUG_NUCAMERA_VERBOSITY > 4
			debug << "DarwinCamera::~DarwinCamera()" << endl;
#endif
			// disable streaming
			setStreaming(false);

			// unmap buffers
			for(int i = 0; i < frameBufferCount; ++i)
				munmap(mem[i], memLength[i]);

			// close the device
			close(fd);
			free(buf);
		});

		on<Trigger<messages::CameraSettings>>([this](const messages::CameraSettings& settings)
		{
			applySettings(settings);
		});

#if DEBUG_NUCAMERA_VERBOSITY > 4
		debug << "DarwinCamera::DarwinCamera()" << endl;
#endif
		storedTimeStamp = std::chrono::duration_cast<std::chrono::milliseconds>(NUClear::clock::now().time_since_epoch()).count();


		// Read camera settings from file.
		/*
		CameraSettings fileSettings;
		fileSettings.LoadFromFile(CONFIG_DIR + string("Camera.cfg"));
		debug << "Loading settings from " << CONFIG_DIR + string("Camera.cfg") << endl;
		*/

		// Open device
		openCameraDevice("/dev/video0");

		//Initialise
		initialiseCamera();
		//readCameraSettings();
		//forceApplySettings(fileSettings);

		readCameraSettings();

		//loadCameraOffset();

		// enable streaming
		setStreaming(true);
    }

	DarwinCamera::~DarwinCamera()
	{
	}

	void DarwinCamera::openCameraDevice(const std::string& device_name)
	{
		// open device
		fd = open(device_name.c_str(), O_RDWR);
#if DEBUG_NUCAMERA_VERBOSITY > 4
		if(fd != -1)
		{
			debug << "DarwinCamera::DarwinCamera(): " << device_name << " Opened Successfully." << endl;
		}
		else {
			debug << "DarwinCamera::DarwinCamera(): " << device_name << " Could Not Be Opened: " << strerror(errno) << endl;
		}
#endif
	}

	void DarwinCamera::setStreaming(bool streaming_on)
	{
		int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		int instruction = streaming_on ? VIDIOC_STREAMON: VIDIOC_STREAMOFF;
		if(ioctl(fd, instruction, &type) == -1)
			throw std::exception();

		//debug << "DarwinCamera: streaming - " << streaming_on << endl;
	}

	void DarwinCamera::initialiseCamera()
	{
		int returnValue;
		// set default parameters
		struct v4l2_control control;
		memset(&control, 0, sizeof(control));
		control.id = V4L2_CID_CAM_INIT;
		control.value = 0;
		if(ioctl(fd, VIDIOC_S_CTRL, &control) < 0)
			throw std::exception();

		v4l2_std_id esid0 = (WIDTH == 320 ? 0x04000000UL : 0x08000000UL);
		returnValue = ioctl(fd, VIDIOC_S_STD, &esid0);

#if DEBUG_NUCAMERA_VERBOSITY > 4
		if(returnValue)
		{
			debug << "DarwinCamera::DarwinCamera(): Error Setting Video Mode: " << strerror(errno) << endl;
		}
		else
		{
			debug << "DarwinCamera::DarwinCamera(): Video Mode set to " << (WIDTH == 320 ? "QVGA" : "VGA") << endl;
		}
#else
		if(returnValue)
			throw std::exception();
#endif

		// set format
		struct v4l2_format fmt;
		memset(&fmt, 0, sizeof(fmt));
		fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		fmt.fmt.pix.width = WIDTH;
		fmt.fmt.pix.height = HEIGHT;
		fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
		fmt.fmt.pix.field = V4L2_FIELD_NONE;

		returnValue = ioctl(fd, VIDIOC_S_FMT, &fmt);
#if DEBUG_NUCAMERA_VERBOSITY > 4
		if(returnValue)
		{
			debug << "DarwinCamera::DarwinCamera(): Error Setting Format: " << strerror(errno) << endl;
		}
		else
		{
			debug << "DarwinCamera::DarwinCamera(): Format set" << endl;
		}
#else
		if(returnValue)
			throw std::exception();
#endif

		if(fmt.fmt.pix.sizeimage != SIZE)
			throw std::exception();

		// set frame rate
		struct v4l2_streamparm fps;
		memset(&fps, 0, sizeof(fps));
		fps.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		if(ioctl(fd, VIDIOC_G_PARM, &fps))
			throw std::exception();
		fps.parm.capture.timeperframe.numerator = 1;
		fps.parm.capture.timeperframe.denominator = FRAMERATE;
		if(ioctl(fd, VIDIOC_S_PARM, &fps) == -1)
			throw std::exception();

		// request buffers
		struct v4l2_requestbuffers rb;
		memset(&rb, 0, sizeof(rb));
		rb.count = frameBufferCount;
		rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		rb.memory = V4L2_MEMORY_MMAP;
		if(ioctl(fd, VIDIOC_REQBUFS, &rb) == -1)
			throw std::exception();

		// map the buffers
		buf = static_cast<struct v4l2_buffer*>(calloc(1, sizeof(struct v4l2_buffer)));
		for(int i = 0; i < frameBufferCount; ++i)
		{
			buf->index = i;
			buf->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			buf->memory = V4L2_MEMORY_MMAP;
			if(ioctl(fd, VIDIOC_QUERYBUF, buf) == -1)
				throw std::exception();
			memLength[i] = buf->length;
			mem[i] = mmap(0, buf->length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf->m.offset);
			if(mem[i] == MAP_FAILED)
				throw std::exception();
		}

		// queue the buffers
		for(int i = 0; i < frameBufferCount; ++i)
		{
			buf->index = i;
			buf->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			buf->memory = V4L2_MEMORY_MMAP;
			if(ioctl(fd, VIDIOC_QBUF, buf) == -1)
				throw std::exception();
		}
	}

	bool DarwinCamera::capturedNew()
	{
		// requeue the buffer of the last captured image which is obselete now
		if(currentBuf && ioctl(fd, VIDIOC_QBUF, currentBuf) == -1)
			throw std::exception();

		// dequeue a frame buffer (this call blocks when there is no new image available) */
		if(ioctl(fd, VIDIOC_DQBUF, buf) == -1)
			throw std::exception();

		if(buf->bytesused != SIZE)
			throw std::exception();

		currentBuf = buf;
		timeStamp = storedTimeStamp + 33.0;
		storedTimeStamp = std::chrono::duration_cast<std::chrono::milliseconds>(NUClear::clock::now().time_since_epoch()).count();
		return true;
	}

	const unsigned char* DarwinCamera::getImage() const
	{
		if(currentBuf == nullptr)
			throw std::exception();
		return static_cast<unsigned char*>(mem[currentBuf->index]);
	}

	double DarwinCamera::getTimeStamp() const
	{
		if(currentBuf == nullptr)
			throw std::exception();
		return timeStamp;
	}

	messages::NUImage* DarwinCamera::grabNewImage()
	{
		while(!capturedNew());
		currentBufferedImage.MapYUV422BufferToImage(getImage(), WIDTH, HEIGHT, true);
		currentBufferedImage.setTimestamp(getTimeStamp());
		currentBufferedImage.setCameraSettings(settings);
		return &currentBufferedImage;
	}

	void DarwinCamera::readCameraSettings()
	{
		settings.exposureAuto               = readSetting(V4L2_CID_EXPOSURE_AUTO);
		settings.autoWhiteBalance           = readSetting(V4L2_CID_AUTO_WHITE_BALANCE);
		settings.whiteBalanceTemperature    = readSetting(V4L2_CID_WHITE_BALANCE_TEMPERATURE);
		settings.exposureAutoPriority       = readSetting(V4L2_CID_EXPOSURE_AUTO_PRIORITY);
		settings.brightness                 = readSetting(V4L2_CID_BRIGHTNESS);
		settings.contrast                   = readSetting(V4L2_CID_CONTRAST);
		settings.saturation                 = readSetting(V4L2_CID_SATURATION);
		settings.gain                       = readSetting(V4L2_CID_GAIN);
		settings.exposureAbsolute           = readSetting(V4L2_CID_EXPOSURE_ABSOLUTE);
		settings.powerLineFrequency         = readSetting(V4L2_CID_POWER_LINE_FREQUENCY);
		settings.sharpness                  = readSetting(V4L2_CID_SHARPNESS);

		settings.p_exposureAuto.set(settings.exposureAuto);
		settings.p_autoWhiteBalance.set(settings.autoWhiteBalance);
		settings.p_whiteBalanceTemperature.set(settings.whiteBalanceTemperature);
		settings.p_exposureAutoPriority.set(settings.exposureAutoPriority);
		settings.p_brightness.set(settings.brightness);
		settings.p_contrast.set(settings.contrast);
		settings.p_saturation.set(settings.saturation);
		settings.p_gain.set(settings.gain);
		settings.p_exposureAbsolute.set(settings.exposureAbsolute);
		settings.p_powerLineFrequency.set(settings.powerLineFrequency);
		settings.p_sharpness.set(settings.sharpness);

#if DEBUG_NUCAMERA_VERBOSITY > 1
		debug << "DarwinCamera::readCameraSettings()" << endl;
		debug << "\texposureAuto " << m_settings.exposureAuto  << endl;
		debug << "autoWhiteBalance " << m_settings.autoWhiteBalance  << endl;
		debug << "whiteBalanceTemperature " << m_settings.whiteBalanceTemperature  << endl;
		debug << "exposureAutoPriority " << m_settings.exposureAutoPriority  << endl;
		debug << "brightness " << m_settings.brightness  << endl;
		debug << "contrast " << m_settings.contrast  << endl;
		debug << "saturation " << m_settings.saturation  << endl;
		debug << "gain " << m_settings.gain  << endl;
		debug << "exposureAbsolute " << m_settings.exposureAbsolute  << endl;
		debug << "powerLineFrequency " << m_settings.powerLineFrequency  << endl;
		debug << "sharpness " << m_settings.sharpness  << endl;
#endif
	}

	int DarwinCamera::readSetting(unsigned int id)
	{
		struct v4l2_queryctrl queryctrl;
		queryctrl.id = id;
		if(ioctl(fd, VIDIOC_QUERYCTRL, &queryctrl) < 0)
			return -1;
		if(queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)
			return -1; // not available
		if(queryctrl.type != V4L2_CTRL_TYPE_BOOLEAN && queryctrl.type != V4L2_CTRL_TYPE_INTEGER && queryctrl.type != V4L2_CTRL_TYPE_MENU)
			return -1; // not supported

		struct v4l2_control control_s;
		control_s.id = id;
		if(ioctl(fd, VIDIOC_G_CTRL, &control_s) < 0)
			return -1;
		if(control_s.value == queryctrl.default_value)
			return -1;
		return control_s.value;
	}

	bool DarwinCamera::applySetting(unsigned int id, int value)
	{
		//debug << "DarwinCamera: Trying: " << controlName(id) << " -> " << value << std::endl;
		struct v4l2_queryctrl queryctrl;
		queryctrl.id = id;
		if(ioctl(fd, VIDIOC_QUERYCTRL, &queryctrl) < 0)
			return false;
		if(queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)
			return false; // not available
		if(queryctrl.type != V4L2_CTRL_TYPE_BOOLEAN && queryctrl.type != V4L2_CTRL_TYPE_INTEGER && queryctrl.type != V4L2_CTRL_TYPE_MENU)
			return false; // not supported
		// clip value
		if(value < queryctrl.minimum)
			value = queryctrl.minimum;
		if(value > queryctrl.maximum)
			value = queryctrl.maximum;

		struct v4l2_control control_s;
		control_s.id = id;
		control_s.value = value;
		if(ioctl(fd, VIDIOC_S_CTRL, &control_s) < 0)
			return false;
		//debug << "DarwinCamera: Apply: " << controlName(id) << " -> " << value << " (" << queryctrl.minimum << "," << queryctrl.maximum << "," << queryctrl.default_value << ")" << std::endl;
		return true;
	}

	void DarwinCamera::applySettings(const messages::CameraSettings& newset)
	{
		// AutoSettings
		if(newset.p_exposureAuto.get() != settings.p_exposureAuto.get())
		{
			settings.p_exposureAuto.set(newset.p_exposureAuto.get());
			applySetting(V4L2_CID_EXPOSURE_AUTO, (settings.p_exposureAuto.get()));
		}
		if(newset.p_autoWhiteBalance.get() != settings.p_autoWhiteBalance.get())
		{
			settings.p_autoWhiteBalance.set(newset.p_autoWhiteBalance.get());
			applySetting(V4L2_CID_AUTO_WHITE_BALANCE, (settings.p_autoWhiteBalance.get()));
		}
		if(newset.p_whiteBalanceTemperature.get() != settings.p_whiteBalanceTemperature.get())
		{
			settings.p_whiteBalanceTemperature.set(newset.p_whiteBalanceTemperature.get());
			applySetting(V4L2_CID_WHITE_BALANCE_TEMPERATURE, (settings.p_whiteBalanceTemperature.get()));
		}
		if(newset.p_exposureAutoPriority.get() != settings.p_exposureAutoPriority.get())
		{
			settings.p_exposureAutoPriority.set(newset.p_exposureAutoPriority.get());
			applySetting(V4L2_CID_EXPOSURE_AUTO_PRIORITY, (settings.p_exposureAutoPriority.get()));
		}

		//Other controls
		if(newset.p_brightness.get() != settings.p_brightness.get())
		{
			settings.p_brightness.set(newset.p_brightness.get());
			applySetting(V4L2_CID_BRIGHTNESS, (settings.p_brightness.get()));
		}
		if(newset.p_contrast.get() != settings.p_contrast.get())
		{
			settings.p_contrast.set(newset.p_contrast.get());
			applySetting(V4L2_CID_CONTRAST, (settings.p_contrast.get()));
		}
		if(newset.p_saturation.get() != settings.p_saturation.get())
		{
			settings.p_saturation.set(newset.p_saturation.get());
			applySetting(V4L2_CID_SATURATION, (settings.p_saturation.get()));
		}
		if(newset.p_gain.get() != settings.p_gain.get())
		{
			settings.p_gain.set(newset.p_gain.get());
			applySetting(V4L2_CID_GAIN, (settings.p_gain.get()));
		}
		if(newset.p_exposureAbsolute.get() != settings.p_exposureAbsolute.get())
		{
			settings.p_exposureAbsolute.set(newset.p_exposureAbsolute.get());
			applySetting(V4L2_CID_EXPOSURE_ABSOLUTE, (settings.p_exposureAbsolute.get()));
		}

		if(newset.p_powerLineFrequency.get() != settings.p_powerLineFrequency.get())
		{
			settings.p_powerLineFrequency.set(newset.p_powerLineFrequency.get());
			applySetting(V4L2_CID_POWER_LINE_FREQUENCY, (settings.p_powerLineFrequency.get()));
		}
		if(newset.p_sharpness.get() != settings.p_sharpness.get())
		{
			settings.p_sharpness.set(newset.p_sharpness.get());
			applySetting(V4L2_CID_SHARPNESS, (settings.p_sharpness.get()));
		}

		//COPIES INTO OLD FORMAT:
		settings.copyParams();
	}

	void DarwinCamera::forceApplySettings(const messages::CameraSettings& newset)
	{
#if DEBUG_NUCAMERA_VERBOSITY > 1
		//Copying the new Paramters into m_settings
		debug << "p_exposureAuto" << newset.p_exposureAuto.get() << endl;
		debug << "p_autoWhiteBalance" << newset.p_autoWhiteBalance.get() << endl;
		debug << "p_whiteBalanceTemperature" << newset.p_whiteBalanceTemperature.get() << endl;
		debug << "p_exposureAutoPriority" << newset.p_exposureAutoPriority.get() << endl;
		debug << "p_brightness" << newset.p_brightness.get() << endl;
		debug << "p_contrast" << newset.p_contrast.get() << endl;
		debug << "p_saturation" << newset.p_saturation.get() << endl;
		debug << "p_gain" << newset.p_gain.get() << endl;
		debug << "p_exposureAbsolute" << newset.p_exposureAbsolute.get() << endl;
		debug << "p_powerLineFrequency" << newset.p_powerLineFrequency.get() << endl;
		debug << "p_sharpness" << newset.p_sharpness.get() << endl;
#endif
		//Copying the new Paramters into m_settings
		settings = newset;

		// Auto Controls
		applySetting(V4L2_CID_EXPOSURE_AUTO, (settings.p_exposureAuto.get()));
		applySetting(V4L2_CID_AUTO_WHITE_BALANCE, (settings.p_autoWhiteBalance.get()));
		applySetting(V4L2_CID_WHITE_BALANCE_TEMPERATURE, (settings.p_whiteBalanceTemperature.get()));
		applySetting(V4L2_CID_EXPOSURE_AUTO_PRIORITY, (settings.p_exposureAutoPriority.get()));
		//Other controls
		applySetting(V4L2_CID_BRIGHTNESS, (settings.p_brightness.get()));
		applySetting(V4L2_CID_CONTRAST, (settings.p_contrast.get()));
		applySetting(V4L2_CID_SATURATION, (settings.p_saturation.get()));
		applySetting(V4L2_CID_GAIN, (settings.p_gain.get()));
		applySetting(V4L2_CID_EXPOSURE_ABSOLUTE, (settings.p_exposureAbsolute.get()));
		applySetting(V4L2_CID_POWER_LINE_FREQUENCY, (settings.p_powerLineFrequency.get()));
		applySetting(V4L2_CID_SHARPNESS, (settings.p_sharpness.get()));

		//COPIES INTO OLD FORMAT:
		settings.copyParams();
	}

	/*
	void DarwinCamera::loadCameraOffset()
	{
		std::ifstream file((CONFIG_DIR + string("CameraOffsets.cfg")).c_str());
		if (file.is_open())
		{
			std::string macaddress = Platform->getMacAddress();
			while (!file.eof())
			{
				std::string buffer;
				std::string addr_buffer;
				float offset_buffer;

				std::getline(file, buffer);
				std::stringstream ss(buffer);
				std::getline(ss, addr_buffer, ':');
				ss >> offset_buffer;

				if (macaddress.compare(addr_buffer) == 0)
				{
					CameraOffset = offset_buffer;
					break;
				}
			}
		}
		else
		{
			errorlog << "DarwinCamera::loadCameraOffset(). Unable to load camera offset." << endl;
		}
	}
	*/

}
