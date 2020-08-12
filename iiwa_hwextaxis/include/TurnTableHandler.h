#pragma once
//wrapper xilab controller
#include <ximc.h>
#include <cstdio>

class TurnTableHandler
{
private:
	device_t device = -1;
	engine_settings_t engine_settings;
	engine_settings_calb_t engine_settings_calb;
	calibration_t calibration;
	static constexpr double Deg2Units = 100.0;
	const wchar_t* error_string(result_t result)
	{
		switch (result)
		{
		case result_error:				return L"error";
		case result_not_implemented:	return L"not implemented";
		case result_nodevice:			return L"no device";
		default:						return L"success";
		}
	}
public:
	bool connected = false;
	TurnTableHandler(/*default params*/) { ; };
	//connect & initialize with defaul params
	~TurnTableHandler() {
		//this->Disconnect();
	};



	bool Connect() {
		int names_count;
		char device_name[256];
		const int probe_flags = ENUMERATE_PROBE | ENUMERATE_NETWORK;
		const char* enumerate_hints = "addr=192.168.1.1,172.16.2.3";
		//// const char* enumerate_hints = "addr="; // this hint will use broadcast enumeration, if ENUMERATE_NETWORK flag is enabled
		char ximc_version_str[32];
		//const int seconds = 3;
		device_enumeration_t devenum;

		printf("This is a ximc test program.\n");
		//	ximc_version returns library version string.
		ximc_version(ximc_version_str);
		printf("libximc version %s\n", ximc_version_str);

		//  Set bindy (network) keyfile. Must be called before any call to "enumerate_devices" or "open_device" if you
		//  wish to use network-attached controllers. Accepts both absolute and relative paths, relative paths are resolved
		//  relative to the process working directory. If you do not need network devices then "set_bindy_key" is optional.
		set_bindy_key("/home/ros-industrial/Desktop/ximc-2.12.1/ximc/keyfile.sqlite");

		//	Device enumeration function. Returns an opaque pointer to device enumeration data.
		devenum = enumerate_devices(probe_flags, enumerate_hints);

		//	Gets device count from device enumeration data
		names_count = get_device_count(devenum);

		//	Terminate if there are no connected devices
		if (names_count <= 0)
		{
			printf("No devices found\n");
			//	Free memory used by device enumeration data
			free_enumerate_devices(devenum);
			return false;
		}

		//	Copy first found device name into a string
		strcpy(device_name, get_device_name(devenum, 0));
		//	Free memory used by device enumeration data
		free_enumerate_devices(devenum);

		printf("Opening device...");
		//	Open device by device name
		device = open_device(device_name);
		printf("done.\n");
		return (this->connected = true);
	}
	bool Disconnect() {
		if (device != -1) {
			command_stop(device);
			close_device(&device);
		}
		return (this->connected = false);
	}
	bool gotoHome() {
		int result = -1;
		result = command_homezero(device);
		if ((result = command_wait_for_stop(device, 100)) != result_ok)
			wprintf(L"error command_wait_for_stop %ls\n", error_string(result));
		return result == 0;
	}
	bool MovePos(double degPos) {
		int result = -1;

		if ((result = command_move(device, degPos * Deg2Units, 0)) != result_ok) {
			wprintf(L"error command_move_corr %ls\n", error_string(result));
			return true;
		}
		else
			wprintf(L"command_move_corr  %ls\n", error_string(result));

		if ((result = command_wait_for_stop(device, 100)) != result_ok) {
			wprintf(L"error command_wait_for_stop %ls\n", error_string(result));
			return false;
		}
		return true;
	}
	double getPos() {
		get_position_t getpos;
		get_position(device, &getpos);

		return (double)getpos.Position / Deg2Units;
	}
};
